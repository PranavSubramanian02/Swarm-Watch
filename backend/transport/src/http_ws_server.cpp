// Zero-dependency POSIX HTTP/1.1 + RFC 6455 WebSocket server with AuthGuard and InputSanitizer

#include "http_ws_server.hpp"
#include "input_sanitizer.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

namespace swarm {

namespace {

// Minimal self-contained SHA-1 for RFC 6455 Sec-WebSocket-Accept handshake
std::array<uint8_t, 20> sha1_digest(const std::string& input) {
    uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE, h3 = 0x10325476, h4 = 0xC3D2E1F0;
    std::vector<uint8_t> msg(input.begin(), input.end());
    uint64_t bit_len = static_cast<uint64_t>(msg.size()) * 8ULL;
    msg.push_back(0x80);
    while ((msg.size() % 64) != 56) msg.push_back(0x00);
    for (int i = 7; i >= 0; --i) msg.push_back(static_cast<uint8_t>((bit_len >> (i * 8)) & 0xFF));

    auto rol = [](uint32_t v, int bits) { return (v << bits) | (v >> (32 - bits)); };
    for (size_t off = 0; off < msg.size(); off += 64) {
        uint32_t w[80]{};
        for (int i = 0; i < 16; ++i) {
            w[i] = (uint32_t(msg[off + 4 * i]) << 24) | (uint32_t(msg[off + 4 * i + 1]) << 16) |
                   (uint32_t(msg[off + 4 * i + 2]) << 8) | uint32_t(msg[off + 4 * i + 3]);
        }
        for (int i = 16; i < 80; ++i) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        for (int i = 0; i < 80; ++i) {
            uint32_t f = 0, k = 0;
            if (i < 20) { f = (b & c) | ((~b) & d); k = 0x5A827999; }
            else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
            else { f = b ^ c ^ d; k = 0xCA62C1D6; }
            uint32_t tmp = rol(a, 5) + f + e + k + w[i];
            e = d; d = c; c = rol(b, 30); b = a; a = tmp;
        }
        h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    }
    std::array<uint8_t, 20> out{};
    uint32_t hs[5] = {h0, h1, h2, h3, h4};
    for (int i = 0; i < 5; ++i) {
        out[4 * i] = (hs[i] >> 24) & 0xFF;
        out[4 * i + 1] = (hs[i] >> 16) & 0xFF;
        out[4 * i + 2] = (hs[i] >> 8) & 0xFF;
        out[4 * i + 3] = hs[i] & 0xFF;
    }
    return out;
}

std::string base64_encode(const uint8_t* data, size_t len) {
    static const char* kTbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (size_t i = 0; i < len; i += 3) {
        uint32_t n = (uint32_t(data[i]) << 16) |
                     (i + 1 < len ? uint32_t(data[i + 1]) << 8 : 0) |
                     (i + 2 < len ? uint32_t(data[i + 2]) : 0);
        out.push_back(kTbl[(n >> 18) & 63]);
        out.push_back(kTbl[(n >> 12) & 63]);
        out.push_back(i + 1 < len ? kTbl[(n >> 6) & 63] : '=');
        out.push_back(i + 2 < len ? kTbl[n & 63] : '=');
    }
    return out;
}

std::string mime_type(const std::string& p) {
    if (p.ends_with(".html")) return "text/html; charset=utf-8";
    if (p.ends_with(".css")) return "text/css; charset=utf-8";
    if (p.ends_with(".js")) return "application/javascript; charset=utf-8";
    if (p.ends_with(".json")) return "application/json";
    if (p.ends_with(".svg")) return "image/svg+xml";
    if (p.ends_with(".png")) return "image/png";
    if (p.ends_with(".csv")) return "text/csv";
    return "application/octet-stream";
}

bool send_all(int fd, const void* buf, size_t len) {
    const char* p = static_cast<const char*>(buf);
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(fd, p + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

std::string trim_cr(std::string s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ')) s.pop_back();
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    return s.substr(i);
}

}  // namespace

HttpWsServer::HttpWsServer(SwarmEngine& engine, std::string web_dir, int port, std::string auth_token)
    : engine_(engine), web_dir_(std::move(web_dir)), port_(port), auth_(std::move(auth_token)) {}

HttpWsServer::~HttpWsServer() { stop(); }

bool HttpWsServer::start() {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) return false;
    int opt = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    if (::listen(listen_fd_, 32) < 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    running_ = true;
    accept_thread_ = std::thread(&HttpWsServer::accept_loop, this);
    return true;
}

void HttpWsServer::stop() {
    if (!running_.exchange(false)) return;
    if (listen_fd_ >= 0) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    if (accept_thread_.joinable()) accept_thread_.join();
    std::lock_guard<std::mutex> lock(ws_mtx_);
    for (int fd : ws_clients_) ::close(fd);
    ws_clients_.clear();
}

void HttpWsServer::accept_loop() {
    while (running_) {
        sockaddr_in caddr{};
        socklen_t clen = sizeof(caddr);
        int cfd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&caddr), &clen);
        if (cfd < 0) continue;
        std::thread(&HttpWsServer::handle_client, this, cfd).detach();
    }
}

void HttpWsServer::handle_client(int fd) {
    timeval tv{2, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::string raw;
    char buf[4096];
    ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
    if (n <= 0) { ::close(fd); return; }
    raw.append(buf, static_cast<size_t>(n));

    auto hdr_end = raw.find("\r\n\r\n");
    if (hdr_end == std::string::npos) { ::close(fd); return; }

    std::istringstream ss(raw.substr(0, hdr_end));
    std::string req_line;
    std::getline(ss, req_line);
    std::istringstream rl(req_line);
    std::string method, full_path;
    rl >> method >> full_path;

    std::string ws_key, token_hdr;
    size_t content_len = 0;
    std::string line;
    while (std::getline(ss, line)) {
        auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string k = line.substr(0, colon);
        std::transform(k.begin(), k.end(), k.begin(), ::tolower);
        std::string v = trim_cr(line.substr(colon + 1));
        if (k == "sec-websocket-key") ws_key = v;
        else if (k == "x-swarm-token") token_hdr = v;
        else if (k == "content-length") content_len = static_cast<size_t>(std::stoul(v));
    }

    // Extract optional ?token= query parameter
    std::string path = full_path;
    auto qpos = full_path.find('?');
    if (qpos != std::string::npos) {
        path = full_path.substr(0, qpos);
        std::string qs = full_path.substr(qpos + 1);
        auto tpos = qs.find("token=");
        if (tpos != std::string::npos && token_hdr.empty()) {
            auto amp = qs.find('&', tpos + 6);
            token_hdr = qs.substr(tpos + 6, amp == std::string::npos ? std::string::npos : amp - tpos - 6);
        }
    }

    // Handle WebSocket upgrade for /ws/swarm
    if (!ws_key.empty() && path == "/ws/swarm") {
        if (!auth_.verify_token(token_hdr)) {
            send_http_response(fd, 401, "application/json", "{\"error\":\"unauthorized\"}");
            ::close(fd);
            return;
        }
        if (upgrade_websocket(fd, ws_key)) {
            std::lock_guard<std::mutex> lock(ws_mtx_);
            ws_clients_.push_back(fd);
            return;  // Keep socket open for live broadcasts
        }
        ::close(fd);
        return;
    }

    // Read remaining POST body if needed
    std::string body = raw.substr(hdr_end + 4);
    while (body.size() < content_len && body.size() < 8192) {
        ssize_t r = ::recv(fd, buf, sizeof(buf), 0);
        if (r <= 0) break;
        body.append(buf, static_cast<size_t>(r));
    }

    handle_http_request(fd, method, path, token_hdr, body);
    ::close(fd);
}

bool HttpWsServer::upgrade_websocket(int fd, const std::string& ws_key) {
    auto hash = sha1_digest(ws_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11");
    std::string accept = base64_encode(hash.data(), hash.size());
    std::string resp =
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + accept + "\r\n\r\n";
    return send_all(fd, resp.data(), resp.size());
}

void HttpWsServer::send_ws_text(int fd, const std::string& payload) {
    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 10);
    frame.push_back(0x81);  // FIN + text opcode
    size_t len = payload.size();
    if (len < 126) {
        frame.push_back(static_cast<uint8_t>(len));
    } else if (len < 65536) {
        frame.push_back(126);
        frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(len & 0xFF));
    } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; --i) frame.push_back(static_cast<uint8_t>((len >> (i * 8)) & 0xFF));
    }
    frame.insert(frame.end(), payload.begin(), payload.end());
    if (!send_all(fd, frame.data(), frame.size())) {
        ::close(fd);
    }
}

void HttpWsServer::broadcast_state() {
    std::lock_guard<std::mutex> lock(ws_mtx_);
    if (ws_clients_.empty()) return;
    std::string json = engine_.serialize_state_json();

    std::vector<uint8_t> frame;
    frame.reserve(json.size() + 10);
    frame.push_back(0x81);
    size_t len = json.size();
    if (len < 126) {
        frame.push_back(static_cast<uint8_t>(len));
    } else if (len < 65536) {
        frame.push_back(126);
        frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(len & 0xFF));
    } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; --i) frame.push_back(static_cast<uint8_t>((len >> (i * 8)) & 0xFF));
    }
    frame.insert(frame.end(), json.begin(), json.end());

    std::vector<int> alive_clients;
    for (int fd : ws_clients_) {
        if (send_all(fd, frame.data(), frame.size())) alive_clients.push_back(fd);
        else ::close(fd);
    }
    ws_clients_ = std::move(alive_clients);
}

void HttpWsServer::send_http_response(int fd, int code, const std::string& content_type, const std::string& body) {
    const char* status_txt = (code == 200) ? "OK" : (code == 401 ? "Unauthorized" : (code == 429 ? "Too Many Requests" : (code == 400 ? "Bad Request" : "Not Found")));
    std::ostringstream oss;
    oss << "HTTP/1.1 " << code << " " << status_txt << "\r\n"
        << "Content-Type: " << content_type << "\r\n"
        << "Content-Length: " << body.size() << "\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        << "Access-Control-Allow-Headers: Content-Type, X-Swarm-Token\r\n"
        << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
        << "Cache-Control: no-store\r\n"
        << "Connection: close\r\n\r\n";
    std::string hdr = oss.str();
    send_all(fd, hdr.data(), hdr.size());
    if (!body.empty()) send_all(fd, body.data(), body.size());
}

void HttpWsServer::handle_http_request(int fd, const std::string& method, const std::string& path,
                                       const std::string& token_hdr, const std::string& body) {
    if (method == "OPTIONS") {
        send_http_response(fd, 200, "text/plain", "");
        return;
    }
    if (path == "/api/v1/health") {
        send_http_response(fd, 200, "application/json", "{\"status\":\"ok\",\"engine\":\"cpp20_swarm\"}");
        return;
    }
    if (path == "/api/v1/state") {
        send_http_response(fd, 200, "application/json", engine_.serialize_state_json());
        return;
    }
    if (path == "/api/v1/telemetry.csv") {
        send_http_response(fd, 200, "text/csv", engine_.export_csv_log());
        return;
    }
    if (method == "POST" && path == "/api/v1/inject") {
        if (!auth_.verify_token(token_hdr)) {
            send_http_response(fd, 401, "application/json", "{\"error\":\"unauthorized\"}");
            return;
        }
        if (!auth_.allow_command()) {
            send_http_response(fd, 429, "application/json", "{\"error\":\"rate_limited\"}");
            return;
        }
        SanitizedInjectCmd cmd = sanitize_inject_payload(body, kMaxUavs);
        if (!cmd.valid) {
            send_http_response(fd, 400, "application/json", "{\"error\":\"invalid_payload\"}");
            return;
        }
        if (cmd.action == "preset") engine_.apply_preset_scenario(cmd.text_param);
        else if (cmd.action == "actuator_fault") engine_.inject_actuator_fault(cmd.uav_id, cmd.eta_roll, cmd.eta_thrust, cmd.roll_bias_rad);
        else if (cmd.action == "icing") engine_.inject_icing(cmd.uav_id, cmd.icing_level);
        else if (cmd.action == "ew_jamming") engine_.set_ew_jamming(cmd.flag, cmd.val1, cmd.val2, std::max(120.0, cmd.val3));
        else if (cmd.action == "gnss_spoof") engine_.set_gnss_spoofing(cmd.flag);
        else if (cmd.action == "rotate_master") engine_.force_master_rotate();
        else if (cmd.action == "kill_uav") engine_.kill_uav(cmd.uav_id);
        else if (cmd.action == "disconnect_uav") engine_.disconnect_uav_comms(cmd.uav_id, cmd.flag);
        else if (cmd.action == "wind") engine_.set_wind(cmd.val1, cmd.val2, 0.0);
        else if (cmd.action == "formation") engine_.set_formation(static_cast<FormationType>(std::clamp(static_cast<int>(cmd.val1), 0, 3)));
        else if (cmd.action == "add_task") engine_.add_dynamic_task(cmd.val1, cmd.val2, cmd.val3);
        else if (cmd.action == "pause") engine_.set_paused(cmd.flag);
        else if (cmd.action == "reset") engine_.reset(SimOptions{});
        send_http_response(fd, 200, "application/json", "{\"status\":\"applied\",\"action\":\"" + cmd.action + "\"}");
        return;
    }
    if (method == "POST" && path == "/api/v1/ablation") {
        auto res = SwarmEngine::run_icaav_ablation(60);
        std::ostringstream o;
        o << std::fixed << std::setprecision(2) << "{\"variants\":[";
        for (size_t i = 0; i < res.size(); ++i) {
            if (i > 0) o << ",";
            const auto& v = res[i];
            o << "{\"label\":\"" << v.label << "\",\"n_runs\":" << v.n_runs
              << ",\"rmse_m\":" << v.mean_rmse_m << ",\"peak_trace_p\":" << v.peak_trace_p
              << ",\"nees_ok_pct\":" << v.nees_in_bounds_pct << ",\"completion_pct\":" << v.mission_completion_pct
              << ",\"collision_pct\":" << v.collision_rate_pct << ",\"soc_spread_pct\":" << v.soc_spread_pct
              << ",\"min_soc_pct\":" << v.min_soc_pct << ",\"cbba_savings_pct\":" << v.cbba_packet_savings_pct
              << ",\"mean_xte_m\":" << v.mean_xte_m << "}";
        }
        o << "]}";
        send_http_response(fd, 200, "application/json", o.str());
        return;
    }

    // Serve static web_console files safely
    std::string rel = (path == "/" ? "/index.html" : path);
    if (!is_safe_static_subpath(rel)) {
        send_http_response(fd, 400, "text/plain", "Bad Path");
        return;
    }
    std::string full = web_dir_ + rel;
    std::ifstream ifs(full, std::ios::binary);
    if (!ifs) {
        send_http_response(fd, 404, "text/plain", "Not Found");
        return;
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();
    send_http_response(fd, 200, mime_type(full), ss.str());
}

}  // namespace swarm

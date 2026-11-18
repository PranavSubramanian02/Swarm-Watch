#pragma once

// Zero-dependency C++20 multi-threaded HTTP/1.1 + RFC 6455 WebSocket server

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "auth_guard.hpp"
#include "swarm_engine.hpp"

namespace swarm {

class HttpWsServer {
public:
    HttpWsServer(SwarmEngine& engine, std::string web_dir, int port = 8080, std::string auth_token = "");
    ~HttpWsServer();

    bool start();
    void stop();
    void broadcast_state();

private:
    void accept_loop();
    void handle_client(int client_fd);
    void handle_http_request(int fd, const std::string& method, const std::string& path,
                             const std::string& token_hdr, const std::string& body);
    void send_http_response(int fd, int code, const std::string& content_type, const std::string& body);
    bool upgrade_websocket(int fd, const std::string& ws_key);
    void send_ws_text(int fd, const std::string& payload);

    SwarmEngine& engine_;
    std::string web_dir_;
    int port_;
    AuthGuard auth_;
    int listen_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread accept_thread_;
    std::mutex ws_mtx_;
    std::vector<int> ws_clients_;
};

}  // namespace swarm

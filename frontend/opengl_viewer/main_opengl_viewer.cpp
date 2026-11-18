// Native 60 FPS C++20 OpenGL/GLFW 3D tactical flight-sim viewer paired with live :8080 WS server

#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <cmath>
#include <iostream>
#include <string>
#include "aero_plant.hpp"
#include "http_ws_server.hpp"
#include "swarm_engine.hpp"

namespace {

struct CameraState {
    double yaw = -0.75;
    double pitch = 0.52;
    double dist = 2100.0;
    int mode = 0;  // 0: Free Orbit, 1: Master Chase-Cam, 2: Top-Down Radar
    bool dragging = false;
    double last_x = 0.0, last_y = 0.0;
};

CameraState g_cam;
swarm::SwarmEngine* g_engine = nullptr;

// Minimal gluPerspective replacement using pure OpenGL glFrustum
void set_perspective(double fovy_deg, double aspect, double znear, double zfar) {
    double fh = std::tan(fovy_deg * swarm::kPi / 360.0) * znear;
    double fw = fh * aspect;
    glFrustum(-fw, fw, -fh, fh, znear, zfar);
}

// Minimal gluLookAt replacement using pure OpenGL glMultMatrixd
void set_look_at(const swarm::Vec3& eye, const swarm::Vec3& center, const swarm::Vec3& up) {
    swarm::Vec3 f = (center - eye).normalized();
    swarm::Vec3 s = f.cross(up).normalized();
    swarm::Vec3 u = s.cross(f);
    double m[16] = {
        s.x, u.x, -f.x, 0.0,
        s.y, u.y, -f.y, 0.0,
        s.z, u.z, -f.z, 0.0,
        -s.dot(eye), -u.dot(eye), f.dot(eye), 1.0
    };
    glMultMatrixd(m);
}

void draw_wire_sphere(double cx, double cy, double cz, double r, int segs) {
    for (int ring = 0; ring < 3; ++ring) {
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < segs; ++i) {
            double a = 2.0 * swarm::kPi * i / segs;
            if (ring == 0) glVertex3d(cx + r * std::cos(a), cy + r * std::sin(a), cz);
            else if (ring == 1) glVertex3d(cx + r * std::cos(a), cy, cz + r * std::sin(a));
            else glVertex3d(cx, cy + r * std::cos(a), cz + r * std::sin(a));
        }
        glEnd();
    }
}

void draw_fixed_wing_mesh(const swarm::Vec3& pos_gl, const swarm::Quat& q_nb, bool is_master, bool dr_active, bool faulted) {
    // Convert NED quaternion to GL right-handed (X=East, Y=Up, Z=-North)
    swarm::Vec3 eul = q_nb.to_euler();
    glPushMatrix();
    glTranslated(pos_gl.x, pos_gl.y, pos_gl.z);
    glRotated(-eul.z * 180.0 / swarm::kPi, 0.0, 1.0, 0.0);
    glRotated(eul.y * 180.0 / swarm::kPi, 1.0, 0.0, 0.0);
    glRotated(-eul.x * 180.0 / swarm::kPi, 0.0, 0.0, 1.0);

    if (is_master) glColor3f(0.98f, 0.80f, 0.08f);
    else if (faulted) glColor3f(0.95f, 0.28f, 0.28f);
    else if (dr_active) glColor3f(0.96f, 0.62f, 0.05f);
    else glColor3f(0.22f, 0.74f, 0.97f);

    // Swept fixed-wing airframe triangles (nose along -Z)
    glBegin(GL_TRIANGLES);
    glVertex3d(0.0, 0.0, -34.0); glVertex3d(-44.0, 0.0, 12.0); glVertex3d(44.0, 0.0, 12.0);
    glVertex3d(0.0, 10.0, 16.0); glVertex3d(0.0, 0.0, -18.0); glVertex3d(0.0, 0.0, 26.0);
    glVertex3d(0.0, 0.0, 26.0); glVertex3d(-16.0, 0.0, 32.0); glVertex3d(16.0, 0.0, 32.0);
    glEnd();
    glPopMatrix();
}

void on_mouse_btn(GLFWwindow* win, int btn, int action, int) {
    if (btn == GLFW_MOUSE_BUTTON_LEFT) {
        g_cam.dragging = (action == GLFW_PRESS);
        glfwGetCursorPos(win, &g_cam.last_x, &g_cam.last_y);
    }
}

void on_cursor(GLFWwindow*, double x, double y) {
    if (!g_cam.dragging) return;
    g_cam.yaw += (x - g_cam.last_x) * 0.006;
    g_cam.pitch = swarm::clampd(g_cam.pitch + (y - g_cam.last_y) * 0.006, 0.12, 1.48);
    g_cam.last_x = x;
    g_cam.last_y = y;
}

void on_scroll(GLFWwindow*, double, double yoff) {
    g_cam.dist = swarm::clampd(g_cam.dist * (yoff > 0 ? 0.9 : 1.1), 350.0, 4200.0);
}

void on_key(GLFWwindow*, int key, int, int action, int) {
    if (action != GLFW_PRESS || !g_engine) return;
    if (key == GLFW_KEY_1) g_cam.mode = 0;
    else if (key == GLFW_KEY_2) g_cam.mode = 1;
    else if (key == GLFW_KEY_3) g_cam.mode = 2;
    else if (key == GLFW_KEY_SPACE) g_engine->set_paused(!g_engine->is_paused());
    else if (key == GLFW_KEY_R) g_engine->force_master_rotate();
    else if (key == GLFW_KEY_J) g_engine->apply_preset_scenario("gnss_blackout");
    else if (key == GLFW_KEY_S) g_engine->apply_preset_scenario("gnss_spoof");
    else if (key == GLFW_KEY_F) g_engine->apply_preset_scenario("actuator_fault");
    else if (key == GLFW_KEY_D) g_engine->apply_preset_scenario("disconnect_uav");
    else if (key == GLFW_KEY_K) g_engine->apply_preset_scenario("master_attrition");
    else if (key == GLFW_KEY_0) g_engine->apply_preset_scenario("nominal");
}

}  // namespace

int main(int argc, char** argv) {
    std::string web_dir = "frontend/web_console";
    int port = 8080;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--web-dir" && i + 1 < argc) web_dir = argv[++i];
        else if (a == "--port" && i + 1 < argc) port = std::stoi(argv[++i]);
    }

    if (!glfwInit()) {
        std::cerr << "GLFW init failed (requires graphical display)\n";
        return 1;
    }
    GLFWwindow* win = glfwCreateWindow(1360, 840, "Survivable Multi-UAV Swarm — Native C++20 OpenGL 3D Viewer", nullptr, nullptr);
    if (!win) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    glfwSetMouseButtonCallback(win, on_mouse_btn);
    glfwSetCursorPosCallback(win, on_cursor);
    glfwSetScrollCallback(win, on_scroll);
    glfwSetKeyCallback(win, on_key);

    swarm::SwarmEngine engine;
    g_engine = &engine;
    swarm::HttpWsServer server(engine, web_dir, port);
    server.start();

    std::cout << "Native C++20 OpenGL 3D Viewer active | Keys: [1]Orbit [2]ChaseMaster [3]TopDown [Space]Pause "
              << "[R]RotateMaster [J]Jam [S]Spoof [F]Fault [D]Disconnect [K]Attrition [0]Reset\n";

    int frame_cnt = 0;
    while (!glfwWindowShouldClose(win)) {
        engine.step();
        if (++frame_cnt % 3 == 0) server.broadcast_state();

        std::array<swarm::UavTrueState, swarm::kMaxUavs> fleet{};
        std::array<swarm::EskfState, swarm::kMaxUavs> beliefs{};
        std::array<swarm::DeviationMetrics, swarm::kMaxUavs> devs{};
        std::array<swarm::L1AdaptiveState, swarm::kMaxUavs> l1{};
        std::array<swarm::MdpMode, swarm::kMaxUavs> modes{};
        std::vector<swarm::IsrTask> tasks;
        swarm::MasterSlaveState ms{};
        swarm::RiskFieldConfig rcfg{};
        std::array<std::deque<swarm::Vec3>, swarm::kMaxUavs> trails{};
        double sim_t = 0.0;
        int n_uavs = 5;
        engine.copy_render_snapshot(fleet, beliefs, devs, l1, modes, tasks, ms, rcfg, trails, sim_t, n_uavs);

        int w = 1360, h = 840;
        glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, std::max(1, h));
        glClearColor(0.03f, 0.05f, 0.09f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        set_perspective(48.0, static_cast<double>(w) / std::max(1, h), 10.0, 9000.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        auto ned_to_gl = [](const swarm::Vec3& ned) { return swarm::Vec3{ned.y, -ned.z, -ned.x}; };
        swarm::Vec3 target{0.0, 120.0, 0.0};
        swarm::Vec3 eye{
            g_cam.dist * std::cos(g_cam.pitch) * std::sin(g_cam.yaw),
            g_cam.dist * std::sin(g_cam.pitch),
            g_cam.dist * std::cos(g_cam.pitch) * std::cos(g_cam.yaw)
        };
        if (g_cam.mode == 1 && ms.active_master >= 0 && fleet[ms.active_master].alive) {
            target = ned_to_gl(fleet[ms.active_master].pos_ned);
            eye = target + swarm::Vec3{260.0, 140.0, 260.0};
        } else if (g_cam.mode == 2) {
            eye = {0.0, 3200.0, 20.0};
            target = {0.0, 0.0, 0.0};
        }
        set_look_at(eye, target, {0.0, 1.0, 0.0});

        // 1. 3D Sculpted thermal risk-field wireframe terrain
        const int kGrid = 32;
        const double kHalf = 1400.0;
        const double step = (2.0 * kHalf) / kGrid;
        glLineWidth(1.0f);
        for (int i = 0; i < kGrid; ++i) {
            glBegin(GL_LINE_STRIP);
            for (int j = 0; j <= kGrid; ++j) {
                double n = -kHalf + i * step;
                double e = -kHalf + j * step;
                double rv = swarm::evaluate_risk_field(rcfg, n, e).value;
                double elev = std::min(110.0, rv * 11.0);
                float heat = static_cast<float>(swarm::clampd(rv / 7.5, 0.0, 1.0));
                glColor4f(0.10f + 0.85f * heat, 0.22f + 0.25f * (1.0f - heat), 0.38f * (1.0f - heat), 0.55f);
                glVertex3d(e, elev, -n);
            }
            glEnd();
        }

        // 2. Red EW GNSS-Jamming cylinder ring
        glColor4f(0.95f, 0.22f, 0.22f, 0.55f);
        glLineWidth(2.0f);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 48; ++i) {
            double a = 2.0 * swarm::kPi * i / 48.0;
            glVertex3d(rcfg.jam_east + rcfg.jam_radius_m * std::cos(a), 140.0, -(rcfg.jam_north + rcfg.jam_radius_m * std::sin(a)));
        }
        glEnd();

        // 3. ISR target loiter rings
        for (const auto& t : tasks) {
            glColor4f(t.completed ? 0.10f : 0.22f, t.completed ? 0.85f : 0.74f, t.completed ? 0.52f : 0.97f, 0.7f);
            glBegin(GL_LINE_LOOP);
            for (int i = 0; i < 24; ++i) {
                double a = 2.0 * swarm::kPi * i / 24.0;
                glVertex3d(t.pos_ned.y + t.loiter_radius_m * std::cos(a), -t.pos_ned.z, -(t.pos_ned.x + t.loiter_radius_m * std::sin(a)));
            }
            glEnd();
        }

        // 4. Multi-hop RF mesh beams (green=1-hop, amber=2-hop relay)
        glLineWidth(1.5f);
        glBegin(GL_LINES);
        for (int i = 0; i < n_uavs; ++i) {
            for (int j = i + 1; j < n_uavs; ++j) {
                int hops = ms.link_hop_matrix[i][j];
                if (hops == 0 || !fleet[i].alive || !fleet[j].alive) continue;
                if (hops == 1) glColor4f(0.08f, 0.82f, 0.55f, 0.55f);
                else glColor4f(0.96f, 0.65f, 0.12f, 0.50f);
                auto p1 = ned_to_gl(fleet[i].pos_ned);
                auto p2 = ned_to_gl(fleet[j].pos_ned);
                glVertex3d(p1.x, p1.y, p1.z);
                glVertex3d(p2.x, p2.y, p2.z);
            }
        }
        glEnd();

        // 5. UAV trails, 3-sigma ESKF ellipsoids, ideal-vs-actual deviation vectors, and 3D fixed-wing meshes
        for (int u = 0; u < n_uavs; ++u) {
            if (!fleet[u].alive) continue;
            bool is_m = (u == ms.active_master);
            auto p_gl = ned_to_gl(fleet[u].pos_ned);

            // Actual ribbon trail
            glLineWidth(is_m ? 2.6f : 1.8f);
            glColor4f(is_m ? 0.98f : 0.22f, is_m ? 0.80f : 0.74f, is_m ? 0.10f : 0.97f, 0.85f);
            glBegin(GL_LINE_STRIP);
            for (const auto& pt : trails[u]) {
                auto g = ned_to_gl(pt);
                glVertex3d(g.x, g.y, g.z);
            }
            glEnd();

            // 3-sigma covariance wireframe sphere
            glLineWidth(1.0f);
            glColor4f(beliefs[u].dr_active ? 0.96f : 0.22f, beliefs[u].dr_active ? 0.55f : 0.78f, 0.97f, 0.35f);
            draw_wire_sphere(p_gl.x, p_gl.y, p_gl.z, swarm::clampd(beliefs[u].pos_sigma_3 * 2.5, 18.0, 140.0), 18);

            // Rotating Master gold halo
            if (is_m) {
                glColor4f(0.98f, 0.80f, 0.08f, 0.9f);
                glLineWidth(2.4f);
                glBegin(GL_LINE_LOOP);
                for (int i = 0; i < 32; ++i) {
                    double a = 2.0 * swarm::kPi * i / 32.0;
                    glVertex3d(p_gl.x + 62.0 * std::cos(a), p_gl.y, p_gl.z + 62.0 * std::sin(a));
                }
                glEnd();
            }

            draw_fixed_wing_mesh(p_gl, fleet[u].q_nb, is_m, beliefs[u].dr_active, l1[u].fault_flag);
        }

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    server.stop();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}

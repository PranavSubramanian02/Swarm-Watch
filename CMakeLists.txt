cmake_minimum_required(VERSION 3.16)
project(SurvivableSwarmAutonomy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL QUIET)
find_package(glfw3 QUIET)

# Static C++20 flight software, security, and transport library
add_library(swarm_core STATIC
    backend/core/src/aero_plant.cpp
    backend/core/src/eskf_15state.cpp
    backend/core/src/l1_adaptive_ftc.cpp
    backend/core/src/guidance_cbf_mdp.cpp
    backend/core/src/swarm_coordination.cpp
    backend/core/src/swarm_engine.cpp
    backend/security/src/auth_guard.cpp
    backend/security/src/input_sanitizer.cpp
    backend/security/src/cyber_ew_defense.cpp
    backend/transport/src/http_ws_server.cpp
)
target_include_directories(swarm_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/backend/core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/backend/security/include
    ${CMAKE_CURRENT_SOURCE_DIR}/backend/transport/include
)
target_compile_options(swarm_core PRIVATE -O3 -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(swarm_core PUBLIC OpenMP::OpenMP_CXX Threads::Threads)

# Headless C++20 HTTP/WebSocket swarm server and CLI ablation runner
add_executable(swarm_server backend/transport/src/main_server.cpp)
target_link_libraries(swarm_server PRIVATE swarm_core)

# Native 60 FPS C++20 OpenGL/GLFW 3D tactical flight-sim viewer + live server
if(OpenGL_FOUND AND glfw3_FOUND)
    add_executable(swarm_opengl_viewer frontend/opengl_viewer/main_opengl_viewer.cpp)
    target_link_libraries(swarm_opengl_viewer PRIVATE swarm_core OpenGL::GL glfw)
    message(STATUS "Enabled native OpenGL/GLFW viewer: swarm_opengl_viewer")
else()
    message(WARNING "OpenGL or GLFW3 missing; skipping swarm_opengl_viewer")
endif()

# CTest physics, autonomy, and security verification suite
enable_testing()
add_executable(test_swarm_core backend/tests/test_swarm_core.cpp)
target_link_libraries(test_swarm_core PRIVATE swarm_core)
add_test(NAME SwarmCoreAndSecurityTests COMMAND test_swarm_core)

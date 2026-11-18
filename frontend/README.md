# Dual Tactical Visualizers (`frontend/`)

The `frontend/` directory houses two synchronized visualization frontends sharing the same live C++20 swarm engine and `:8080` telemetry stream.

## 1. Native C++20 OpenGL/GLFW 3D Viewer (`frontend/opengl_viewer/`)
`main_opengl_viewer.cpp` renders a native 60 FPS hardware-accelerated 3D tactical airspace window while hosting the embedded `:8080` HTTP/WebSocket server concurrently so the desktop 3D viewport, browser console, and Kotlin Android app observe the identical live swarm.

- **Visual Layers**: Sculpted 3D thermal risk-field wireframe terrain, red EW GNSS-jamming cylinder, ISR loiter orbit rings, quaternion-oriented 3D fixed-wing aircraft meshes, actual flight trails, wireframe $3\sigma$ covariance spheres, rotating Master gold halo, and 1-hop / 2-hop RF mesh beams.
- **Interactive Controls**: Mouse-drag 3D orbit, scroll zoom, and hotkeys (`1` Orbit, `2` Master Chase-Cam, `3` Top-Down Radar, `Space` Pause, `R` Force Master Rotation, `J` EW Jamming, `S` GNSS Spoof, `F` Actuator LoE + Icing, `D` Disconnect UAV-3, `K` Master Attrition, `0` Reset).

```bash
./build/swarm_opengl_viewer --port 8080
```

## 2. Accessible Web Tactical Console (`frontend/web_console/`)
Served directly from C++20 at `http://127.0.0.1:8080/` with zero Node/npm build tooling. Built with pure HTML5, CSS3, Vanilla JavaScript, a Zero-WebGL 3D-perspective / 2D-radar HTML5 Canvas renderer (ensuring full compatibility inside VS Code Simple Browser and desktop browsers), and Plotly.js diagnostics. Includes WCAG AA contrast tokens, ARIA landmarks, full keyboard focus navigation, a live C++20 OpenMP ablation trigger, and a Privacy/Legal/Data-Deletion panel.

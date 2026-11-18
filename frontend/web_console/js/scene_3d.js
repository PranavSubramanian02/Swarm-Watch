// Zero-WebGL 60 FPS interactive 3D-perspective and 2D-radar HTML5 Canvas tactical renderer
// Works reliably inside both VS Code Simple Browser and Chrome/Firefox without WebGL failures

const TacticalScene = {
    canvas: null,
    ctx: null,
    mode: "3d",
    yaw: -0.65,
    pitch: 0.58,
    dragging: false,
    lastX: 0,
    lastY: 0,
    lastFrame: null,

    init(hostId) {
        const host = document.getElementById(hostId);
        host.innerHTML = "";
        const c = document.createElement("canvas");
        c.style.width = "100%";
        c.style.height = "100%";
        c.style.display = "block";
        c.style.cursor = "grab";
        host.appendChild(c);
        this.canvas = c;
        this.ctx = c.getContext("2d");

        c.addEventListener("mousedown", (e) => {
            this.dragging = true;
            this.lastX = e.clientX;
            this.lastY = e.clientY;
            c.style.cursor = "grabbing";
        });
        window.addEventListener("mouseup", () => {
            this.dragging = false;
            c.style.cursor = "grab";
        });
        c.addEventListener("mousemove", (e) => {
            if (!this.dragging || this.mode !== "3d") return;
            this.yaw += (e.clientX - this.lastX) * 0.008;
            this.pitch = Math.max(0.18, Math.min(1.42, this.pitch + (e.clientY - this.lastY) * 0.008));
            this.lastX = e.clientX;
            this.lastY = e.clientY;
            if (this.lastFrame) this.render(this.lastFrame);
        });
    },

    setMode(m) {
        this.mode = m;
        if (this.lastFrame) this.render(this.lastFrame);
    },

    project(n, e, alt, w, h) {
        const half = 1500.0;
        const nx = e / half;
        const ny = n / half;
        const nz = (alt - 120.0) / 360.0;
        if (this.mode === "2d") {
            const s = Math.min(w, h) * 0.44;
            return { x: w * 0.5 + nx * s, y: h * 0.5 - ny * s, scale: s / half };
        }
        const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw);
        const cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
        const x1 = nx * cy - ny * sy;
        const y1 = nx * sy + ny * cy;
        const z2 = nz * cp - y1 * sp;
        const y2 = nz * sp + y1 * cp;
        const persp = 3.0 / (4.1 - y2);
        const sc = Math.min(w, h) * 0.36;
        return { x: w * 0.5 + x1 * sc * persp, y: h * 0.56 - z2 * sc * persp, scale: (sc * persp) / half };
    },

    render(frame) {
        if (!this.canvas || !this.ctx || !frame) return;
        this.lastFrame = frame;
        const rect = this.canvas.getBoundingClientRect();
        const w = Math.max(rect.width, 640);
        const h = Math.max(rect.height, 420);
        if (this.canvas.width !== w || this.canvas.height !== h) {
            this.canvas.width = w;
            this.canvas.height = h;
        }
        const ctx = this.ctx;
        ctx.fillStyle = "#050911";
        ctx.fillRect(0, 0, w, h);

        // 1. Tactical coordinate grid
        ctx.strokeStyle = "#121e33";
        ctx.lineWidth = 1;
        for (let g = -1200; g <= 1200; g += 300) {
            const a = this.project(g, -1200, 0, w, h);
            const b = this.project(g, 1200, 0, w, h);
            ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
            const c = this.project(-1200, g, 0, w, h);
            const d = this.project(1200, g, 0, w, h);
            ctx.beginPath(); ctx.moveTo(c.x, c.y); ctx.lineTo(d.x, d.y); ctx.stroke();
        }

        // 2. Thermal Gaussian hazard peaks
        (frame.hazards || []).forEach((hz) => {
            const p = this.project(hz.n, hz.e, 15, w, h);
            const r = Math.max(18, hz.sig * p.scale * 1.15);
            const grad = ctx.createRadialGradient(p.x, p.y, 4, p.x, p.y, r);
            grad.addColorStop(0, "rgba(248, 113, 113, 0.45)");
            grad.addColorStop(0.55, "rgba(251, 191, 36, 0.22)");
            grad.addColorStop(1, "rgba(251, 191, 36, 0.0)");
            ctx.fillStyle = grad;
            ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2); ctx.fill();
        });

        // 3. EW GNSS jamming zone
        if (frame.jamming) {
            const jp = this.project(frame.jamming.n, frame.jamming.e, 100, w, h);
            const jr = Math.max(24, frame.jamming.r * jp.scale);
            ctx.strokeStyle = frame.jamming.global ? "#f87171" : "rgba(248, 113, 113, 0.7)";
            ctx.lineWidth = frame.jamming.global ? 2.5 : 1.5;
            ctx.setLineDash([6, 4]);
            ctx.beginPath(); ctx.arc(jp.x, jp.y, jr, 0, Math.PI * 2); ctx.stroke();
            ctx.setLineDash([]);
            ctx.fillStyle = "#fecaca";
            ctx.font = "10px monospace";
            ctx.fillText(frame.jamming.global ? "GLOBAL EW JAMMING" : "EW JAM ZONE", jp.x - 36, jp.y - jr - 6);
        }

        // 4. ISR orbit targets
        (frame.tasks || []).forEach((t) => {
            const tp = this.project(t.n, t.e, t.alt, w, h);
            const tr = Math.max(8, t.r * tp.scale);
            ctx.strokeStyle = t.done ? "#34d399" : "#38bdf8";
            ctx.lineWidth = 1.4;
            ctx.beginPath(); ctx.arc(tp.x, tp.y, tr, 0, Math.PI * 2); ctx.stroke();
            ctx.fillStyle = t.done ? "#34d399" : "#cbd5e1";
            ctx.font = "10px monospace";
            ctx.fillText(`T${t.id + 1}`, tp.x + tr + 3, tp.y + 3);
        });

        // 5. Multi-hop RF mesh links
        const uavs = frame.uavs || [];
        (frame.links || []).forEach((lnk) => {
            const ua = uavs[lnk.a], ub = uavs[lnk.b];
            if (!ua || !ub || !ua.alive || !ub.alive) return;
            const pa = this.project(ua.true_pos[0], ua.true_pos[1], ua.true_pos[2], w, h);
            const pb = this.project(ub.true_pos[0], ub.true_pos[1], ub.true_pos[2], w, h);
            ctx.strokeStyle = lnk.hops === 1 ? "rgba(52, 211, 153, 0.42)" : "rgba(251, 191, 36, 0.45)";
            ctx.lineWidth = 1.4;
            if (lnk.hops > 1) ctx.setLineDash([4, 4]);
            ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y); ctx.stroke();
            ctx.setLineDash([]);
        });

        // 6. UAV trails, ideal deviation line, 15s drift cone, 3-sigma ellipse, and rotating Master halo
        uavs.forEach((u) => {
            if (!u.alive) return;
            const p = this.project(u.true_pos[0], u.true_pos[1], u.true_pos[2], w, h);
            const pi = this.project(u.ideal_pos[0], u.ideal_pos[1], u.ideal_pos[2], w, h);

            // Actual trail ribbon
            if (u.trail && u.trail.length > 1) {
                ctx.strokeStyle = u.is_master ? "rgba(250, 204, 21, 0.8)" : "rgba(56, 189, 248, 0.65)";
                ctx.lineWidth = u.is_master ? 2.2 : 1.5;
                ctx.beginPath();
                u.trail.forEach((pt, idx) => {
                    const tp = this.project(pt[0], pt[1], pt[2], w, h);
                    if (idx === 0) ctx.moveTo(tp.x, tp.y);
                    else ctx.lineTo(tp.x, tp.y);
                });
                ctx.stroke();
            }

            // Ideal-to-actual deviation link (dashed)
            ctx.strokeStyle = "rgba(248, 113, 113, 0.7)";
            ctx.lineWidth = 1.2;
            ctx.setLineDash([4, 3]);
            ctx.beginPath(); ctx.moveTo(pi.x, pi.y); ctx.lineTo(p.x, p.y); ctx.stroke();
            ctx.setLineDash([]);

            // 15s predicted drift cone
            if (u.drift_cone && u.drift_cone.length > 0) {
                const lastPt = u.drift_cone[u.drift_cone.length - 1];
                const dp = this.project(lastPt[0], lastPt[1], u.true_pos[2], w, h);
                ctx.strokeStyle = u.dr_active ? "rgba(251, 191, 36, 0.55)" : "rgba(56, 189, 248, 0.28)";
                ctx.lineWidth = 1.2;
                ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(dp.x, dp.y); ctx.stroke();
            }

            // 3-sigma covariance circle
            const sigR = Math.max(8, Math.min(48, u.pos_sigma3 * p.scale * 2.2));
            ctx.strokeStyle = u.dr_active ? "rgba(251, 191, 36, 0.65)" : "rgba(56, 189, 248, 0.45)";
            ctx.lineWidth = 1.1;
            ctx.beginPath(); ctx.arc(p.x, p.y, sigR, 0, Math.PI * 2); ctx.stroke();

            // Master gold halo
            if (u.is_master) {
                ctx.strokeStyle = "#facc15";
                ctx.lineWidth = 2.2;
                ctx.beginPath(); ctx.arc(p.x, p.y, 14, 0, Math.PI * 2); ctx.stroke();
            }

            // Aircraft marker
            ctx.fillStyle = u.is_master ? "#facc15" : (u.l1_fault ? "#f87171" : (!u.connected ? "#fb923c" : "#38bdf8"));
            ctx.beginPath(); ctx.arc(p.x, p.y, 5.5, 0, Math.PI * 2); ctx.fill();
            ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 1.3; ctx.stroke();

            ctx.fillStyle = "#f1f5f9";
            ctx.font = "11px monospace";
            const tag = `U${u.id + 1}${u.is_master ? "[M]" : ""}${u.dr_active ? " DR" : ""}${!u.connected ? " OFF" : ""}`;
            ctx.fillText(tag, p.x + 9, p.y - 6);
        });

        // Legend footer
        ctx.fillStyle = "#cbd5e1";
        ctx.font = "11px monospace";
        ctx.fillText(
            this.mode === "3d"
                ? "Drag to Orbit 3D | Gold: Active Master | Cyan: Slave | Dashed Red: XTE | Green/Amber: 1/2-Hop RF"
                : "2D Tactical Radar | Gold: Active Master | Dashed Red: Ideal-vs-Actual XTE | Circles: 3-Sigma ESKF",
            14, h - 12
        );
    },
};

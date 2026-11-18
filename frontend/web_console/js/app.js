// Main Web Console controller binding C++20 WebSocket frames, alerts, plots, and controls

let wsConn = null;
let pausedFlag = false;
let plotTick = 0;

function updateDashboard(frame) {
    const ms = frame.master_slave || {};
    const uavs = frame.uavs || [];
    const tasks = frame.tasks || [];

    const mLabel = `UAV-${(ms.active_master ?? 0) + 1} · E${String(ms.epoch_id ?? 1).padStart(2, "0")}`;
    document.getElementById("master-pill").textContent = `MASTER: ${mLabel}`;
    document.getElementById("kpi-master").textContent = mLabel;
    document.getElementById("kpi-epoch-rem").textContent = `Next rotation: ${(ms.epoch_rem_s ?? 0).toFixed(1)} s (R:${ms.total_rotations ?? 0} F:${ms.total_failovers ?? 0})`;
    document.getElementById("kpi-fiedler").textContent = (ms.fiedler_lambda2 ?? 0).toFixed(2);
    document.getElementById("kpi-latency").textContent = `Avg RF latency: ${(ms.avg_latency_ms ?? 0).toFixed(0)} ms · Relays: ${ms.relay_hops ?? 0}`;

    const alive = uavs.filter((u) => u.alive);
    const maxXte = alive.length ? Math.max(...alive.map((u) => u.xte_m)) : 0;
    const drCnt = alive.filter((u) => u.dr_active).length;
    document.getElementById("kpi-xte").textContent = `${maxXte.toFixed(1)} m`;
    document.getElementById("kpi-dr-cnt").textContent = `DR Active: ${drCnt} / ${alive.length} UAVs`;
    const doneCnt = tasks.filter((t) => t.done).length;
    document.getElementById("kpi-tasks").textContent = `${doneCnt} / ${tasks.length}`;
    document.getElementById("kpi-simtime").textContent = `Mission Time: ${(frame.sim_time_s ?? 0).toFixed(1)} s`;

    // Fleet roster table
    document.getElementById("fleet-tbody").innerHTML = uavs.map((u) => {
        const role = u.is_master ? `<span class="badge badge-master">MASTER</span>` : `<span class="badge">SLAVE</span>`;
        const link = !u.alive ? `<span class="badge badge-crit">LOST</span>` : (u.connected ? `<span class="badge badge-ok">ONLINE</span>` : `<span class="badge badge-crit">DISCONNECTED</span>`);
        const nav = u.dr_active ? `<span class="badge badge-warn">DR + NHC</span>` : `<span class="badge badge-ok">GNSS</span>`;
        const eta = Math.round(Math.min(u.l1_roll_hat, u.l1_thrust_hat) * 100);
        return `<tr>
            <td>UAV-${u.id + 1}</td>
            <td>${role}</td>
            <td>${link}</td>
            <td>${nav}</td>
            <td>${u.xte_m.toFixed(1)}</td>
            <td>${u.pos_sigma3.toFixed(1)}</td>
            <td>${eta}%${u.l1_fault ? " [FTC]" : ""}</td>
            <td>${(u.soc * 100).toFixed(1)}%</td>
            <td>${u.mode}</td>
        </tr>`;
    }).join("");

    // Live tactical alert feed
    document.getElementById("alert-feed").innerHTML = (frame.alerts || []).map((a) => `
        <div class="alert-item ${a.sev}">
            <div class="alert-head"><span>[${a.sev}] ${a.code}</span><span>t = ${a.t.toFixed(1)}s</span></div>
            <div class="alert-title">${a.title}</div>
            <div class="alert-detail">${a.detail}</div>
        </div>
    `).join("");

    TacticalScene.render(frame);
    SwarmPlots.push(frame);
    if (++plotTick % 5 === 0) SwarmPlots.render(frame);
}

function startSocketLoop() {
    if (wsConn) wsConn.close();
    wsConn = SwarmAPI.connectSocket(updateDashboard, (st) => {
        if (st === "RECONNECTING") setTimeout(startSocketLoop, 1500);
    });
}

window.addEventListener("DOMContentLoaded", () => {
    TacticalScene.init("viewport-host");
    const savedTok = localStorage.getItem("swarm_lan_token");
    if (savedTok) document.getElementById("inp-token").value = savedTok;
    document.getElementById("inp-token").addEventListener("change", (e) => {
        localStorage.setItem("swarm_lan_token", e.target.value.trim());
        startSocketLoop();
    });

    // Accessible tab switching
    document.querySelectorAll(".tab-btn").forEach((btn) => {
        btn.addEventListener("click", () => {
            const tab = btn.dataset.tab;
            document.querySelectorAll(".tab-btn").forEach((b) => {
                const act = b.dataset.tab === tab;
                b.classList.toggle("active", act);
                b.setAttribute("aria-selected", act ? "true" : "false");
            });
            document.querySelectorAll(".view-section").forEach((s) => s.classList.toggle("active", s.id === `view-${tab}`));
            if (tab === "diagnostics") {
                document.querySelectorAll(".js-plotly-plot").forEach((el) => Plotly.Plots.resize(el));
            }
        });
    });

    document.getElementById("btn-sc-blackout").addEventListener("click", () => SwarmAPI.inject({ action: "preset", text: "gnss_blackout" }));
    document.getElementById("btn-sc-spoof").addEventListener("click", () => SwarmAPI.inject({ action: "preset", text: "gnss_spoof" }));
    document.getElementById("btn-sc-fault").addEventListener("click", () => SwarmAPI.inject({ action: "preset", text: "actuator_fault" }));
    document.getElementById("btn-sc-disc").addEventListener("click", () => SwarmAPI.inject({ action: "preset", text: "disconnect_uav" }));
    document.getElementById("btn-sc-kill").addEventListener("click", () => SwarmAPI.inject({ action: "preset", text: "master_attrition" }));
    document.getElementById("btn-sc-rot").addEventListener("click", () => SwarmAPI.inject({ action: "rotate_master" }));
    document.getElementById("btn-sc-reset").addEventListener("click", () => SwarmAPI.inject({ action: "reset" }));
    document.getElementById("btn-pause").addEventListener("click", () => {
        pausedFlag = !pausedFlag;
        SwarmAPI.inject({ action: "pause", flag: pausedFlag });
    });
    document.getElementById("sel-formation").addEventListener("change", (e) => {
        SwarmAPI.inject({ action: "formation", val1: parseInt(e.target.value, 10) });
    });
    document.getElementById("sel-view-mode").addEventListener("change", (e) => TacticalScene.setMode(e.target.value));

    document.getElementById("btn-run-ablation").addEventListener("click", async () => {
        const btn = document.getElementById("btn-run-ablation");
        btn.disabled = true;
        btn.textContent = "Running C++20 OpenMP...";
        try {
            const data = await SwarmAPI.runAblation();
            document.getElementById("ablation-tbody").innerHTML = (data.variants || []).map((v) => `
                <tr>
                    <td>${v.label}</td>
                    <td>${v.rmse_m.toFixed(1)}</td>
                    <td>${v.nees_ok_pct.toFixed(1)}%</td>
                    <td>${v.completion_pct.toFixed(1)}%</td>
                    <td>${v.collision_pct.toFixed(1)}%</td>
                    <td>${v.soc_spread_pct.toFixed(2)}%</td>
                    <td>${v.cbba_savings_pct.toFixed(1)}%</td>
                </tr>
            `).join("");
        } finally {
            btn.disabled = false;
            btn.textContent = "Run OpenMP Ablation (4 × 60 Runs)";
        }
    });

    document.getElementById("btn-delete-local").addEventListener("click", () => {
        localStorage.clear();
        sessionStorage.clear();
        document.getElementById("inp-token").value = "";
        document.getElementById("delete-status").textContent = "All local browser data deleted.";
    });

    startSocketLoop();
});

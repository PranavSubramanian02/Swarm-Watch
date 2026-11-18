// Plotly live GNC diagnostics for XTE, 3-sigma uncertainty, SOC balance, and L1 effectiveness

const SwarmPlots = {
    history: { t: [], maxXte: [], maxSig3: [] },

    push(frame) {
        const alive = (frame.uavs || []).filter((u) => u.alive);
        if (alive.length === 0) return;
        this.history.t.push(frame.sim_time_s);
        this.history.maxXte.push(Math.max(...alive.map((u) => u.xte_m)));
        this.history.maxSig3.push(Math.max(...alive.map((u) => u.pos_sigma3)));
        if (this.history.t.length > 160) {
            this.history.t.shift();
            this.history.maxXte.shift();
            this.history.maxSig3.shift();
        }
    },

    render(frame) {
        if (typeof Plotly === "undefined" || !frame) return;
        const base = {
            paper_bgcolor: "rgba(0,0,0,0)",
            plot_bgcolor: "#050911",
            font: { color: "#cbd5e1", size: 11 },
            margin: { l: 48, r: 20, t: 10, b: 36 },
            legend: { orientation: "h", y: 1.14 },
        };
        Plotly.react("plot-xte-sigma", [
            { x: this.history.t, y: this.history.maxXte, mode: "lines", name: "Max XTE [m]", line: { color: "#f87171", width: 2 } },
            { x: this.history.t, y: this.history.maxSig3, mode: "lines", name: "Max 3σ [m]", line: { color: "#38bdf8", width: 2 } },
        ], { ...base, xaxis: { title: "Mission Time [s]", gridcolor: "#1e2d47" }, yaxis: { title: "Meters", gridcolor: "#1e2d47" } }, { responsive: true });

        const uavs = frame.uavs || [];
        const names = uavs.map((u) => `UAV-${u.id + 1}`);
        Plotly.react("plot-soc-l1", [
            { x: names, y: uavs.map((u) => u.soc * 100), type: "bar", name: "Battery SOC [%]", marker: { color: "#34d399" } },
            { x: names, y: uavs.map((u) => Math.min(u.l1_roll_hat, u.l1_thrust_hat) * 100), type: "bar", name: "L1 Health η̂ [%]", marker: { color: "#facc15" } },
        ], { ...base, barmode: "group", yaxis: { title: "Percent [%]", range: [0, 105], gridcolor: "#1e2d47" } }, { responsive: true });
    },
};

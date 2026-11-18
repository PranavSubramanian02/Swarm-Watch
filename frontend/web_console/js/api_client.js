// REST and WebSocket transport client for C++20 Swarm Server

const SwarmAPI = {
    getToken() {
        const el = document.getElementById("inp-token");
        return el ? el.value.trim() : "";
    },

    headers() {
        const h = { "Content-Type": "application/json" };
        const tok = this.getToken();
        if (tok) h["X-Swarm-Token"] = tok;
        return h;
    },

    async inject(payload) {
        const res = await fetch("/api/v1/inject", {
            method: "POST",
            headers: this.headers(),
            body: JSON.stringify(payload),
        });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
    },

    async runAblation() {
        const res = await fetch("/api/v1/ablation", {
            method: "POST",
            headers: this.headers(),
            body: "{}",
        });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
    },

    connectSocket(onFrame, onStatus) {
        const proto = window.location.protocol === "https:" ? "wss" : "ws";
        const tok = this.getToken();
        const q = tok ? `?token=${encodeURIComponent(tok)}` : "";
        const ws = new WebSocket(`${proto}://${window.location.host}/ws/swarm${q}`);
        ws.onopen = () => onStatus("CONNECTED");
        ws.onclose = () => onStatus("RECONNECTING");
        ws.onmessage = (evt) => {
            try { onFrame(JSON.parse(evt.data)); } catch (_) {}
        };
        return ws;
    },
};

package com.swarmwatch.app.security

// Enforces loopback or RFC-1918 private LAN server destinations only (zero external data exfiltration)

object LocalNetworkGuard {
    fun isAllowedLocalHost(hostPort: String): Boolean {
        val host = hostPort.substringBefore(":").trim().lowercase()
        if (host == "127.0.0.1" || host == "localhost" || host == "10.0.2.2") return true
        if (host.startsWith("192.168.") || host.startsWith("10.")) return true
        if (host.startsWith("172.")) {
            val parts = host.split(".")
            val second = parts.getOrNull(1)?.toIntOrNull() ?: -1
            if (second in 16..31) return true
        }
        return false
    }
}

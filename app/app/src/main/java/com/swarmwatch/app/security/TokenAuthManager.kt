package com.swarmwatch.app.security

// LAN token attachment and strict WebSocket frame size validation

import okhttp3.Request
import java.net.URLEncoder

object TokenAuthManager {
    private const val MAX_PAYLOAD_BYTES = 131072  // 128 KB upper bound

    fun appendTokenQuery(url: String, token: String): String {
        val clean = token.trim()
        if (clean.isEmpty()) return url
        val sep = if (url.contains("?")) "&" else "?"
        return "$url${sep}token=${URLEncoder.encode(clean, "UTF-8")}"
    }

    fun attachAuthHeader(builder: Request.Builder, token: String): Request.Builder {
        val clean = token.trim()
        if (clean.isNotEmpty()) builder.addHeader("X-Swarm-Token", clean)
        return builder
    }

    fun isSafePayloadSize(text: String): Boolean = text.length <= MAX_PAYLOAD_BYTES
}

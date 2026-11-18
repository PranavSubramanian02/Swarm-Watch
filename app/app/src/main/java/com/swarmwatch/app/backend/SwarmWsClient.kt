package com.swarmwatch.app.backend

// OkHttp WebSocket and REST client connecting to C++20 Swarm Server with security guards

import com.swarmwatch.app.security.LocalNetworkGuard
import com.swarmwatch.app.security.TokenAuthManager
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import java.util.concurrent.TimeUnit

class SwarmWsClient(
    private val onSnapshot: (SwarmSnapshot) -> Unit,
    private val onStatus: (String) -> Unit
) {
    private val client = OkHttpClient.Builder()
        .pingInterval(10, TimeUnit.SECONDS)
        .connectTimeout(4, TimeUnit.SECONDS)
        .build()

    private var socket: WebSocket? = null

    fun connect(hostPort: String, token: String) {
        disconnect()
        val cleanHost = hostPort.trim().removePrefix("http://").removePrefix("ws://")
        if (!LocalNetworkGuard.isAllowedLocalHost(cleanHost)) {
            onStatus("BLOCKED_NON_LOCAL_HOST")
            return
        }
        val wsUrl = TokenAuthManager.appendTokenQuery("ws://$cleanHost/ws/swarm", token)
        val req = TokenAuthManager.attachAuthHeader(Request.Builder().url(wsUrl), token).build()
        onStatus("CONNECTING")

        socket = client.newWebSocket(req, object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                onStatus("LIVE")
            }

            override fun onMessage(webSocket: WebSocket, text: String) {
                if (!TokenAuthManager.isSafePayloadSize(text)) return
                SwarmJsonParser.parse(text)?.let(onSnapshot)
            }

            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                onStatus("DISCONNECTED")
            }

            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                onStatus("DISCONNECTED")
            }
        })
    }

    fun disconnect() {
        socket?.close(1000, "Operator disconnect")
        socket = null
        onStatus("DISCONNECTED")
    }
}

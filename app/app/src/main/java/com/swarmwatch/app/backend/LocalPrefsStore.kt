package com.swarmwatch.app.backend

// Minimal local-only SharedPreferences store for host IP and optional LAN token

import android.content.Context

class LocalPrefsStore(context: Context) {
    private val prefs = context.getSharedPreferences("swarmwatch_local_prefs", Context.MODE_PRIVATE)

    fun getHostPort(): String = prefs.getString("host_port", "127.0.0.1:8080") ?: "127.0.0.1:8080"
    fun setHostPort(v: String) = prefs.edit().putString("host_port", v.trim()).apply()

    fun getToken(): String = prefs.getString("lan_token", "") ?: ""
    fun setToken(v: String) = prefs.edit().putString("lan_token", v.trim()).apply()

    fun getNotificationsEnabled(): Boolean = prefs.getBoolean("notif_enabled", true)
    fun setNotificationsEnabled(v: Boolean) = prefs.edit().putBoolean("notif_enabled", v).apply()

    fun wipeAll() {
        prefs.edit().clear().apply()
    }
}

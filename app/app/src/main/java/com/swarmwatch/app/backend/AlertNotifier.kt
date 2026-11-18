package com.swarmwatch.app.backend

// Dual-trigger alert dispatcher: Android NotificationChannel push + in-app alert deduplication

import android.app.NotificationChannel
import android.app.NotificationManager
import android.content.Context
import android.os.Build
import androidx.core.app.NotificationCompat
import com.swarmwatch.app.security.PermissionConsentManager

class AlertNotifier(private val context: Context) {
    private val channelId = "swarmwatch_tactical_alerts"
    private var lastNotifiedSeq: Long = 0L

    init {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val ch = NotificationChannel(
                channelId,
                "Swarm Tactical Alerts",
                NotificationManager.IMPORTANCE_HIGH
            ).apply {
                description = "Alerts for UAV disconnects, Master-Slave rotations, Dead Reckoning, and trajectory deviations."
            }
            val nm = context.getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            nm.createNotificationChannel(ch)
        }
    }

    // Emit Android system notification only for new, high-value tactical events
    fun processIncomingAlerts(alerts: List<AlertEvent>, notificationsAllowed: Boolean) {
        if (alerts.isEmpty()) return
        val fresh = alerts.filter { it.seq > lastNotifiedSeq }.sortedBy { it.seq }
        if (fresh.isEmpty()) return
        lastNotifiedSeq = fresh.maxOf { it.seq }

        if (!notificationsAllowed || !PermissionConsentManager.hasNotificationPermission(context)) return
        val nm = context.getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        for (a in fresh) {
            if (a.code in setOf(
                    "UAV_DISCONNECTED", "UAV_RECONNECTED", "MASTER_ROTATED", "MASTER_FAILOVER",
                    "DR_ACTIVATED", "GNSS_SPOOF_REJECTED", "TRAJECTORY_DEVIATION",
                    "L1_FTC_ACTIVE", "MDP_EMERGENCY_RTB", "UAV_ATTRITION"
                )
            ) {
                val notif = NotificationCompat.Builder(context, channelId)
                    .setSmallIcon(android.R.drawable.ic_dialog_alert)
                    .setContentTitle("[${a.severity}] ${a.title}")
                    .setContentText(a.detail)
                    .setPriority(NotificationCompat.PRIORITY_HIGH)
                    .setAutoCancel(true)
                    .build()
                nm.notify(a.seq.toInt(), notif)
            }
        }
    }

    fun resetSequence() {
        lastNotifiedSeq = 0L
    }
}

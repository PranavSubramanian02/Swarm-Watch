package com.swarmwatch.app.security

// 1-tap complete deletion of all saved local preferences, host settings, and alert state

import android.app.NotificationManager
import android.content.Context
import com.swarmwatch.app.backend.AlertNotifier
import com.swarmwatch.app.backend.LocalPrefsStore

object DataWipeManager {
    fun deleteAllLocalData(context: Context, prefs: LocalPrefsStore, notifier: AlertNotifier) {
        prefs.wipeAll()
        notifier.resetSequence()
        val nm = context.getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        nm.cancelAll()
    }
}

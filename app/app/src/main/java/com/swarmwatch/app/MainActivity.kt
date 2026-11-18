package com.swarmwatch.app

// SwarmWatch entry point binding 3-tab Jetpack Compose UI to C++20 WebSocket, Alerts, and Security layers

import android.Manifest
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Notifications
import androidx.compose.material.icons.filled.Radar
import androidx.compose.material.icons.filled.Sensors
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.swarmwatch.app.backend.*
import com.swarmwatch.app.frontend.*
import com.swarmwatch.app.security.DataWipeManager
import com.swarmwatch.app.security.PermissionConsentManager
import kotlinx.coroutines.delay

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val prefs = LocalPrefsStore(applicationContext)
        val notifier = AlertNotifier(applicationContext)

        setContent {
            SwarmWatchTheme {
                var tab by remember { mutableIntStateOf(0) }
                var snapshot by remember { mutableStateOf(OfflineDemoFeed.buildFrame(0)) }
                var connStatus by remember { mutableStateOf("DEMO") }
                var offlineDemo by remember { mutableStateOf(true) }
                var hostPort by remember { mutableStateOf(prefs.getHostPort()) }
                var token by remember { mutableStateOf(prefs.getToken()) }
                var notifEnabled by remember { mutableStateOf(prefs.getNotificationsEnabled()) }
                var showLegal by remember { mutableStateOf(false) }

                val permLauncher = rememberLauncherForActivityResult(
                    ActivityResultContracts.RequestPermission()
                ) { granted ->
                    notifEnabled = granted
                    prefs.setNotificationsEnabled(granted)
                }

                val wsClient = remember {
                    SwarmWsClient(
                        onSnapshot = { snap ->
                            runOnUiThread {
                                snapshot = snap
                                notifier.processIncomingAlerts(snap.alerts, notifEnabled)
                            }
                        },
                        onStatus = { st -> runOnUiThread { connStatus = st } }
                    )
                }

                DisposableEffect(Unit) {
                    onDispose { wsClient.disconnect() }
                }

                // Standalone offline demo loop when not connected to live laptop server
                LaunchedEffect(offlineDemo) {
                    var tick = 0
                    while (offlineDemo) {
                        val frame = OfflineDemoFeed.buildFrame(tick++)
                        snapshot = frame
                        notifier.processIncomingAlerts(frame.alerts, notifEnabled)
                        delay(500)
                    }
                }

                Scaffold(
                    containerColor = SwarmColors.BgCockpit,
                    topBar = {
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .background(SwarmColors.BgPanel)
                                .padding(horizontal = 14.dp, vertical = 12.dp),
                            horizontalArrangement = Arrangement.SpaceBetween
                        ) {
                            Text("SWARMWATCH", color = SwarmColors.TextPrimary, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
                            Text(
                                "M:U${snapshot.activeMaster + 1} E${snapshot.epochId} · ${if (offlineDemo) "DEMO" else connStatus}",
                                color = SwarmColors.Gold,
                                fontFamily = FontFamily.Monospace,
                                fontSize = 12.sp
                            )
                        }
                    },
                    bottomBar = {
                        NavigationBar(containerColor = SwarmColors.BgPanel) {
                            NavigationBarItem(
                                selected = tab == 0,
                                onClick = { tab = 0 },
                                icon = { Icon(Icons.Default.Notifications, contentDescription = "Live Swarm Alerts Tab") },
                                label = { Text("Alerts") }
                            )
                            NavigationBarItem(
                                selected = tab == 1,
                                onClick = { tab = 1 },
                                icon = { Icon(Icons.Default.Radar, contentDescription = "Trajectory Deviation Tab") },
                                label = { Text("Trajectory") }
                            )
                            NavigationBarItem(
                                selected = tab == 2,
                                onClick = { tab = 2 },
                                icon = { Icon(Icons.Default.Sensors, contentDescription = "Swarm Roster and Settings Tab") },
                                label = { Text("Swarm") }
                            )
                        }
                    }
                ) { innerPad ->
                    Box(modifier = Modifier.padding(innerPad).fillMaxSize()) {
                        when (tab) {
                            0 -> AlertsScreen(snapshot)
                            1 -> TrajectoryScreen(snapshot)
                            else -> SwarmScreen(
                                snapshot = snapshot,
                                connectionStatus = if (offlineDemo) "OFFLINE DEMO" else connStatus,
                                hostPort = hostPort,
                                token = token,
                                offlineDemo = offlineDemo,
                                notifEnabled = notifEnabled,
                                onConnect = { h, tok ->
                                    hostPort = h; token = tok
                                    prefs.setHostPort(h); prefs.setToken(tok)
                                    offlineDemo = false
                                    wsClient.connect(h, tok)
                                },
                                onDisconnect = { wsClient.disconnect() },
                                onToggleDemo = { on ->
                                    offlineDemo = on
                                    if (on) { wsClient.disconnect(); connStatus = "DEMO" }
                                },
                                onToggleNotif = { on ->
                                    if (on && PermissionConsentManager.needsRuntimeNotificationPermission() &&
                                        !PermissionConsentManager.hasNotificationPermission(applicationContext)
                                    ) {
                                        permLauncher.launch(Manifest.permission.POST_NOTIFICATIONS)
                                    } else {
                                        notifEnabled = on
                                        prefs.setNotificationsEnabled(on)
                                    }
                                },
                                onOpenLegal = { showLegal = true }
                            )
                        }
                    }
                }

                if (showLegal) {
                    LegalDialog(
                        onDeleteAllData = {
                            DataWipeManager.deleteAllLocalData(applicationContext, prefs, notifier)
                            hostPort = "127.0.0.1:8080"
                            token = ""
                            showLegal = false
                        },
                        onDismiss = { showLegal = false }
                    )
                }
            }
        }
    }
}

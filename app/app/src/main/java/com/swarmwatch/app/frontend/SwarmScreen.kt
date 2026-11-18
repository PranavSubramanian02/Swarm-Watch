package com.swarmwatch.app.frontend

// Tab 3: 5-UAV Fleet Roster, Local Server Connection, Offline Demo toggle, and Legal/Delete button

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.swarmwatch.app.backend.SwarmSnapshot

@Composable
fun SwarmScreen(
    snapshot: SwarmSnapshot,
    connectionStatus: String,
    hostPort: String,
    token: String,
    offlineDemo: Boolean,
    notifEnabled: Boolean,
    onConnect: (String, String) -> Unit,
    onDisconnect: () -> Unit,
    onToggleDemo: (Boolean) -> Unit,
    onToggleNotif: (Boolean) -> Unit,
    onOpenLegal: () -> Unit
) {
    var hostInput by remember(hostPort) { mutableStateOf(hostPort) }
    var tokenInput by remember(token) { mutableStateOf(token) }

    LazyColumn(
        modifier = Modifier.fillMaxSize().padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {
        item {
            Column(
                modifier = Modifier.fillMaxWidth().background(SwarmColors.BgPanel, RoundedCornerShape(10.dp)).padding(12.dp),
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Text("C++20 SERVER CONNECTION ($connectionStatus)", color = SwarmColors.Cyan, fontFamily = FontFamily.Monospace, fontSize = 11.sp, fontWeight = FontWeight.Bold)
                OutlinedTextField(
                    value = hostInput,
                    onValueChange = { hostInput = it },
                    label = { Text("Local Host:Port (127.0.0.1:8080 or 10.0.2.2:8080)") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth()
                )
                OutlinedTextField(
                    value = tokenInput,
                    onValueChange = { tokenInput = it },
                    label = { Text("Optional LAN Security Token") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth()
                )
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(onClick = { onConnect(hostInput, tokenInput) }) { Text("Connect") }
                    OutlinedButton(onClick = onDisconnect) { Text("Disconnect") }
                }
                Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.SpaceBetween, modifier = Modifier.fillMaxWidth()) {
                    Text("Offline Demo Mode (No Laptop Needed)", color = SwarmColors.TextPrimary, fontSize = 13.sp)
                    Switch(checked = offlineDemo, onCheckedChange = onToggleDemo)
                }
                Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.SpaceBetween, modifier = Modifier.fillMaxWidth()) {
                    Text("Android System Push Alerts", color = SwarmColors.TextPrimary, fontSize = 13.sp)
                    Switch(checked = notifEnabled, onCheckedChange = onToggleNotif)
                }
                OutlinedButton(onClick = onOpenLegal, modifier = Modifier.fillMaxWidth()) {
                    Text("Privacy Policy, Terms, Licenses & Delete Local Data")
                }
            }
        }

        item {
            Text(
                "SWARM ROSTER · MASTER: UAV-${snapshot.activeMaster + 1} (EPOCH ${snapshot.epochId}) · λ2=${"%.2f".format(snapshot.fiedlerLambda2)}",
                color = SwarmColors.Gold,
                fontFamily = FontFamily.Monospace,
                fontSize = 11.sp,
                fontWeight = FontWeight.Bold
            )
        }

        items(snapshot.uavs, key = { it.id }) { u ->
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .background(SwarmColors.BgPanel, RoundedCornerShape(8.dp))
                    .padding(10.dp),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Column {
                    Text(
                        "UAV-${u.id + 1} · ${if (u.isMaster) "MASTER" else "SLAVE"}",
                        color = if (u.isMaster) SwarmColors.Gold else SwarmColors.TextPrimary,
                        fontWeight = FontWeight.Bold,
                        fontFamily = FontFamily.Monospace
                    )
                    Text(
                        "${if (u.connected) "ONLINE" else "DISCONNECTED"} | ${if (u.drActive) "DR+NHC" else "GNSS"} | ${u.mode}",
                        color = if (!u.connected) SwarmColors.Critical else SwarmColors.TextSecondary,
                        fontSize = 11.sp,
                        fontFamily = FontFamily.Monospace
                    )
                }
                Column(horizontalAlignment = Alignment.End) {
                    Text("SOC ${(u.soc * 100).toInt()}%", color = SwarmColors.Nominal, fontFamily = FontFamily.Monospace, fontSize = 12.sp)
                    Text("XTE ${"%.1f".format(u.xteM)}m", color = SwarmColors.Cyan, fontFamily = FontFamily.Monospace, fontSize = 11.sp)
                }
            }
        }
    }
}

package com.swarmwatch.app.frontend

// Tab 1: Severity-filtered live tactical alerts (Disconnect, Master switch, DR, Spoof, XTE, L1)

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.swarmwatch.app.backend.AlertEvent
import com.swarmwatch.app.backend.SwarmSnapshot

@Composable
fun AlertsScreen(snapshot: SwarmSnapshot) {
    var filter by remember { mutableStateOf("ALL") }
    val filtered = remember(snapshot.alerts, filter) {
        if (filter == "ALL") snapshot.alerts else snapshot.alerts.filter { it.severity == filter }
    }

    Column(modifier = Modifier.fillMaxSize().padding(12.dp)) {
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp), modifier = Modifier.fillMaxWidth()) {
            listOf("ALL", "CRITICAL", "CAUTION", "INFO").forEach { sev ->
                FilterChip(
                    selected = filter == sev,
                    onClick = { filter = sev },
                    label = { Text(sev, fontSize = 11.sp, fontFamily = FontFamily.Monospace) }
                )
            }
        }
        Spacer(Modifier.height(8.dp))
        if (filtered.isEmpty()) {
            Text(
                "No alerts matching filter yet. Connect to C++20 server or enable Offline Demo Mode in Swarm tab.",
                color = SwarmColors.TextSecondary,
                fontSize = 13.sp,
                modifier = Modifier.padding(12.dp)
            )
        } else {
            LazyColumn(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                items(filtered, key = { it.seq }) { alert ->
                    AlertCard(alert)
                }
            }
        }
    }
}

@Composable
private fun AlertCard(a: AlertEvent) {
    val accent = when (a.severity) {
        "CRITICAL" -> SwarmColors.Critical
        "CAUTION" -> SwarmColors.Caution
        else -> SwarmColors.Cyan
    }
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .background(SwarmColors.BgPanel, RoundedCornerShape(8.dp))
            .padding(12.dp)
            .semantics { contentDescription = "${a.severity} alert: ${a.title}. ${a.detail}" }
    ) {
        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text("[${a.severity}] ${a.code}", color = accent, fontFamily = FontFamily.Monospace, fontSize = 11.sp, fontWeight = FontWeight.Bold)
            Text("t=${"%.1f".format(a.timeS)}s", color = SwarmColors.TextSecondary, fontFamily = FontFamily.Monospace, fontSize = 11.sp)
        }
        Spacer(Modifier.height(3.dp))
        Text(a.title, color = SwarmColors.TextPrimary, fontWeight = FontWeight.Bold, fontSize = 14.sp)
        Spacer(Modifier.height(2.dp))
        Text(a.detail, color = SwarmColors.TextSecondary, fontSize = 12.sp)
    }
}

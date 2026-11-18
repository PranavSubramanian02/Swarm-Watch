package com.swarmwatch.app.frontend

// Tab 2: Ideal (dashed) vs Actual (solid) trajectory deviation canvas and 4 compact KPI cards

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.swarmwatch.app.backend.SwarmSnapshot

@Composable
fun TrajectoryScreen(snapshot: SwarmSnapshot) {
    var selectedUav by remember { mutableIntStateOf(0) }
    val uav = snapshot.uavs.getOrNull(selectedUav) ?: snapshot.uavs.firstOrNull()

    Column(modifier = Modifier.fillMaxSize().padding(12.dp), verticalArrangement = Arrangement.spacedBy(10.dp)) {
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp), modifier = Modifier.fillMaxWidth()) {
            snapshot.uavs.forEach { item ->
                FilterChip(
                    selected = item.id == selectedUav,
                    onClick = { selectedUav = item.id },
                    label = {
                        Text(
                            "U${item.id + 1}${if (item.isMaster) "*" else ""}",
                            fontFamily = FontFamily.Monospace,
                            fontSize = 11.sp
                        )
                    }
                )
            }
        }

        // 2D Ideal (dashed) vs Actual (solid) trajectory deviation radar
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f)
                .background(SwarmColors.BgPanel, RoundedCornerShape(10.dp))
                .padding(8.dp)
        ) {
            Canvas(
                modifier = Modifier
                    .fillMaxSize()
                    .semantics { contentDescription = "2D tactical trajectory deviation canvas comparing ideal and actual paths" }
            ) {
                val half = 1500f
                fun toPx(n: Double, e: Double): Offset {
                    val x = size.width * 0.5f + (e.toFloat() / half) * (size.minDimension * 0.44f)
                    val y = size.height * 0.5f - (n.toFloat() / half) * (size.minDimension * 0.44f)
                    return Offset(x, y)
                }

                // Grid crosshairs
                drawLine(Color(0xFF1E2D47), Offset(0f, size.height * 0.5f), Offset(size.width, size.height * 0.5f), 1f)
                drawLine(Color(0xFF1E2D47), Offset(size.width * 0.5f, 0f), Offset(size.width * 0.5f, size.height), 1f)

                val dash = PathEffect.dashPathEffect(floatArrayOf(10f, 8f), 0f)
                snapshot.uavs.forEach { item ->
                    if (!item.alive) return@forEach
                    val pos = toPx(item.north, item.east)
                    val ideal = toPx(item.idealNorth, item.idealEast)
                    val drift = toPx(item.driftEnd.north, item.driftEnd.east)
                    val isSel = item.id == selectedUav

                    // Actual solid trail
                    for (k in 1 until item.trail.size) {
                        val a = toPx(item.trail[k - 1].north, item.trail[k - 1].east)
                        val b = toPx(item.trail[k].north, item.trail[k].east)
                        drawLine(
                            if (item.isMaster) SwarmColors.Gold else SwarmColors.Cyan,
                            a, b, strokeWidth = if (isSel) 3.5f else 1.8f
                        )
                    }
                    // Ideal-to-actual deviation vector (dashed red)
                    drawLine(SwarmColors.Critical, ideal, pos, strokeWidth = if (isSel) 2.6f else 1.4f, pathEffect = dash)
                    // Forward predicted drift vector (amber)
                    if (isSel) {
                        drawLine(SwarmColors.Caution, pos, drift, strokeWidth = 2.2f, pathEffect = dash)
                    }
                    // Master gold halo
                    if (item.isMaster) {
                        drawCircle(SwarmColors.Gold, radius = 16f, center = pos, style = Stroke(width = 2.5f))
                    }
                    drawCircle(
                        color = if (item.isMaster) SwarmColors.Gold else if (!item.connected) SwarmColors.Critical else SwarmColors.Cyan,
                        radius = if (isSel) 8f else 5.5f,
                        center = pos
                    )
                }
            }
        }

        // 4 compact telemetry cards
        if (uav != null) {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp), modifier = Modifier.fillMaxWidth()) {
                KpiMiniCard("PATH DEVIATION", "XTE ${"%.1f".format(uav.xteM)} m", "Alt Δz: ${"%.1f".format(uav.altDevM)} m", Modifier.weight(1f))
                KpiMiniCard("NAV & 3σ", if (uav.drActive) "DEAD RECKONING" else "GNSS LOCK", "3σ: ${"%.1f".format(uav.posSigma3M)} m", Modifier.weight(1f))
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp), modifier = Modifier.fillMaxWidth()) {
                KpiMiniCard("SWARM ROLE", if (uav.isMaster) "MASTER (E${snapshot.epochId})" else "SLAVE", "Next rot: ${"%.0f".format(snapshot.epochRemS)} s", Modifier.weight(1f))
                KpiMiniCard("LINK & HEALTH", if (uav.connected) "CONNECTED" else "DISCONNECTED", "SOC ${(uav.soc * 100).toInt()}% · L1 ${uav.l1HealthPct}%", Modifier.weight(1f))
            }
        }
    }
}

@Composable
private fun KpiMiniCard(label: String, value: String, sub: String, modifier: Modifier) {
    Column(
        modifier = modifier
            .background(SwarmColors.BgPanel, RoundedCornerShape(8.dp))
            .padding(10.dp)
    ) {
        Text(label, color = SwarmColors.TextSecondary, fontSize = 10.sp, fontFamily = FontFamily.Monospace)
        Text(value, color = SwarmColors.TextPrimary, fontSize = 14.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
        Text(sub, color = SwarmColors.Cyan, fontSize = 11.sp, fontFamily = FontFamily.Monospace)
    }
}

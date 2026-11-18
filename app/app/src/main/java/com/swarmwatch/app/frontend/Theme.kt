package com.swarmwatch.app.frontend

// WCAG AA (>= 4.5:1) dark aerospace color palette using system fonts only

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

object SwarmColors {
    val BgCockpit = Color(0xFF070B13)
    val BgPanel = Color(0xFF0D1422)
    val BgElevated = Color(0xFF141F34)
    val TextPrimary = Color(0xFFF1F5F9)
    val TextSecondary = Color(0xFFCBD5E1)
    val Cyan = Color(0xFF38BDF8)
    val Gold = Color(0xFFFACC15)
    val Nominal = Color(0xFF34D399)
    val Caution = Color(0xFFFBBF24)
    val Critical = Color(0xFFF87171)
}

private val DarkScheme = darkColorScheme(
    primary = SwarmColors.Cyan,
    secondary = SwarmColors.Gold,
    background = SwarmColors.BgCockpit,
    surface = SwarmColors.BgPanel,
    onBackground = SwarmColors.TextPrimary,
    onSurface = SwarmColors.TextPrimary,
    error = SwarmColors.Critical
)

@Composable
fun SwarmWatchTheme(content: @Composable () -> Unit) {
    MaterialTheme(colorScheme = DarkScheme, content = content)
}

package com.swarmwatch.app.frontend

// Privacy Policy, Experimental Simulation Disclaimer, Open-Source Licenses, and 1-Tap Data Deletion

import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun LegalDialog(
    onDeleteAllData: () -> Unit,
    onDismiss: () -> Unit
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        containerColor = SwarmColors.BgPanel,
        title = { Text("Privacy, Terms & Open-Source Notice", color = SwarmColors.TextPrimary, fontWeight = FontWeight.Bold) },
        text = {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text(
                    "Privacy Policy: SwarmWatch operates strictly over your local loopback or private LAN. Zero cloud servers, zero analytics/ad SDKs, zero tracking cookies, and zero personal accounts are used.",
                    color = SwarmColors.TextSecondary, fontSize = 12.sp
                )
                Text(
                    "Terms & Disclaimer: Developed by Pranav Subramanian as academic M.Tech aerospace research software. Experimental simulation monitor only—not flight-certified avionics.",
                    color = SwarmColors.TextSecondary, fontSize = 12.sp
                )
                Text(
                    "Licenses: Jetpack Compose, AndroidX, OkHttp, and Kotlin Coroutines are Apache-2.0 licensed. Uses system typography only.",
                    color = SwarmColors.TextSecondary, fontSize = 12.sp
                )
            }
        },
        confirmButton = {
            Button(
                colors = ButtonDefaults.buttonColors(containerColor = SwarmColors.Critical),
                onClick = onDeleteAllData
            ) {
                Text("Delete All Local Data")
            }
        },
        dismissButton = {
            TextButton(onClick = onDismiss) { Text("Close") }
        }
    )
}

package com.swarmwatch.app.backend

// Standalone deterministic demo feed so SwarmWatch works offline without a laptop server

import kotlin.math.cos
import kotlin.math.sin

object OfflineDemoFeed {
    fun buildFrame(tick: Int): SwarmSnapshot {
        val t = tick * 0.5
        val epoch = (t / 35.0).toInt() + 1
        val activeMaster = (epoch - 1) % 5
        val drOn = (t in 25.0..65.0)
        val uav3Disconnected = (t in 35.0..50.0)
        val uav2Fault = (t >= 45.0)

        val uavs = (0 until 5).map { id ->
            val ang = 0.045 * t + id * 1.25
            val r = 680.0 + id * 85.0
            val idealN = r * cos(ang)
            val idealE = r * sin(ang)
            val dev = if (id == 1 && uav2Fault) 34.0 else if (drOn) 16.5 else 4.2
            val actN = idealN + dev * cos(ang * 2.0)
            val actE = idealE + dev * sin(ang * 2.0)
            val trail = (0..18).map { k ->
                val a = ang - k * 0.06
                Vec2Pt(r * cos(a) + dev * 0.6 * cos(a * 2.0), r * sin(a) + dev * 0.6 * sin(a * 2.0))
            }.reversed()
            UavSnapshot(
                id = id,
                alive = true,
                isMaster = (id == activeMaster),
                connected = !(id == 2 && uav3Disconnected),
                drActive = drOn,
                mode = if (id == 1 && uav2Fault) "CONSERVATIVE_RISK" else if (drOn) "DEGRADED_COOP" else "NOMINAL",
                north = actN,
                east = actE,
                altM = 135.0 + 18.0 * (id % 3),
                idealNorth = idealN,
                idealEast = idealE,
                xteM = dev,
                altDevM = if (drOn) 4.8 else 1.1,
                posSigma3M = if (drOn) 24.5 else 6.2,
                soc = (0.94 - t * 0.0012 - id * 0.01).coerceIn(0.20, 1.0),
                l1HealthPct = if (id == 1 && uav2Fault) 58 else 99,
                l1Fault = (id == 1 && uav2Fault),
                trail = trail,
                driftEnd = Vec2Pt(actN - 180.0 * sin(ang), actE + 180.0 * cos(ang))
            )
        }

        val alerts = mutableListOf<AlertEvent>()
        if (uav2Fault) {
            alerts.add(AlertEvent(4, 45.0, "CRITICAL", "L1_FTC_ACTIVE", 1,
                "Actuator Fault Detected (UAV-2)", "L1 Adaptive FTC engaged (η̂ = 58%); trajectory bounded."))
        }
        if (uav3Disconnected) {
            alerts.add(AlertEvent(3, 35.0, "CRITICAL", "UAV_DISCONNECTED", 2,
                "UAV-3 Disconnected from Swarm", "RF heartbeat lost; running autonomous 15-state ESKF."))
        }
        if (epoch > 1) {
            alerts.add(AlertEvent(2, 35.0, "INFO", "MASTER_ROTATED", activeMaster,
                "Master Rotated: UAV-1 -> UAV-${activeMaster + 1}", "Epoch $epoch scheduled TDMA rotation."))
        }
        if (drOn) {
            alerts.add(AlertEvent(1, 25.0, "CAUTION", "DR_ACTIVATED", -1,
                "Dead Reckoning Activated", "GNSS jammed; 15-state Quaternion ESKF + NHC active."))
        }

        return SwarmSnapshot(
            simTimeS = t,
            activeMaster = activeMaster,
            epochId = epoch,
            epochRemS = (35.0 - (t % 35.0)),
            totalRotations = epoch - 1,
            totalFailovers = 0,
            fiedlerLambda2 = if (uav3Disconnected) 1.4 else 3.2,
            globalJamming = drOn,
            spoofActive = false,
            uavs = uavs,
            alerts = alerts
        )
    }
}

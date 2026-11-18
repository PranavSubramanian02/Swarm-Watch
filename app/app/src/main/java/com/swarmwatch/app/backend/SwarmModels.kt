package com.swarmwatch.app.backend

// Typed telemetry, trajectory deviation, and alert data models matching C++20 JSON schema

import org.json.JSONObject

data class Vec2Pt(val north: Double, val east: Double)

data class UavSnapshot(
    val id: Int,
    val alive: Boolean,
    val isMaster: Boolean,
    val connected: Boolean,
    val drActive: Boolean,
    val mode: String,
    val north: Double,
    val east: Double,
    val altM: Double,
    val idealNorth: Double,
    val idealEast: Double,
    val xteM: Double,
    val altDevM: Double,
    val posSigma3M: Double,
    val soc: Double,
    val l1HealthPct: Int,
    val l1Fault: Boolean,
    val trail: List<Vec2Pt>,
    val driftEnd: Vec2Pt
)

data class AlertEvent(
    val seq: Long,
    val timeS: Double,
    val severity: String,
    val code: String,
    val uavId: Int,
    val title: String,
    val detail: String
)

data class SwarmSnapshot(
    val simTimeS: Double = 0.0,
    val activeMaster: Int = 0,
    val epochId: Int = 1,
    val epochRemS: Double = 35.0,
    val totalRotations: Int = 0,
    val totalFailovers: Int = 0,
    val fiedlerLambda2: Double = 0.0,
    val globalJamming: Boolean = false,
    val spoofActive: Boolean = false,
    val uavs: List<UavSnapshot> = emptyList(),
    val alerts: List<AlertEvent> = emptyList()
)

object SwarmJsonParser {
    // Parse C++20 JSON frame safely using Android built-in org.json (zero extra dependency)
    fun parse(raw: String): SwarmSnapshot? {
        return try {
            val root = JSONObject(raw)
            val ms = root.optJSONObject("master_slave") ?: JSONObject()
            val jam = root.optJSONObject("jamming") ?: JSONObject()
            val uavsArr = root.optJSONArray("uavs")
            val uavs = mutableListOf<UavSnapshot>()
            if (uavsArr != null) {
                for (i in 0 until uavsArr.length()) {
                    val u = uavsArr.getJSONObject(i)
                    val tp = u.optJSONArray("true_pos")
                    val ip = u.optJSONArray("ideal_pos")
                    val dc = u.optJSONArray("drift_cone")
                    val trArr = u.optJSONArray("trail")
                    val trail = mutableListOf<Vec2Pt>()
                    if (trArr != null) {
                        for (k in 0 until trArr.length()) {
                            val pt = trArr.getJSONArray(k)
                            trail.add(Vec2Pt(pt.optDouble(0), pt.optDouble(1)))
                        }
                    }
                    val lastDrift = if (dc != null && dc.length() > 0) {
                        val last = dc.getJSONArray(dc.length() - 1)
                        Vec2Pt(last.optDouble(0), last.optDouble(1))
                    } else {
                        Vec2Pt(tp?.optDouble(0) ?: 0.0, tp?.optDouble(1) ?: 0.0)
                    }
                    val rHat = u.optDouble("l1_roll_hat", 1.0)
                    val tHat = u.optDouble("l1_thrust_hat", 1.0)
                    uavs.add(
                        UavSnapshot(
                            id = u.optInt("id", i),
                            alive = u.optBoolean("alive", true),
                            isMaster = u.optBoolean("is_master", false),
                            connected = u.optBoolean("connected", true),
                            drActive = u.optBoolean("dr_active", false),
                            mode = u.optString("mode", "NOMINAL"),
                            north = tp?.optDouble(0) ?: 0.0,
                            east = tp?.optDouble(1) ?: 0.0,
                            altM = tp?.optDouble(2) ?: 140.0,
                            idealNorth = ip?.optDouble(0) ?: 0.0,
                            idealEast = ip?.optDouble(1) ?: 0.0,
                            xteM = u.optDouble("xte_m", 0.0),
                            altDevM = u.optDouble("alt_dev_m", 0.0),
                            posSigma3M = u.optDouble("pos_sigma3", 0.0),
                            soc = u.optDouble("soc", 1.0),
                            l1HealthPct = (minOf(rHat, tHat) * 100.0).toInt(),
                            l1Fault = u.optBoolean("l1_fault", false),
                            trail = trail,
                            driftEnd = lastDrift
                        )
                    )
                }
            }
            val alertsArr = root.optJSONArray("alerts")
            val alerts = mutableListOf<AlertEvent>()
            if (alertsArr != null) {
                for (i in 0 until alertsArr.length()) {
                    val a = alertsArr.getJSONObject(i)
                    alerts.add(
                        AlertEvent(
                            seq = a.optLong("seq", i.toLong()),
                            timeS = a.optDouble("t", 0.0),
                            severity = a.optString("sev", "INFO"),
                            code = a.optString("code", ""),
                            uavId = a.optInt("uav_id", -1),
                            title = a.optString("title", ""),
                            detail = a.optString("detail", "")
                        )
                    )
                }
            }
            SwarmSnapshot(
                simTimeS = root.optDouble("sim_time_s", 0.0),
                activeMaster = ms.optInt("active_master", 0),
                epochId = ms.optInt("epoch_id", 1),
                epochRemS = ms.optDouble("epoch_rem_s", 35.0),
                totalRotations = ms.optInt("total_rotations", 0),
                totalFailovers = ms.optInt("total_failovers", 0),
                fiedlerLambda2 = ms.optDouble("fiedler_lambda2", 0.0),
                globalJamming = jam.optBoolean("global", false),
                spoofActive = jam.optBoolean("spoof", false),
                uavs = uavs,
                alerts = alerts
            )
        } catch (_: Exception) {
            null
        }
    }
}

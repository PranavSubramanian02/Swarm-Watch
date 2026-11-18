# SwarmWatch Kotlin Android Companion App (`app/`)

`SwarmWatch` is a minimal, zero-tracker Kotlin + Jetpack Compose Android companion app connecting directly to the C++20 swarm server over local loopback (`adb reverse`) or private LAN WebSockets (`ws://<host>:8080/ws/swarm`).

## Internal Modularity (`app/src/main/java/com/swarmwatch/app/`)
- **`backend/`**: `SwarmModels.kt` (typed telemetry/alert JSON parser), `SwarmWsClient.kt` (OkHttp WebSocket client), `AlertNotifier.kt` (dual-trigger Android `NotificationChannel` push + deduplicated feed for UAV disconnect/reconnect, Master–Slave rotation & failover, Dead Reckoning activation, GNSS spoof rejection, trajectory deviation, and $\mathcal{L}_1$ actuator faults), `OfflineDemoFeed.kt` (standalone offline demo trace), and `LocalPrefsStore.kt`.
- **`security/`**: `LocalNetworkGuard.kt` + `res/xml/network_security_config.xml` (restricting cleartext traffic strictly to loopback/RFC-1918 LAN), `TokenAuthManager.kt` (optional `X-Swarm-Token` handshake and 128 KB payload bound), `PermissionConsentManager.kt` (explicit Android 13+ notification runtime consent), and `DataWipeManager.kt` (1-tap complete local data deletion).
- **`frontend/`**: 3 clean Material 3 dark HUD screens—`AlertsScreen.kt` (severity-filtered live tactical alerts), `TrajectoryScreen.kt` (2D Ideal dashed vs. Actual solid path deviation canvas + 4 compact KPI cards), `SwarmScreen.kt` (5-UAV roster, server connect, offline demo toggle), and `LegalSheet.kt` (Privacy Policy, experimental simulation disclaimer, open-source licenses, and 1-tap local data wipe).

## Running on Android Phone or Emulator

```bash
# Start the C++20 swarm server (or native OpenGL viewer) on your laptop
./build/swarm_server --port 8080

# For a physical Android phone over USB, forward port 8080 cleanly
adb reverse tcp:8080 tcp:8080
```

Open `Project4_Survivable_Multi_UAV_Autonomy/app` in Android Studio, build and run on your phone (`127.0.0.1:8080`) or Android Emulator (`10.0.2.2:8080`), or toggle **Offline Demo Mode** to preview all alerts and trajectory deviations standalone.

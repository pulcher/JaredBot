# Execution Plan — ESP32 Serial Relay → SignalR

Date: 2026-02-09

## 1) Goal

Build an ESP32-based “serial relay” that:

- Communicates **bidirectionally** with an Arduino Uno over a dedicated serial link.
  - **Uno side**: uses NeoSWSerial (requirement).
  - **ESP32 side**: uses a **hardware UART** (recommended; NeoSWSerial is AVR-focused and not an ESP32 fit).
- Connects to WiFi as a station (STA) when possible.
- **Always** provides a fallback WiFi Access Point (AP) with a minimal configuration webpage (because upstream AP is not guaranteed).
- Connects to an **ASP.NET Core SignalR** hub for monitoring/relay.
- Displays status on an SSD1306 OLED (128x32) via I2C.

The plan is organized into milestones you can build/upload/test independently.

## 2) Current Repository State

- Target: ESP32 DevKit v1, Arduino framework (PlatformIO env `esp32doit-devkit-v1`).
- Entry point: `src/main.cpp`.
- OLED (SSD1306 128x32, I2C) is now wired up using Adafruit SSD1306/GFX dependencies.

## 3) Locked SignalR Hub Route (Decision)

We will standardize the hub route to:

- **Hub route**: `/telemetryHub`

The device will store only a **Server Origin** in configuration (scheme + host + optional port), e.g.:

- `http://192.168.1.50:5000`
- `http://192.168.4.2:5000` (when laptop is connected to the ESP32 AP)

The hub path `/telemetryHub` is not configurable (fixed).

### Exact endpoints the ESP32 will use

Given `server_origin`:

- Negotiate: `${server_origin}/telemetryHub/negotiate?negotiateVersion=1`
- WebSocket URL: derived from negotiate response (WebSockets transport)

### SignalR JSON protocol reminders

- Immediately after WebSocket connect, send the SignalR handshake frame:
  - `{"protocol":"json","version":1}` followed by record separator byte `0x1e`.
- Subsequent messages are JSON frames delimited by `0x1e`.

## 4) Architecture (High Level)

### ESP32 firmware (this repo)

Subsystems:

1. **USB serial debug**
   - `Serial` at 115200 for logs and bring-up.

2. **Uno link (secondary serial)**
   - ESP32 uses `HardwareSerial` (commonly `Serial2`).
   - Start at **9600 baud** (can change later).
   - Line-delimited text for early milestones; binary framing can come later if needed.

3. **OLED status (SSD1306 128x32 I2C)**
   - Shows AP IP, STA IP (if any), hub status, UART counters/errors.

4. **WiFi modes**
   - AP always enabled.
   - STA optional; configured via web UI; credentials saved persistently.

5. **HTTP server**
  - Minimal status page and minimal config page.
  - `/` status page.
  - `/config` WiFi configuration page.

6. **SignalR client**
   - Negotiate → WebSocket connect → handshake → send/receive.

### Arduino Uno firmware (separate sketch)

- Uses NeoSWSerial for the ESP32 link.
- Starts as a simple echo/loopback responder so ESP32 bring-up is easy.

## 5) Hardware/Physical Notes (Checklist)

- **Shared ground** between ESP32 and Uno is required.
- **Level shifting**: Uno TX is typically 5V; ESP32 RX is typically 3.3V tolerant only.
  - Add a level shifter or resistor divider on Uno→ESP32 line.
- Pins are intentionally **not** locked in this plan yet (already wired in your setup). We will parameterize them later.

## 6) Milestones (Implementation + Acceptance Tests)

Each milestone should be a clean commit point (even if you don’t commit, treat it as a stable checkpoint).

### M0 — Baseline Build/Upload/Monitor

**Implement**
- Periodic USB Serial output (already exists).

**Acceptance**
- `platformio run` succeeds.
- `platformio run -t upload` succeeds.
- Monitor shows periodic output at 115200.

---

### M1 — ESP32 UART Bridge (ESP32 Only)

**Implement**
- Initialize ESP32 UART for Uno link at 9600.
- Bridge bytes both directions for debug:
  - USB `Serial` → Uno UART
  - Uno UART → USB `Serial`
- Add counters:
  - `uart_rx_bytes`, `uart_tx_bytes`

**Acceptance**
- Firmware boots with Uno disconnected (no crash).
- With Uno attached (or a USB-UART adapter simulating it), bidirectional traffic is visible on USB monitor.

---

### M2 — Uno NeoSWSerial Loopback (Separate Sketch)

**Implement**
- Uno reads from NeoSWSerial and echoes back with a prefix (e.g., `UNO:`) and newline.
- Optional: also mirror to Uno `Serial` for local debug.

**Acceptance**
- From ESP32 monitor, sending text results in a correct echoed response.
- Run for 5–10 minutes; verify no lockups and no obvious framing corruption.

---

### M3 — SSD1306 OLED Bring-up

**Implement**
- Add OLED library dependencies.
- Initialize I2C + display.
- Show a simple status screen:
  - WiFi mode + IP (shows STA IP when connected, otherwise AP IP)
  - UART RX/TX counters

**Acceptance**
- OLED renders reliably after power cycle.
- OLED shows the current IP address.
- Counters update when serial traffic occurs.

---

### M4 — STA Attempt (60s) + AP Fallback + Status Webpage

**Implement**
- On boot, attempt to connect to a local WiFi AP in STA mode for **at least 60 seconds**.
  - Use credentials loaded from persisted storage (Preferences/NVS).
- If STA connect fails after the timeout, start SoftAP.
- Start a minimal HTTP server with:
  - `GET /` → status (WiFi mode + IP info, uptime, UART counters)
  - Provide a link to `/config`.

**Acceptance**
- Device attempts STA for ~60 seconds before falling back.
- If STA succeeds:
  - Serial log prints `STA connected` and a `STA IP`.
  - Visiting `http://<sta_ip>/` shows the status page with WiFi mode `STA`.
- If STA fails:
  - Serial log prints AP SSID/IP and the HTTP URL.
  - Phone/laptop connects to AP and visiting `http://192.168.4.1/` (default) shows the status page with WiFi mode `AP`.

---

### M5 — Config Page + Persistent Storage

**Implement**
- Add a minimal WiFi config webpage:
  - `GET /config` shows a form to set STA credentials.
  - `POST /config` saves the provided values.
- Persist values in NVS using `Preferences`:
  - Store at minimum:
    - `sta_ssid`
    - `sta_pass`
  - (Later) store `server_origin` for SignalR.
- On boot, retrieve the stored credentials and attempt STA connect using them before falling back to AP.
- After successfully saving config, reboot (simple and reliable for early milestones).

**Acceptance**
- When in AP mode, you can browse to `http://192.168.4.1/config`, enter SSID/password, and submit.
- Device responds that settings are saved and reboots.
- After reboot:
  - The device attempts STA connect using the saved credentials.
  - If it connects, `GET /` is reachable at `http://<sta_ip>/` and shows WiFi mode `STA`.
  - If it does not connect, it falls back to AP and `GET /` remains reachable at `http://192.168.4.1/`.

---

### M6 — STA Connect (AP Always On)

**Implement**
- Boot sequence:
  - Start AP immediately.
  - If STA credentials exist, attempt STA connect.
  - Keep AP running regardless of STA outcome.
- OLED shows:
  - AP IP
  - STA status (connecting/connected/failed)
  - STA IP when connected

**Acceptance**
- With valid credentials: STA connects and OLED shows STA IP.
- With invalid credentials: device remains reachable on AP; config still works.

---

### M7 — SignalR Connect-Only (No Relay Yet)

**Implement**
- Add a WebSocket client and JSON parser.
- Implement SignalR steps:
  1. HTTP POST to `${server_origin}/telemetryHub/negotiate?negotiateVersion=1`
  2. Parse negotiate response
  3. Open WebSocket to the provided URL
  4. Send SignalR handshake frame + `0x1e`
  5. Receive loop: parse frames (initially just log them)
- OLED shows hub status: connecting / connected / error.

**Acceptance**
- With hub running and reachable: device transitions to “connected”.
- If hub is down: device retries with backoff; UART bridge and OLED remain responsive.

Important: “localhost” is never correct for the ESP32. If you’re running the hub on your laptop while connected to the ESP32 AP, use the laptop’s AP-side IP (often `192.168.4.2`).

---

### M8 — Define the Relay Message Contract (Text-First)

**Implement**
- Keep the initial contract simple:
  - Serial→Hub: each newline-delimited line becomes one hub invocation.
  - Hub→Serial: a hub-to-device message becomes a line sent to Uno UART with `\n`.
- Add basic protections:
  - Max line length (drop + count if exceeded)
  - Drop/queue policy when hub disconnected

**Acceptance**
- Uno-originated lines show up at the hub.
- Hub-originated lines appear at the Uno.

---

### M9 — Integration + Resilience

**Implement**
- Combine everything in a non-blocking main loop:
  - UART read/write
  - OLED refresh (periodic)
  - WiFi state machine
  - SignalR connection state machine
- Reconnection/backoff for hub.
- Add counters and last error string/enum.

**Acceptance**
- Recover without reboot when:
  - WiFi disappears and returns
  - hub restarts
  - Uno is unplugged/replugged
- Device remains configurable via AP during failures.

## 7) Testing Commands (PlatformIO)

- Build: `platformio run`
- Upload: `platformio run -t upload`
- Monitor: `platformio device monitor -b 115200`

## 8) Troubleshooting & Field Notes

- If upload fails with “port busy”:
  - Use Sysinternals Process Explorer (`Ctrl+F`, search `COM7`) or Sysinternals `handle.exe -a COM7` to find which process holds the port.
- If serial data is garbled:
  - Confirm baud matches on both sides.
  - Confirm common ground.
  - Confirm level shifting on Uno→ESP32.
- If SignalR won’t connect:
  - Verify `server_origin` is reachable from the ESP32’s current network.
  - Don’t use `localhost`.

## 9) Deferred Decisions (Explicitly Not Locked Yet)

- Exact ESP32 UART pins for the Uno link.
- Exact Uno pins for NeoSWSerial.
- OLED I2C address if it differs from the typical value in your module.

These are intentionally left open because your hardware is already wired; we can add them as constants/config once you confirm the actual pin map.

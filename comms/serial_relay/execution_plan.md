# Execution Plan — ESP32 Serial Relay → WebSocket Relay (optional SignalR bridge)

Date: 2026-02-09

## 1) Goal

Build an ESP32-based “serial relay” that:

- Communicates **bidirectionally** with an Arduino Uno over a dedicated serial link.
  - **Uno side**: uses NeoSWSerial (requirement).
  - **ESP32 side**: uses a **hardware UART** (recommended; NeoSWSerial is AVR-focused and not an ESP32 fit).
- Connects to WiFi as a station (STA) when possible.
- **Always** provides a fallback WiFi Access Point (AP) with a minimal configuration webpage (because upstream AP is not guaranteed).
- Hosts a **WebSocket-ish** endpoint so other machines can send commands and receive serial data.
- (Optional) If you want official SignalR clients, run a small **PC bridge service** that exposes a SignalR hub and talks to the ESP32 over WebSockets.
- Displays status on an SSD1306 OLED (128x32) via I2C.

The plan is organized into milestones you can build/upload/test independently.

## 2) Current Repository State

- Target: ESP32 DevKit v1, Arduino framework (PlatformIO env `esp32doit-devkit-v1`).
- Entry point: `src/main.cpp`.
- OLED (SSD1306 128x32, I2C) is now wired up using Adafruit SSD1306/GFX dependencies.

## 3) Device-Hosted WebSocket Endpoint (Decision)

The ESP32 hosts the endpoint other machines connect to.

### Primary transport

- **WebSocket server** hosted on the ESP32
- Proposed port: **81** (keeps HTTP server on port 80)
- Path: `/` (default)

Example URLs:
- If ESP32 is in AP mode: `ws://192.168.4.1:81/`
- If ESP32 is on your LAN (STA mode): `ws://<sta_ip>:81/`

### Compatibility note (SignalR)

Official ASP.NET Core SignalR clients (C#) cannot speak directly to an arbitrary WebSocket server without SignalR’s negotiate + framing protocol.

If you need official SignalR clients later, the recommended approach is:

- Run a small ASP.NET Core service on a PC/RPi that hosts a SignalR hub (e.g. `/telemetryHub`)
- That service connects to the ESP32 via its WebSocket endpoint and bridges messages

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
  - Shows AP IP, STA IP (if any), UART counters/errors.

4. **WiFi modes**
   - AP always enabled.
   - STA optional; configured via web UI; credentials saved persistently.

5. **HTTP server**
  - Minimal status page and minimal config page.
  - `/` status page.
  - `/config` WiFi configuration page.

6. **WebSocket server (device-hosted)**
  - Accepts client connections.
  - Broadcasts serial lines to clients.
  - Receives client messages and forwards them to the Uno UART.

### Arduino Uno firmware (separate sketch)

- Uses NeoSWSerial for the ESP32 link.
- Starts as a simple echo/loopback responder so ESP32 bring-up is easy.

## 5) Hardware/Physical Notes (Checklist)

- **Shared ground** between ESP32 and Uno is required.
- **Level shifting**: Uno TX is typically 5V; ESP32 RX is typically 3.3V tolerant only.
  - Add a level shifter or resistor divider on Uno→ESP32 line.

## 5.1) Pin Map (Current Firmware Defaults)

These are the pin assignments currently used by the ESP32 firmware in `src/main.cpp`. If your wiring differs, update the constants in the firmware.

### OLED (SSD1306 128x32) — I2C

- **SDA**: GPIO **21** (ESP32 default)
- **SCL**: GPIO **22** (ESP32 default)
- **I2C address**: **0x3C**

Notes:
- Many SSD1306 modules are `0x3C`, but some are `0x3D`.

### Arduino Uno Link — ESP32 Serial2 (Hardware UART)

- **Baud**: **9600**
- **ESP32 RX2**: GPIO **16** (connect to **Uno TX** through level shift/divider)
- **ESP32 TX2**: GPIO **17** (connect to **Uno RX**)

Notes:
- NeoSWSerial is an Uno-side requirement; on the ESP32 we intentionally use a hardware UART.
- If you change pins, keep them on valid ESP32 GPIOs and update the `kUnoUartRxPin` / `kUnoUartTxPin` constants.

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

### M7 — WebSocket Server Bring-up (Device-Hosted)

**Implement**
- Add a WebSocket **server** on the ESP32 (separate port from the HTTP server).
- Log connect/disconnect and incoming text messages.
- Keep firmware responsive (UART + OLED + HTTP must keep working).

**Acceptance**
- From a laptop/PC, you can connect to the ESP32 WebSocket endpoint and send/receive test messages.
- UART + OLED + HTTP remain responsive while WebSocket clients connect.

---

### M8 — Define the Relay Message Contract (Text-First)

**Implement**
- Keep the initial contract simple:
  - Serial→WebSocket clients: each newline-delimited serial line is broadcast as a WebSocket text message.
  - WebSocket client→Serial: each received WebSocket text message is sent to the Uno UART with `\n` appended.
- Add basic protections:
  - Max line length (drop + count if exceeded)
  - Drop/queue policy when no WebSocket clients are connected (initially: just drop)

**Acceptance**
- Uno-originated lines show up at the WebSocket client.
- Client-originated lines appear at the Uno.

---

### M9 — Integration + Resilience

**Implement**
- Combine everything in a non-blocking main loop:
  - UART read/write
  - OLED refresh (periodic)
  - WiFi state machine
  - WebSocket server loop
- Add counters and last error string/enum.

**Acceptance**
- Recover without reboot when:
  - WiFi disappears and returns
  - a client disconnects/reconnects
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
- If WebSocket client won’t connect:
  - Confirm you’re connecting to the ESP32’s current IP (STA IP or AP IP).
  - If you’re connected to the ESP32 AP, the ESP32 is typically `192.168.4.1`.
  - If you’re on LAN STA, use the printed STA IP.

## 9) Deferred Decisions (Explicitly Not Locked Yet)

- Exact Uno pins for NeoSWSerial.
- OLED I2C address **if** your module is `0x3D` instead of `0x3C`.
- ESP32 UART/I2C pins **if** your wiring differs from the defaults documented above.

These are intentionally left open because your hardware is already wired; we can add them as constants/config once you confirm the actual pin map.

## 10) PCB Capture Checklist (KiCad-Friendly)

If you think you’ll eventually spin a PCB, capturing the details below *now* (while the prototype works) saves a lot of rework later. KiCad cares most about **nets**, so the goal is to lock down a connector-oriented **pin map** and stable **net names**.

Process rule for this repo: whenever we add a new hardware part or change wiring, we must update this section.

### 10.1) Snapshot the Known Parts (Current Prototype)

Parts currently in use in this project:

- **ESP32 DevKit v1** (PlatformIO env: `esp32doit-devkit-v1`)
- **Arduino Uno** (external firmware, NeoSWSerial required on Uno side)
- **OLED**: SSD1306 128x32, I2C
- **Level shifting** (required): Uno TX (5V) → ESP32 RX (3.3V)
- **Power**: typically USB 5V into ESP32 DevKit (onboard 3.3V regulator)

If your build uses additional items (buck regulator, battery, external sensor, enclosure), add them here as soon as they exist.

### 10.2) Capture the Pin Map as Nets (Recommended Net Names)

Use these net names consistently in documentation and later in the KiCad schematic:

- `+5V`, `+3V3`, `GND`
- `I2C_SDA`, `I2C_SCL`
- `UART_UNO_TX_5V` (Uno TX before shifting), `UART_ESP_RX_3V3` (ESP RX after shifting)
- `UART_ESP_TX_3V3` (ESP TX), `UART_UNO_RX_5V` (Uno RX)
- If WebSocket client won’t connect:
### 10.3) Known Pin Connections (as implemented in firmware defaults)

These reflect the current defaults in `src/main.cpp` and typical ESP32 DevKit wiring.

#### ESP32 DevKit v1 ↔ OLED (SSD1306, I2C)

| Function | Net Name | ESP32 GPIO | OLED Pin (typical module) | Notes |
|---|---:|---:|---|---|
| I2C SDA | `I2C_SDA` | 21 | SDA | `Wire.begin()` defaults to 21/22 on ESP32 |
| I2C SCL | `I2C_SCL` | 22 | SCL | |
| Power | `+3V3` | 3V3 | VCC | Many OLED modules accept 3.3V; verify yours |
| Ground | `GND` | GND | GND | |

OLED module details to capture:
- **I2C address**: default in firmware is `0x3C` (some modules are `0x3D`)
- If the OLED module includes pull-ups, note the approximate pull-up value (commonly 4.7k–10k)

#### ESP32 DevKit v1 ↔ Arduino Uno (UART link)

| Function | Net Name | ESP32 GPIO | Uno Signal | Notes |
|---|---:|---:|---|---|
| ESP32 RX2 (Serial2 RX) | `UART_ESP_RX_3V3` | 16 | Uno TX (via shifter) | **Must** level shift Uno TX 5V → 3.3V |
| ESP32 TX2 (Serial2 TX) | `UART_ESP_TX_3V3` | 17 | Uno RX | 3.3V is usually read as HIGH by Uno; verify if issues |
| Ground | `GND` | GND | GND | Common ground required |

UART details to capture:
- **Baud**: 9600
- **Framing**: 8N1
- **Protocol**: newline-delimited UTF-8/ASCII text (for early milestones)

#### Level Shifting (Uno TX → ESP32 RX)

Capture which exact method you used (this matters for the PCB BOM):

- **Resistor divider** (common/simple): `UART_UNO_TX_5V` → (R1) → `UART_ESP_RX_3V3` with (R2) to GND
  - Record **R1/R2 values** you used (example: 2k/1k, 10k/20k, etc.)
- **Level shifter IC** (more robust): record the exact IC part number and channel used

### 10.4) Connector Decisions (What to Lock Down Early)

For KiCad + PCB layout, it helps to decide how you want to physically connect everything:

- OLED: 4-pin header (`GND`, `VCC`, `SCL`, `SDA`) — confirm pin order on your module
- Uno link: 3-pin or 4-pin header (`GND`, `TX`, `RX`, optional `+5V` if you plan to power Uno)
- Power input: USB only vs a dedicated connector (JST, barrel, screw terminal)

### 10.5) Files/Formats That Convert Cleanly to KiCad

- Keep the tables above up to date (Markdown is fine).
- Optional but best: create a KiCad schematic early even if it’s “connectors + net labels only”.
- If you want a quick-importable format later, maintain a simple CSV pin map with columns:
  `Connector,PinNumber,NetName,Voltage,Direction,Notes`

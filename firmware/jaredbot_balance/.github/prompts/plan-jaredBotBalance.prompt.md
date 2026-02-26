## Plan: Persist Tuning Settings on Arduino Uno

Add non-volatile persistence for JaredBot tuning parameters using the Arduino Uno’s built-in EEPROM (Uno does not support ESP32-style NVS). Persist a trim offset (not the raw auto-calibrated setpoint) plus PID and Kalman parameters, load them on boot with sane defaults, and save after each successful CSV `C:` command (with write-debounce and integrity checks).

**Steps**
1. Confirm persistence mechanism
   - Use AVR EEPROM via `EEPROM.h` (1KB available) instead of NVS.

2. Define the persisted settings model
   - Introduce two concepts:
     - `calibratedZeroDeg`: computed at boot from the 400-sample calibration (not persisted).
     - `trimDeg`: persisted offset; applied as `setpointDeg = calibratedZeroDeg + trimDeg`.
   - Persist these tunables:
     - `trimDeg`
     - `kp`, `ki`, `kd`
     - `Q_angle`, `Q_gyro`, `R_angle`, `K1`
   - Keep `setpointDeg` as the controller’s effective setpoint (derived value; not persisted).

3. Add an EEPROM record format with integrity protection
   - Create an EEPROM “record” struct containing:
     - `magic` (constant), `version`, `seq` (monotonic), payload (settings), and `crc`.
   - Store two copies (slot A and slot B) in EEPROM:
     - On boot: read both, validate `magic/version/crc`, pick the record with the highest `seq`.
     - If neither record is valid: use compile-time defaults (existing constants) and `trimDeg = 0`.

4. Implement EEPROM load/save helpers (non-ISR)
   - Provide helpers (either as a small module or kept in main):
     - `loadSettingsFromEeprom()` → returns “valid or not” and fills `trimDeg`/PID/Kalman.
     - `scheduleSettingsSave()` → marks dirty and timestamps last change.
     - `flushSettingsSaveIfDue()` → if dirty and debounce elapsed, take an atomic snapshot and write the next slot.
   - Debounce recommendation: 500–2000ms after the last successful command to reduce write wear.

5. Wire load order into `setup()`
   - Start Serial/extSerial first.
   - Load persisted settings into RAM defaults (PID/Kalman/trim).
   - Run IMU calibration to compute `calibratedZeroDeg`.
   - Compute `setpointDeg = calibratedZeroDeg + trimDeg`.
   - Send `S:` once at startup showing the *effective* `setpointDeg` and current tunables.

6. Update command handling to operate on trim, not raw setpoint
   - For `C:SP,<x>` and the `SET` command’s first field:
     - Treat `<x>` as the *effective* desired `setpointDeg`.
     - Convert to trim: `trimDeg = desiredSetpointDeg - calibratedZeroDeg`, then recompute `setpointDeg`.
   - For `PID`/`KF`/`SET` updates:
     - Apply to RAM atomically (already done).
     - Respond immediately (`A:OK` then `S:` snapshot).
     - Call `scheduleSettingsSave()`.
   - For `RESET` and `KFRESET`:
     - Keep existing behavior; still `A:OK` + `S:`.
     - Only persist if they change persisted values (typically no); otherwise can skip saving.

7. Add “defaults if nothing saved” behavior
   - Defaults = current compile-time initialization values already in the firmware:
     - PID defaults (current `kp/ki/kd`), Kalman defaults (`Q_angle/Q_gyro/R_angle/K1`), `trimDeg = 0`.

**Relevant files**
- `d:/repos/JaredBot/firmware/jaredbot_balance/src/main.cpp` — add `trimDeg`/`calibratedZeroDeg`, adjust calibration + `SP/SET` semantics, call load/save helpers, and flush pending saves in `loop()`.
- (Optional, recommended) `d:/repos/JaredBot/firmware/jaredbot_balance/src/settings_eeprom.h` and `.../src/settings_eeprom.cpp` — isolate EEPROM record logic + CRC + slot selection.

**Verification**
1. Build: `C:\Users\pulcher\.platformio\penv\Scripts\platformio.exe run`.
2. Runtime:
   - Boot with blank EEPROM → verify `S:` shows defaults and calibrated `setpointDeg`.
   - Send `C:PID,...` → verify immediate `A:OK` then `S:` reflects change.
   - Power-cycle → verify settings persist.
   - Send multiple commands quickly → verify only one EEPROM write occurs after debounce.

**Decisions**
- Uno uses EEPROM (not NVS).
- Persist trim offset (`trimDeg`) rather than raw calibrated setpoint.
- Save after every successful `C:` command, but debounce to protect EEPROM endurance.

**Further Considerations**
1. Debounce interval: 500ms (fast tuning) vs 2000ms (more wear protection).

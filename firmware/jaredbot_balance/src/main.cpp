#include <Arduino.h>
#include <MsTimer2.h>
#include <MPU6050.h>
#include <Wire.h>
#include <NeoSWSerial.h>
#include "settings_eeprom.h"

// Use analog pins as digital: A3 = 17 (RX), A2 = 16 (TX)
NeoSWSerial extSerial(17, 16); // RX, TX

MPU6050 mpu6050;
int16_t ax, ay, az, gx, gy, gz;

// TB6612 pins
const int right_R1 = 8;
const int right_R2 = 12;
const int PWM_R = 10;
const int left_L1 = 7;
const int left_L2 = 6;
const int PWM_L = 9;

// ---------------- ENCODERS ----------------
// Keyestudio KS0193 kit wiring (per official docs):
//   Right motor Hall encoder pulse -> D4
//   Left  motor Hall encoder pulse -> D5
// These are single-channel pulse outputs (not full quadrature), so we count rising edges.
// If the connector orientation flips left/right, swap the pin constants.
static const uint8_t ENC_RIGHT_PIN = 4; // D4
static const uint8_t ENC_LEFT_PIN  = 5; // D5

static const bool ENC_RIGHT_INVERT = false;
static const bool ENC_LEFT_INVERT  = false;

volatile long encRightCount = 0;
volatile long encLeftCount = 0;
static uint8_t encLastPortD = 0;

static inline void snapshotEncoders(long &left, long &right)
{
    noInterrupts();
    left = encLeftCount;
    right = encRightCount;
    interrupts();
}

static inline void encoderPollUpdate()
{
    // NeoSWSerial uses pin-change ISRs internally; to avoid ISR vector conflicts,
    // we do fast polling here (direct port read) and count rising edges.
    const uint8_t nowPortD = PIND;
    const uint8_t changed = nowPortD ^ encLastPortD;
    encLastPortD = nowPortD;

    if (changed & _BV(4)) {
        if (nowPortD & _BV(4)) {
            encRightCount += (ENC_RIGHT_INVERT ? -1 : 1);
        }
    }
    if (changed & _BV(5)) {
        if (nowPortD & _BV(5)) {
            encLeftCount += (ENC_LEFT_INVERT ? -1 : 1);
        }
    }
}

// Angle parameters
float Angle;
// Setpoint/trim angle used by the balance controller (calibrated at startup).
// Historically called `angle0`.
float setpointDeg = 1.8;
// Auto-calibrated baseline at boot (not persisted).
float calibratedZeroDeg = 0.0;
// Persisted offset applied after calibration.
float trimDeg = 0.0;
float Gyro_x, Gyro_y, Gyro_z;

// Kalman filter variables
float Q_angle = 0.001;
float Q_gyro = 0.003;
float R_angle = 0.5;
char C_0 = 1;
// Control-loop timestep (seconds). Updated from micros() at runtime.
float dt = 0.005;
float K1 = 0.05;

// Control-loop dt statistics (microseconds) for jitter validation.
static uint32_t dtLastUs = 0;
static uint32_t dtMinUs = 0xFFFFFFFFu;
static uint32_t dtMaxUs = 0;
static uint32_t dtSumUs = 0;
static uint16_t dtCount = 0;

float K_0, K_1, t_0, t_1;
float angle_err;
float q_bias;

float angle = 0;
float angle_speed = 0;
float angleY_one = 0;

float Pdot[4] = {0, 0, 0, 0};
float P[2][2] = {{1, 0}, {0, 1}};
float PCt_0, PCt_1, E;

// PD parameters
double kp = 34, ki = 0, kd = 0.62;
int PD_pwm;
float pwm1 = 0, pwm2 = 0;

// PID state
volatile float pidIntegral = 0;

// Telemetry timers
unsigned long lastDebug = 0;     // USB Serial status interval
unsigned long lastCSV = 0;       // 50 Hz extSerial
unsigned long lastSettingsSent = 0;
const unsigned long settingsIntervalMs = 20000;

// NOTE: extSerial uses NeoSWSerial (bit-banged). At 9600 baud, each character costs ~1ms of CPU.
// A long CSV line can easily consume tens of milliseconds and starve the 200 Hz control loop.
// Keep telemetry low-rate unless you increase the baud rate.
static const unsigned long telemetryIntervalMs = 500; // 2 Hz

// Encoder reporting (keep separate so we don't break existing T: CSV parsing)
unsigned long lastEncReport = 0;
const unsigned long encReportIntervalMs = 200; // 5 Hz

// Motor deadband compensation (0 disables). Tunable at runtime via ESP32 command.
static uint8_t minPwm = 0;

// EEPROM save debounce (write endurance + avoid blocking too often)
const unsigned long settingsSaveDebounceMs = 1000;
bool settingsSaveDirty = false;
unsigned long settingsSaveDueAt = 0;
uint32_t settingsEepromSeq = 0;
uint8_t settingsEepromSlot = 255;

static inline uint8_t computeNextSettingsSlot(uint8_t currentSlot)
{
    if (currentSlot == 0) return 1;
    if (currentSlot == 1) return 0;
    return 0;
}

static PersistedSettings snapshotPersistedSettings()
{
    PersistedSettings s;
    noInterrupts();
    s.trimDeg = trimDeg;
    s.kp = (float)kp;
    s.ki = (float)ki;
    s.kd = (float)kd;
    s.Q_angle = Q_angle;
    s.Q_gyro = Q_gyro;
    s.R_angle = R_angle;
    s.K1 = K1;
    interrupts();
    return s;
}

static void debugPrintPersistedCandidate(const __FlashStringHelper *tag, const PersistedSettings &s)
{
    // USB Serial only: keep extSerial clean for the ESP32 command protocol.
    Serial.print(F("[EEPROM] "));
    Serial.print(tag);
    Serial.print(F(" trimDeg=")); Serial.print(s.trimDeg, 4);
    Serial.print(F(" kp=")); Serial.print(s.kp, 4);
    Serial.print(F(" ki=")); Serial.print(s.ki, 4);
    Serial.print(F(" kd=")); Serial.print(s.kd, 4);
    Serial.print(F(" Q_angle=")); Serial.print(s.Q_angle, 6);
    Serial.print(F(" Q_gyro=")); Serial.print(s.Q_gyro, 6);
    Serial.print(F(" R_angle=")); Serial.print(s.R_angle, 6);
    Serial.print(F(" K1=")); Serial.print(s.K1, 4);
    Serial.print(F(" | seq=")); Serial.print(settingsEepromSeq);
    Serial.print(F(" slot=")); Serial.print(settingsEepromSlot);
    Serial.print(F(" nextSlot=")); Serial.print(computeNextSettingsSlot(settingsEepromSlot));
    Serial.print(F(" dirty=")); Serial.print(settingsSaveDirty ? 1 : 0);
    Serial.print(F(" dueAt=")); Serial.print(settingsSaveDueAt);
    Serial.println();

    // Helpful runtime context for setpoint vs persisted trim.
    float sp, cz, tr;
    noInterrupts();
    sp = setpointDeg;
    cz = calibratedZeroDeg;
    tr = trimDeg;
    interrupts();
    Serial.print(F("[EEPROM] "));
    Serial.print(tag);
    Serial.print(F(" setpointDeg=")); Serial.print(sp, 4);
    Serial.print(F(" calibratedZeroDeg=")); Serial.print(cz, 4);
    Serial.print(F(" (persisted is trimDeg=")); Serial.print(tr, 4);
    Serial.println(F(")"));
}

static void debugPrintPersistedValuesOnly(const __FlashStringHelper *tag, const PersistedSettings &s)
{
    // USB Serial only: keep extSerial clean for the ESP32 command protocol.
    Serial.print(F("[EEPROM] "));
    Serial.print(tag);
    Serial.print(F(" trimDeg=")); Serial.print(s.trimDeg, 4);
    Serial.print(F(" kp=")); Serial.print(s.kp, 4);
    Serial.print(F(" ki=")); Serial.print(s.ki, 4);
    Serial.print(F(" kd=")); Serial.print(s.kd, 4);
    Serial.print(F(" Q_angle=")); Serial.print(s.Q_angle, 6);
    Serial.print(F(" Q_gyro=")); Serial.print(s.Q_gyro, 6);
    Serial.print(F(" R_angle=")); Serial.print(s.R_angle, 6);
    Serial.print(F(" K1=")); Serial.print(s.K1, 4);
    Serial.print(F(" | seq=")); Serial.print(settingsEepromSeq);
    Serial.print(F(" slot=")); Serial.print(settingsEepromSlot);
    Serial.println();
}

// Command buffer for ESP32
String cmdBuffer = "";

// Forward declarations
void DSzhongduan();
void sendSettingsRecord();
bool handleEsp32CommandLine(const String &line);
void scheduleSettingsSave();
void flushSettingsSaveIfDue(unsigned long now);
void angle_calculate(int16_t ax, int16_t ay, int16_t az,
                     int16_t gx, int16_t gy, int16_t gz,
                     float dt, float Q_angle, float Q_gyro,
                     float R_angle, float C_0, float K1);
void Kalman_Filter(double angle_m, double gyro_m, float dt);
void Yiorderfilter(float angle_m, float gyro_m, float dt);
void PD();
void anglePWM();

// Control loop scheduling (runs in loop() at ~200 Hz).
static const uint32_t kControlPeriodUs = 5000;
static uint32_t lastControlUs = 0;

static inline float clampf(float v, float lo, float hi);

static inline float clampDt(float dtSeconds)
{
    // Prevent huge dt (e.g., serial blocking) from destabilizing the filter/controller.
    // Also avoid dt=0 which would freeze the integrator/estimator.
    return clampf(dtSeconds, 0.001f, 0.020f);
}

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline void resetControllerState()
{
    noInterrupts();
    pidIntegral = 0;
    interrupts();
}

static inline void resetKalmanState()
{
    noInterrupts();
    q_bias = 0;
    angle_err = 0;
    angle_speed = 0;
    Pdot[0] = Pdot[1] = Pdot[2] = Pdot[3] = 0;
    P[0][0] = 1; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 1;
    interrupts();
}

void scheduleSettingsSave()
{
    settingsSaveDirty = true;
    settingsSaveDueAt = millis() + settingsSaveDebounceMs;

    // Preview what will be saved once the debounce timer expires.
    PersistedSettings preview = snapshotPersistedSettings();
    debugPrintPersistedCandidate(F("scheduled"), preview);
}

void flushSettingsSaveIfDue(unsigned long now)
{
    if (!settingsSaveDirty) return;
    // Handle millis() wrap by signed subtraction.
    if ((long)(now - settingsSaveDueAt) < 0) return;

    PersistedSettings s = snapshotPersistedSettings();
    debugPrintPersistedCandidate(F("write-attempt"), s);

    bool ok = savePersistedSettings(s, settingsEepromSeq, settingsEepromSlot);
    if (ok) {
        settingsSaveDirty = false;
        debugPrintPersistedCandidate(F("write-ok"), s);
    } else {
        // Retry later if EEPROM write/verify failed.
        settingsSaveDueAt = now + 2000;
        debugPrintPersistedCandidate(F("write-failed"), s);
    }
}

void sendSettingsRecord()
{
    // Settings frame (CSV):
    //   S:<setpointDeg>,<kp>,<ki>,<kd>,<Q_angle>,<Q_gyro>,<R_angle>,<K1>\n

    // Take a consistent snapshot (these values are used inside the 5ms ISR).
    float sp, qa, qg, ra, k1;
    double _kp, _ki, _kd;
    noInterrupts();
    sp = setpointDeg;
    _kp = kp;
    _ki = ki;
    _kd = kd;
    qa = Q_angle;
    qg = Q_gyro;
    ra = R_angle;
    k1 = K1;
    interrupts();

    extSerial.print("S:");
    extSerial.print(sp, 4);
    extSerial.print(",");
    extSerial.print(_kp, 4);
    extSerial.print(",");
    extSerial.print(_ki, 4);
    extSerial.print(",");
    extSerial.print(_kd, 4);
    extSerial.print(",");
    extSerial.print(qa, 6);
    extSerial.print(",");
    extSerial.print(qg, 6);
    extSerial.print(",");
    extSerial.print(ra, 6);
    extSerial.print(",");
    extSerial.print(k1, 4);
    extSerial.println();
}

// ---------------- SETUP ----------------
void setup()
{
    pinMode(right_R1, OUTPUT);
    pinMode(right_R2, OUTPUT);
    pinMode(left_L1, OUTPUT);
    pinMode(left_L2, OUTPUT);
    pinMode(PWM_R, OUTPUT);
    pinMode(PWM_L, OUTPUT);

    // Encoders (Hall pulse inputs on D4/D5 via pin-change interrupt)
    pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
    pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
    encLastPortD = PIND;

    digitalWrite(right_R1, 1);
    digitalWrite(right_R2, 0);
    digitalWrite(left_L1, 0);
    digitalWrite(left_L2, 1);
    analogWrite(PWM_R, 0);
    analogWrite(PWM_L, 0);

    Wire.begin();
    Serial.begin(9600);
    delay(1500);
    extSerial.begin(9600);
    delay(1500);

    // Load persisted settings (EEPROM). Defaults remain if nothing valid is stored.
    PersistedSettings persisted;
    if (loadPersistedSettings(persisted, settingsEepromSeq, settingsEepromSlot)) {
        debugPrintPersistedValuesOnly(F("loaded"), persisted);
        trimDeg = persisted.trimDeg;
        kp = persisted.kp;
        ki = persisted.ki;
        kd = persisted.kd;
        Q_angle = persisted.Q_angle;
        Q_gyro = persisted.Q_gyro;
        R_angle = persisted.R_angle;
        K1 = persisted.K1;
        resetControllerState();
    } else {
        trimDeg = 0.0;
        Serial.println(F("[EEPROM] no valid record; using defaults"));
    }

    mpu6050.initialize();
    delay(2);

    // Auto-zero calibration
    float sum = 0;
    for (int i = 0; i < 400; i++) {
        mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float A = -atan2(ay, az) * (180 / PI);
        sum += A;
        delay(5);
    }
    calibratedZeroDeg = sum / 400.0;
    setpointDeg = calibratedZeroDeg + trimDeg;

    // Start the control scheduler now that calibration is complete.
    lastControlUs = micros();

    // Initialize encoder reporting baselines after calibration.
    lastEncReport = millis();

    // Send settings record once at startup (after calibration)
    sendSettingsRecord();
    lastSettingsSent = millis();

    // NOTE: Control loop runs in loop() (micros-scheduled). We avoid doing I2C in an ISR.
}

// ---------------- MAIN LOOP ----------------
void loop()
{
    // Sample encoders as frequently as possible (polling edge detector).
    encoderPollUpdate();

    unsigned long now = millis();
    static long lastEncLeft = 0;
    static long lastEncRight = 0;

    // ---------------- 200 Hz CONTROL LOOP ----------------
    // Run the controller as close to 200 Hz as possible, using measured dt.
    // This avoids timing jitter from software serial / I2C inside ISRs.
    const uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastControlUs) >= kControlPeriodUs) {
        const uint32_t elapsedUs = (uint32_t)(nowUs - lastControlUs);
        lastControlUs = nowUs;

        // Track real dt to validate timing stability.
        dtLastUs = elapsedUs;
        if (elapsedUs < dtMinUs) dtMinUs = elapsedUs;
        if (elapsedUs > dtMaxUs) dtMaxUs = elapsedUs;
        dtSumUs += elapsedUs;
        if (dtCount != 0xFFFFu) dtCount++;

        dt = clampDt(elapsedUs * 1e-6f);

        mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
        PD();
        anglePWM();
    }

    // Flush any pending EEPROM save (debounced)
    flushSettingsSaveIfDue(now);

    // ----------- Periodic settings record on extSerial -----------
    if (now - lastSettingsSent >= settingsIntervalMs) {
        sendSettingsRecord();
        lastSettingsSent = now;
    }

    // ----------- Receive commands from ESP32 -----------
    while (extSerial.available()) {
        char c = extSerial.read();
        // Accept either CR or LF as line terminator.
        // (If sender uses CRLF, we'll process on CR and ignore the following empty LF.)
        if (c == '\n' || c == '\r') {
            if (cmdBuffer.length() > 0) {
                bool ok = handleEsp32CommandLine(cmdBuffer);
                if (!ok) {
                    Serial.print("[ESP32 INVALID] ");
                    Serial.println(cmdBuffer);
                    extSerial.println("A:ERR");
                }
            }
            cmdBuffer = "";
        } else {
            // Prevent unbounded growth if the sender goes off the rails.
            if (cmdBuffer.length() < 120) {
                cmdBuffer += c;
            } else {
                cmdBuffer = "";
            }
        }
    }

    // ----------- USB Serial Debug -----------
    if (now - lastDebug >= 10000) {
        lastDebug = now;

        long l, r;
        snapshotEncoders(l, r);

        Serial.print("angle=");
        Serial.print(angle);
        Serial.print(" speed=");
        Serial.print(angle_speed);
        Serial.print(" pwm=");
        Serial.print(PD_pwm);
        Serial.print(" dt_us=");
        Serial.print(dtLastUs);
        Serial.print(" minPWM=");
        Serial.print(minPwm);
        Serial.print(" encL=");
        Serial.print(l);
        Serial.print(" encR=");
        Serial.print(r);
        Serial.println();
    }

    // ----------- Encoder CSV Telemetry (low-rate) -----------
    // Separate frame type so existing ESP32 parsing of T: does not break.
    // E: <encLeft>,<encRight>,<dEncLeft>,<dEncRight>
    if (now - lastEncReport >= encReportIntervalMs) {
        lastEncReport = now;

        long l, r;
        snapshotEncoders(l, r);
        const long dL = l - lastEncLeft;
        const long dR = r - lastEncRight;
        lastEncLeft = l;
        lastEncRight = r;

        // Only extSerial gets the structured frame; USB Serial stays human-readable.
        extSerial.print("E:");
        extSerial.print(l);
        extSerial.print(",");
        extSerial.print(r);
        extSerial.print(",");
        extSerial.print(dL);
        extSerial.print(",");
        extSerial.print(dR);
        extSerial.println();
    }

    // ----------- CSV Telemetry on extSerial -----------
    // NOTE: Keep this rate modest: NeoSWSerial @ 9600 can block and wreck control-loop timing.
    // If you need higher-rate telemetry, increase extSerial baud (and receiver baud) first.
    if (now - lastCSV >= telemetryIntervalMs) {
        lastCSV = now;

        // Optional lightweight heartbeat (printing every 20ms can add serial blocking/jitter).
        static unsigned long lastDot = 0;
        if (now - lastDot >= 1000) {
            lastDot = now;
            Serial.print(".");
        }
        
        // CSV: angle,angle_speed,gyro_x,pd_pwm,pwm1,pwm2,dt_last_us,dt_min_us,dt_max_us,dt_avg_us,dt_n
        const uint32_t dtAvgUs = (dtCount > 0) ? (dtSumUs / dtCount) : 0;
        const uint32_t dtMinOutUs = (dtMinUs == 0xFFFFFFFFu) ? 0 : dtMinUs;
        extSerial.print("T:");
        extSerial.print(angle, 2);
        extSerial.print(",");
        extSerial.print(angle_speed, 2);
        extSerial.print(",");
        extSerial.print(Gyro_x, 2);
        extSerial.print(",");
        extSerial.print(PD_pwm);
        extSerial.print(",");
        extSerial.print((int)pwm1);
        extSerial.print(",");
        extSerial.print((int)pwm2);
        extSerial.print(",");
        extSerial.print(dtLastUs);
        extSerial.print(",");
        extSerial.print(dtMinOutUs);
        extSerial.print(",");
        extSerial.print(dtMaxUs);
        extSerial.print(",");
        extSerial.print(dtAvgUs);
        extSerial.print(",");
        extSerial.print(dtCount);
        extSerial.println();

        // Reset dt window each time we emit a T: line.
        dtMinUs = 0xFFFFFFFFu;
        dtMaxUs = 0;
        dtSumUs = 0;
        dtCount = 0;
    }
}

    // ---------------- ISR ----------------
    // Legacy hook retained for compatibility, but intentionally unused.
    void DSzhongduan() {}

// ---------------- ESP32 COMMAND HANDLING ----------------
// Supported payload formats (all lines must start with "C:"):
//   CSV commands only:
//     C:SP,<setpoint>
//     C:PID,<kp>,<ki>,<kd>
//     C:KF,<Q_angle>,<Q_gyro>,<R_angle>
//     C:MINPWM,<0-255>  (motor deadband compensation; 0 disables)
//     C:SET,<setpoint>,<kp>,<ki>,<kd>,<Q_angle>,<Q_gyro>,<R_angle>,<K1>
//     C:RESET       (clears PID integral)
//     C:KFRESET     (resets Kalman internal state P/q_bias)
bool handleEsp32CommandLine(const String &line)
{
    if (!line.startsWith("C:")) {
        return false;
    }

    String payload = line.substring(2);
    payload.trim();
    const int payloadLen = payload.length();
    if (payloadLen == 0) {
        return false;
    }

    // Local staging values (so we apply atomically)
    bool touched = false;
    bool touchedPid = false;
    bool touchedSetpoint = false;

    float newSetpointDeg = setpointDeg;
    double newKp = kp, newKi = ki, newKd = kd;
    float newQAngle = Q_angle, newQGyro = Q_gyro, newRAngle = R_angle;
    float newK1 = K1;

    auto applyStaged = [&]() {
        if (!touched) {
            return;
        }

        // Constrain to sane ranges (keeps accidental bad commands from bricking control)
        newSetpointDeg = clampf(newSetpointDeg, -30.0f, 30.0f);
        newQAngle = clampf(newQAngle, 0.0f, 1.0f);
        newQGyro = clampf(newQGyro, 0.0f, 1.0f);
        newRAngle = clampf(newRAngle, 0.0f, 10.0f);
        newK1 = clampf(newK1, 0.0f, 1.0f);

        if (newKp < 0) newKp = 0;
        if (newKp > 200) newKp = 200;
        if (newKi < 0) newKi = 0;
        if (newKi > 50) newKi = 50;
        if (newKd < 0) newKd = 0;
        if (newKd > 20) newKd = 20;

        noInterrupts();
        if (touchedSetpoint) {
            // Persist trim, not the raw setpoint, since we auto-calibrate baseline each boot.
            trimDeg = clampf(newSetpointDeg - calibratedZeroDeg, -30.0f, 30.0f);
            setpointDeg = calibratedZeroDeg + trimDeg;
        }
        kp = newKp;
        ki = newKi;
        kd = newKd;
        Q_angle = newQAngle;
        Q_gyro = newQGyro;
        R_angle = newRAngle;
        K1 = newK1;
        if (touchedPid) {
            pidIntegral = 0;
        }
        interrupts();

        // Immediately show the exact persisted payload implied by these staged values.
        // (This is what will be written once scheduleSettingsSave() fires + debounce elapses.)
        PersistedSettings preview = snapshotPersistedSettings();
        debugPrintPersistedCandidate(F("after-applyStaged"), preview);
    };

    // -------- CSV command mode --------
    String cmd = payload;
    int firstComma = payload.indexOf(',');
    if (firstComma >= 0) {
        cmd = payload.substring(0, firstComma);
    }
    cmd.trim();
    cmd.toUpperCase();

    auto tokenAfter = [&](int &pos) -> String {
        if (pos >= payloadLen) return String("");
        int comma = payload.indexOf(',', pos);
        if (comma < 0) comma = payloadLen;
        String t = payload.substring(pos, comma);
        t.trim();
        pos = comma + 1;
        return t;
    };

    if (cmd == "RESET") {
        resetControllerState();
        extSerial.println("A:OK");
        sendSettingsRecord();
        lastSettingsSent = millis();
        return true;
    }
    if (cmd == "KFRESET") {
        resetKalmanState();
        extSerial.println("A:OK");
        sendSettingsRecord();
        lastSettingsSent = millis();
        return true;
    }

    int pos = (firstComma >= 0) ? (firstComma + 1) : payloadLen;

    if (cmd == "MINPWM") {
        String t0 = tokenAfter(pos);
        if (t0.length() == 0) return false;
        long v = t0.toInt();
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        minPwm = (uint8_t)v;
        extSerial.println("A:OK");
        return true;
    }

    if (cmd == "SP") {
        String t0 = tokenAfter(pos);
        if (t0.length() == 0) return false;
        newSetpointDeg = t0.toFloat();
        touched = true;
        touchedPid = true;
        touchedSetpoint = true;
        applyStaged();
        extSerial.println("A:OK");
        sendSettingsRecord();
        lastSettingsSent = millis();
        scheduleSettingsSave();
        return true;
    }

    if (cmd == "PID") {
        String t0 = tokenAfter(pos);
        String t1 = tokenAfter(pos);
        String t2 = tokenAfter(pos);
        if (t0.length() == 0 || t1.length() == 0 || t2.length() == 0) return false;
        newKp = t0.toFloat();
        newKi = t1.toFloat();
        newKd = t2.toFloat();
        touched = true;
        touchedPid = true;
        applyStaged();
        extSerial.println("A:OK");
        sendSettingsRecord();
        lastSettingsSent = millis();
        scheduleSettingsSave();
        return true;
    }

    // Single-parameter updates (CSV):
    //   C:KP,<kp>  C:KI,<ki>  C:KD,<kd>
    //   C:QA,<Q_angle>  C:QG,<Q_gyro>  C:RA,<R_angle>  C:K1,<K1>
    if (cmd == "KP" || cmd == "KI" || cmd == "KD" || cmd == "QA" || cmd == "QG" || cmd == "RA" || cmd == "K1") {
        String t0 = tokenAfter(pos);
        if (t0.length() == 0) return false;
        float v = t0.toFloat();

        if (cmd == "KP") {
            newKp = v;
            touchedPid = true;
        } else if (cmd == "KI") {
            newKi = v;
            touchedPid = true;
        } else if (cmd == "KD") {
            newKd = v;
            touchedPid = true;
        } else if (cmd == "QA") {
            newQAngle = v;
        } else if (cmd == "QG") {
            newQGyro = v;
        } else if (cmd == "RA") {
            newRAngle = v;
        } else if (cmd == "K1") {
            newK1 = v;
        }

        touched = true;
        applyStaged();
        extSerial.println("A:OK");
        sendSettingsRecord();
        lastSettingsSent = millis();
        scheduleSettingsSave();
        return true;
    }

    if (cmd == "KF") {
        String t0 = tokenAfter(pos);
        String t1 = tokenAfter(pos);
        String t2 = tokenAfter(pos);
        if (t0.length() == 0 || t1.length() == 0 || t2.length() == 0) return false;
        newQAngle = t0.toFloat();
        newQGyro = t1.toFloat();
        newRAngle = t2.toFloat();
        touched = true;
        applyStaged();
        extSerial.println("A:OK");
        sendSettingsRecord();
        lastSettingsSent = millis();
        scheduleSettingsSave();
        return true;
    }

    if (cmd == "SET") {
        // SET,<sp>,<kp>,<ki>,<kd>,<qa>,<qg>,<ra>,<k1>
        String t0 = tokenAfter(pos);
        String t1 = tokenAfter(pos);
        String t2 = tokenAfter(pos);
        String t3 = tokenAfter(pos);
        String t4 = tokenAfter(pos);
        String t5 = tokenAfter(pos);
        String t6 = tokenAfter(pos);
        String t7 = tokenAfter(pos);
        if (t0.length() == 0 || t1.length() == 0 || t2.length() == 0 || t3.length() == 0 ||
            t4.length() == 0 || t5.length() == 0 || t6.length() == 0 || t7.length() == 0) {
            return false;
        }
        newSetpointDeg = t0.toFloat();
        newKp = t1.toFloat();
        newKi = t2.toFloat();
        newKd = t3.toFloat();
        newQAngle = t4.toFloat();
        newQGyro = t5.toFloat();
        newRAngle = t6.toFloat();
        newK1 = t7.toFloat();
        touched = true;
        touchedPid = true;
        touchedSetpoint = true;
        applyStaged();
        extSerial.println("A:OK");
        sendSettingsRecord();
        lastSettingsSent = millis();
        scheduleSettingsSave();
        return true;
    }

    return false;
}

// ---------------- ANGLE CALC ----------------
void angle_calculate(int16_t ax, int16_t ay, int16_t az,
                     int16_t gx, int16_t gy, int16_t gz,
                     float dt, float Q_angle, float Q_gyro,
                     float R_angle, float C_0, float K1)
{
    Angle = -atan2(ay, az) * (180 / PI);
    Gyro_x = -gx / 131.0;
    Kalman_Filter(Angle, Gyro_x, dt);

    Gyro_z = -gz / 131.0;

    float angleAx = -atan2(ax, az) * (180 / PI);
    Gyro_y = -gy / 131.0;
    Yiorderfilter(angleAx, Gyro_y, dt);
}

// ---------------- KALMAN FILTER ----------------
void Kalman_Filter(double angle_m, double gyro_m, float dt)
{
    angle += (gyro_m - q_bias) * dt;
    angle_err = angle_m - angle;

    Pdot[0] = Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;

    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    q_bias += K_1 * angle_err;
    angle_speed = gyro_m - q_bias;
    angle += K_0 * angle_err;
}

// ---------------- FIRST ORDER FILTER ----------------
void Yiorderfilter(float angle_m, float gyro_m, float dt)
{
    angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}

// ---------------- PD CONTROL ----------------
void PD()
{
    // Balance error. Note: original code used (angle + angle0), where angle0 was a trim/setpoint.
    float err = angle + setpointDeg;

    // Integrate only if KI is enabled.
    if (ki != 0) {
        float i = pidIntegral + err * dt;
        // Anti-windup clamp; values are in (deg * sec).
        pidIntegral = clampf(i, -300.0f, 300.0f);
    } else {
        pidIntegral = 0;
    }

    double out = kp * err + ki * pidIntegral + kd * angle_speed;
    PD_pwm = (int)out;
}

// ---------------- MOTOR PWM ----------------
void anglePWM()
{
    pwm1 = -PD_pwm;
    pwm2 = -PD_pwm;

    if (pwm1 > 255) pwm1 = 255;
    if (pwm1 < -255) pwm1 = -255;
    if (pwm2 > 255) pwm2 = 255;
    if (pwm2 < -255) pwm2 = -255;

    // Optional deadband compensation: ensure a minimum duty when commanding non-zero.
    if (minPwm != 0) {
        if (pwm1 > 0 && pwm1 < minPwm) pwm1 = minPwm;
        if (pwm1 < 0 && -pwm1 < minPwm) pwm1 = -minPwm;
        if (pwm2 > 0 && pwm2 < minPwm) pwm2 = minPwm;
        if (pwm2 < 0 && -pwm2 < minPwm) pwm2 = -minPwm;
    }

    if (angle > 80 || angle < -80) {
        pwm1 = pwm2 = 0;
        pidIntegral = 0;
    }

    if (pwm2 >= 0) {
        digitalWrite(left_L1, LOW);
        digitalWrite(left_L2, HIGH);
        analogWrite(PWM_L, pwm2);
    } else {
        digitalWrite(left_L1, HIGH);
        digitalWrite(left_L2, LOW);
        analogWrite(PWM_L, -pwm2);
    }

    if (pwm1 >= 0) {
        digitalWrite(right_R1, LOW);
        digitalWrite(right_R2, HIGH);
        analogWrite(PWM_R, pwm1);
    } else {
        digitalWrite(right_R1, HIGH);
        digitalWrite(right_R2, LOW);
        analogWrite(PWM_R, -pwm1);
    }
}

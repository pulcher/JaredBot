#include <Arduino.h>
#include <NeoSWSerial.h>

#include "balance_controller.h"
#include "encoder_service.h"
#include "hardware_config.h"
#include "imu_filter.h"
#include "motor_driver.h"
#include "persistence.h"
#include "settings.h"
#include "telemetry.h"

namespace {

enum class FaultCode : uint8_t {
    None = 0,
    ExcessiveTilt,
    ImuUpdate,
    LoopOverrun,
    InvalidCommand,
    EepromWrite,
};

struct RuntimeContext
{
    hw::SystemMode mode = hw::SystemMode::Boot;
    FaultCode fault = FaultCode::None;
    bool settingsDirty = false;
    bool armToggleLatched = false;
    unsigned long settingsSaveDueAtMs = 0;
    uint32_t settingsSeq = 0;
    uint8_t settingsSlot = 255;

    uint32_t lastControlUs = 0;
    uint32_t dtLastUs = 0;
    uint32_t dtMinUs = 0xFFFFFFFFu;
    uint32_t dtMaxUs = 0;
    uint32_t dtSumUs = 0;
    uint16_t dtCount = 0;

    unsigned long lastUsbRuntimeTelemetryMs = 0;
    unsigned long lastUsbEncoderTelemetryMs = 0;
    unsigned long lastRelayRuntimeTelemetryMs = 0;
    unsigned long lastRelayEncoderTelemetryMs = 0;
    unsigned long lastSettingsTelemetryMs = 0;
    unsigned long lastStateTelemetryMs = 0;
    unsigned long lastMotorTestStepMs = 0;
    unsigned long lastImuTestMs = 0;
    unsigned long lastLoopJitterWarnMs = 0;

    long lastUsbLeftCount = 0;
    long lastUsbRightCount = 0;
    long lastRelayLeftCount = 0;
    long lastRelayRightCount = 0;
    long lastAverageCount = 0;
    float calibratedZeroDeg = 0.0f;
    ControlOutput lastOutput;
    int16_t rawLeftPwm = 0;
    int16_t rawRightPwm = 0;
    bool rawMotorOverride = false;
    String relayCommandBuffer;
    String usbCommandBuffer;
};

NeoSWSerial relaySerial(hw::kRelayRxPin, hw::kRelayTxPin);
MotorDriver motors;
EncoderService encoders;
ImuFilter imu;
BalanceController controller;
Telemetry telemetry(Serial, relaySerial);
RuntimeContext runtime;
ControlSettings settings;

float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

const __FlashStringHelper *faultName(FaultCode code)
{
    switch (code) {
        case FaultCode::None: return F("NONE");
        case FaultCode::ExcessiveTilt: return F("TILT");
        case FaultCode::ImuUpdate: return F("IMU");
        case FaultCode::LoopOverrun: return F("OVERRUN");
        case FaultCode::InvalidCommand: return F("CMD");
        case FaultCode::EepromWrite: return F("EEPROM");
    }
    return F("UNKNOWN");
}

hw::SystemMode defaultMode()
{
    switch (hw::kBringupMode) {
        case hw::BringupMode::Normal: return hw::SystemMode::Idle;
        case hw::BringupMode::TestImu: return hw::SystemMode::TestImu;
        case hw::BringupMode::TestMotors: return hw::SystemMode::TestMotors;
        case hw::BringupMode::TestEncoders: return hw::SystemMode::TestEncoders;
        case hw::BringupMode::TestSerial: return hw::SystemMode::TestSerial;
    }
    return hw::SystemMode::Idle;
}

void setFault(FaultCode code, const String &detail)
{
    runtime.fault = code;
    runtime.mode = hw::SystemMode::Fault;
    controller.disarm();
    motors.stop();
    telemetry.sendEvent(F("FAULT"), String((const __FlashStringHelper *)faultName(code)) + "," + detail);
}

void clearFault()
{
    runtime.fault = FaultCode::None;
    if (!controller.armed()) {
        runtime.mode = hw::SystemMode::Idle;
    }
}

void applySettingsToSubsystems()
{
    controller.setSettings(settings);
    controller.setCalibratedZero(runtime.calibratedZeroDeg);
    motors.setMinPwm(settings.minPwm);
    imu.setKalmanTuning(settings.qAngle, settings.qGyro, settings.rAngle, settings.k1);
}

void scheduleSettingsSave()
{
    runtime.settingsDirty = true;
    runtime.settingsSaveDueAtMs = millis() + 1000;
}

void saveSettingsIfDue(unsigned long nowMs)
{
    if (!runtime.settingsDirty) {
        return;
    }
    if ((long)(nowMs - runtime.settingsSaveDueAtMs) < 0) {
        return;
    }

    if (savePersistedSettings(settings, runtime.settingsSeq, runtime.settingsSlot)) {
        runtime.settingsDirty = false;
        telemetry.sendEvent(F("SAVE"), F("OK"));
    } else {
        setFault(FaultCode::EepromWrite, F("write_failed"));
    }
}

void sendStateTelemetry()
{
    telemetry.sendState(runtime.mode, controller.armed(), String((const __FlashStringHelper *)faultName(runtime.fault)));
}

void sendSettingsTelemetry()
{
    telemetry.sendSettings(settings, runtime.calibratedZeroDeg);
}

void resetDtWindow()
{
    runtime.dtMinUs = 0xFFFFFFFFu;
    runtime.dtMaxUs = 0;
    runtime.dtSumUs = 0;
    runtime.dtCount = 0;
}

uint32_t dtAverageUs()
{
    return runtime.dtCount > 0 ? (runtime.dtSumUs / runtime.dtCount) : 0;
}

long currentAverageCount(long left, long right)
{
    return (left + right) / 2;
}

void armControllerAtCurrentPosition()
{
    long left = 0;
    long right = 0;
    encoders.snapshot(left, right);
    controller.arm(currentAverageCount(left, right));
    runtime.lastAverageCount = currentAverageCount(left, right);
    runtime.mode = hw::SystemMode::Balance;
    telemetry.sendEvent(F("ARM"), F("1"));
}

void disarmController()
{
    controller.disarm();
    runtime.rawMotorOverride = false;
    runtime.rawLeftPwm = 0;
    runtime.rawRightPwm = 0;
    motors.stop();
    if (runtime.mode != hw::SystemMode::Fault) {
        runtime.mode = hw::SystemMode::Idle;
    }
    telemetry.sendEvent(F("ARM"), F("0"));
}

void handleButton()
{
    const bool pressed = digitalRead(hw::kButtonPin) == LOW;
    if (pressed && !runtime.armToggleLatched) {
        runtime.armToggleLatched = true;
        if (controller.armed()) {
            disarmController();
        } else if (runtime.mode == hw::SystemMode::Idle || runtime.mode == hw::SystemMode::Balance) {
            armControllerAtCurrentPosition();
        }
    } else if (!pressed) {
        runtime.armToggleLatched = false;
    }
}

void applyMotorTestPattern(unsigned long nowMs)
{
    const unsigned long phase = ((nowMs / 2000UL) % 4UL);
    switch (phase) {
        case 0: motors.apply(90, 90); break;
        case 1: motors.apply(-90, -90); break;
        case 2: motors.apply(90, -90); break;
        default: motors.stop(); break;
    }
}

void applyRawMotorOverride()
{
    motors.apply(runtime.rawLeftPwm, runtime.rawRightPwm);
}

void runBringupMode(unsigned long nowMs)
{
    switch (runtime.mode) {
        case hw::SystemMode::TestImu:
            break;

        case hw::SystemMode::TestMotors:
            if (runtime.rawMotorOverride) {
                applyRawMotorOverride();
            } else {
                applyMotorTestPattern(nowMs);
            }
            break;

        case hw::SystemMode::TestEncoders:
        case hw::SystemMode::TestSerial:
        case hw::SystemMode::Idle:
        case hw::SystemMode::Balance:
        case hw::SystemMode::Fault:
        case hw::SystemMode::Boot:
        case hw::SystemMode::Calibrating:
            break;
    }
}

String nextToken(String &payload)
{
    const int comma = payload.indexOf(',');
    String token;
    if (comma < 0) {
        token = payload;
        payload = "";
    } else {
        token = payload.substring(0, comma);
        payload = payload.substring(comma + 1);
    }
    token.trim();
    return token;
}

bool parseFloatToken(String &payload, float &out)
{
    const String token = nextToken(payload);
    if (token.length() == 0) {
        return false;
    }
    out = token.toFloat();
    return true;
}

bool parseIntToken(String &payload, long &out)
{
    const String token = nextToken(payload);
    if (token.length() == 0) {
        return false;
    }
    out = token.toInt();
    return true;
}

int16_t clampPwmCommand(long value)
{
    if (value > 255) return 255;
    if (value < -255) return -255;
    return (int16_t)value;
}

void acknowledge(Print &out, bool ok)
{
    out.println(ok ? F("A:OK") : F("A:ERR"));
}

void sendFullStatus()
{
    sendStateTelemetry();
    sendSettingsTelemetry();
}

bool setModeFromString(const String &mode)
{
    if (mode == F("IDLE")) {
        disarmController();
        runtime.mode = hw::SystemMode::Idle;
        return true;
    }
    if (mode == F("BALANCE")) {
        armControllerAtCurrentPosition();
        return true;
    }
    if (mode == F("TEST_IMU")) {
        disarmController();
        runtime.mode = hw::SystemMode::TestImu;
        return true;
    }
    if (mode == F("TEST_MOTORS")) {
        disarmController();
        runtime.mode = hw::SystemMode::TestMotors;
        return true;
    }
    if (mode == F("TEST_ENCODERS")) {
        disarmController();
        runtime.mode = hw::SystemMode::TestEncoders;
        return true;
    }
    if (mode == F("TEST_SERIAL")) {
        disarmController();
        runtime.mode = hw::SystemMode::TestSerial;
        return true;
    }
    return false;
}

bool handleCommandLine(const String &line)
{
    if (!line.startsWith("C:")) {
        return false;
    }

    String payload = line.substring(2);
    payload.trim();
    String command = nextToken(payload);
    command.toUpperCase();

    if (command == F("ARM")) {
        long value = 0;
        if (!parseIntToken(payload, value)) return false;
        if (value != 0) {
            armControllerAtCurrentPosition();
        } else {
            disarmController();
        }
        return true;
    }

    if (command == F("MODE")) {
        String mode = nextToken(payload);
        mode.toUpperCase();
        return setModeFromString(mode);
    }

    if (command == F("STATUS")) {
        sendFullStatus();
        return true;
    }

    if (command == F("SAVE")) {
        runtime.settingsDirty = true;
        runtime.settingsSaveDueAtMs = millis();
        return true;
    }

    if (command == F("RESET")) {
        controller.reset();
        return true;
    }

    if (command == F("KFRESET")) {
        imu.resetFilterState();
        return true;
    }

    if (command == F("CLEARFAULT")) {
        clearFault();
        return true;
    }

    if (command == F("STOP")) {
        runtime.rawMotorOverride = false;
        runtime.rawLeftPwm = 0;
        runtime.rawRightPwm = 0;
        motors.stop();
        if (!controller.armed() && runtime.mode != hw::SystemMode::Fault) {
            runtime.mode = hw::SystemMode::Idle;
        }
        return true;
    }

    if (command == F("ENCRESET")) {
        encoders.reset();
        runtime.lastUsbLeftCount = 0;
        runtime.lastUsbRightCount = 0;
        runtime.lastRelayLeftCount = 0;
        runtime.lastRelayRightCount = 0;
        runtime.lastAverageCount = 0;
        return true;
    }

    if (command == F("RAWPWM")) {
        long left = 0;
        long right = 0;
        if (!parseIntToken(payload, left) || !parseIntToken(payload, right)) return false;
        if (controller.armed() || runtime.mode == hw::SystemMode::Fault) return false;
        runtime.rawLeftPwm = clampPwmCommand(left);
        runtime.rawRightPwm = clampPwmCommand(right);
        runtime.rawMotorOverride = (runtime.rawLeftPwm != 0 || runtime.rawRightPwm != 0);
        runtime.mode = hw::SystemMode::TestMotors;
        applyRawMotorOverride();
        telemetry.sendEvent(F("RAWPWM"), String(runtime.rawLeftPwm) + "," + String(runtime.rawRightPwm));
        return true;
    }

    if (command == F("TRIM")) {
        float value = 0.0f;
        if (!parseFloatToken(payload, value)) return false;
        settings.trimDeg = clampf(value, -12.0f, 12.0f);
        applySettingsToSubsystems();
        scheduleSettingsSave();
        return true;
    }

    if (command == F("KP") || command == F("KI") || command == F("KD") ||
        command == F("HKP") || command == F("HKI") || command == F("VKP") ||
        command == F("QA") || command == F("QG") || command == F("RA") ||
        command == F("K1") || command == F("MINPWM")) {
        float value = 0.0f;
        if (!parseFloatToken(payload, value)) return false;
        if (command == F("KP")) settings.kp = clampf(value, 0.0f, 200.0f);
        else if (command == F("KI")) settings.ki = clampf(value, 0.0f, 50.0f);
        else if (command == F("KD")) settings.kd = clampf(value, 0.0f, 20.0f);
        else if (command == F("HKP")) settings.holdKp = clampf(value, -1.0f, 1.0f);
        else if (command == F("HKI")) settings.holdKi = clampf(value, -0.5f, 0.5f);
        else if (command == F("VKP")) settings.velocityKp = clampf(value, -1.0f, 1.0f);
        else if (command == F("QA")) settings.qAngle = clampf(value, 0.0f, 1.0f);
        else if (command == F("QG")) settings.qGyro = clampf(value, 0.0f, 1.0f);
        else if (command == F("RA")) settings.rAngle = clampf(value, 0.0f, 10.0f);
        else if (command == F("K1")) settings.k1 = clampf(value, 0.0f, 1.0f);
        else if (command == F("MINPWM")) settings.minPwm = (uint8_t)clampf(value, 0.0f, 255.0f);
        applySettingsToSubsystems();
        scheduleSettingsSave();
        return true;
    }

    if (command == F("SET")) {
        float values[12];
        for (uint8_t i = 0; i < 12; ++i) {
            if (!parseFloatToken(payload, values[i])) {
                return false;
            }
        }
        settings.trimDeg = clampf(values[0], -12.0f, 12.0f);
        settings.kp = clampf(values[1], 0.0f, 200.0f);
        settings.ki = clampf(values[2], 0.0f, 50.0f);
        settings.kd = clampf(values[3], 0.0f, 20.0f);
        settings.holdKp = clampf(values[4], -1.0f, 1.0f);
        settings.holdKi = clampf(values[5], -0.5f, 0.5f);
        settings.velocityKp = clampf(values[6], -1.0f, 1.0f);
        settings.qAngle = clampf(values[7], 0.0f, 1.0f);
        settings.qGyro = clampf(values[8], 0.0f, 1.0f);
        settings.rAngle = clampf(values[9], 0.0f, 10.0f);
        settings.k1 = clampf(values[10], 0.0f, 1.0f);
        settings.minPwm = (uint8_t)clampf(values[11], 0.0f, 255.0f);
        applySettingsToSubsystems();
        scheduleSettingsSave();
        return true;
    }

    return false;
}

void pollCommandStream(Stream &stream, String &buffer, Print &ackOut)
{
    while (stream.available()) {
        const char c = (char)stream.read();
        if (c == '\r' || c == '\n') {
            if (buffer.length() > 0) {
                const bool ok = handleCommandLine(buffer);
                acknowledge(ackOut, ok);
                if (!ok) {
                    runtime.fault = FaultCode::InvalidCommand;
                    telemetry.sendEvent(F("CMD"), buffer);
                }
            }
            buffer = "";
        } else if (buffer.length() < 160) {
            buffer += c;
        } else {
            buffer = "";
        }
    }
}

void sendEncoderTelemetry(bool toUsb, bool toRelay)
{
    long left = 0;
    long right = 0;
    encoders.snapshot(left, right);
    if (toUsb) {
        telemetry.sendEncoders(left, right,
                               left - runtime.lastUsbLeftCount,
                               right - runtime.lastUsbRightCount,
                               true, false);
        runtime.lastUsbLeftCount = left;
        runtime.lastUsbRightCount = right;
    }
    if (toRelay) {
        telemetry.sendEncoders(left, right,
                               left - runtime.lastRelayLeftCount,
                               right - runtime.lastRelayRightCount,
                               false, true);
        runtime.lastRelayLeftCount = left;
        runtime.lastRelayRightCount = right;
    }
}

void runControlLoop(uint32_t nowUs)
{
    if ((uint32_t)(nowUs - runtime.lastControlUs) < hw::kControlPeriodUs) {
        return;
    }

    const uint32_t elapsedUs = (uint32_t)(nowUs - runtime.lastControlUs);
    runtime.lastControlUs = nowUs;
    runtime.dtLastUs = elapsedUs;
    if (elapsedUs < runtime.dtMinUs) runtime.dtMinUs = elapsedUs;
    if (elapsedUs > runtime.dtMaxUs) runtime.dtMaxUs = elapsedUs;
    runtime.dtSumUs += elapsedUs;
    if (runtime.dtCount != 0xFFFFu) runtime.dtCount++;

    const float dtSeconds = clampf(elapsedUs * 1e-6f, 0.001f, 0.020f);
    if (elapsedUs > hw::kControlPeriodUs * 4UL && millis() - runtime.lastLoopJitterWarnMs >= 1000UL) {
        runtime.lastLoopJitterWarnMs = millis();
        telemetry.sendEvent(F("WARN"), F("loop_jitter"));
    }

    if (!imu.update(dtSeconds)) {
        setFault(FaultCode::ImuUpdate, F("update_failed"));
        return;
    }

    if (runtime.mode == hw::SystemMode::Fault) {
        return;
    }

    long left = 0;
    long right = 0;
    encoders.snapshot(left, right);
    const long averageCount = currentAverageCount(left, right);
    const long deltaAverageCount = averageCount - runtime.lastAverageCount;
    runtime.lastAverageCount = averageCount;

    if (runtime.mode == hw::SystemMode::Balance && controller.armed()) {
        runtime.lastOutput = controller.update(
            dtSeconds,
            imu.angleDeg(),
            imu.angleRateDegPerSec(),
            averageCount,
            deltaAverageCount);

        if (fabs(imu.angleDeg()) > hw::kTiltFaultDeg) {
            setFault(FaultCode::ExcessiveTilt, String(imu.angleDeg(), 2));
            return;
        }

        motors.apply(runtime.lastOutput.leftPwm, runtime.lastOutput.rightPwm);
    } else {
        runtime.lastOutput = ControlOutput();
        motors.stop();
    }
}

} // namespace

void setup()
{
    pinMode(hw::kButtonPin, INPUT_PULLUP);
    pinMode(hw::kBuzzerPin, OUTPUT);
    digitalWrite(hw::kBuzzerPin, LOW);

    Serial.begin(hw::kUsbBaud);
    relaySerial.begin(hw::kRelayBaud);

    telemetry.sendBoot(F("BalanceBotUltra starting"));

    motors.begin();
    encoders.begin();

    if (loadPersistedSettings(settings, runtime.settingsSeq, runtime.settingsSlot)) {
        telemetry.sendEvent(F("EEPROM"), F("loaded"));
    } else {
        telemetry.sendEvent(F("EEPROM"), F("defaults"));
    }

    runtime.mode = hw::SystemMode::Calibrating;
    imu.begin();
    applySettingsToSubsystems();
    runtime.calibratedZeroDeg = imu.calibrateBaseline(hw::kCalibrationSamples, hw::kCalibrationDelayMs);
    controller.setCalibratedZero(runtime.calibratedZeroDeg);

    runtime.mode = defaultMode();
    runtime.lastControlUs = micros();
    runtime.lastUsbRuntimeTelemetryMs = millis();
    runtime.lastUsbEncoderTelemetryMs = millis();
    runtime.lastRelayRuntimeTelemetryMs = millis();
    runtime.lastRelayEncoderTelemetryMs = millis();
    runtime.lastSettingsTelemetryMs = millis();
    runtime.lastStateTelemetryMs = millis();
    sendFullStatus();

    if (hw::kAutoArmAfterCalibration && runtime.mode == hw::SystemMode::Idle) {
        armControllerAtCurrentPosition();
    }
}

void loop()
{
    encoders.poll();
    const unsigned long nowMs = millis();
    const uint32_t nowUs = micros();

    handleButton();
    pollCommandStream(relaySerial, runtime.relayCommandBuffer, relaySerial);
    pollCommandStream(Serial, runtime.usbCommandBuffer, Serial);

    runControlLoop(nowUs);
    runBringupMode(nowMs);
    saveSettingsIfDue(nowMs);

    if (nowMs - runtime.lastStateTelemetryMs >= hw::kStateTelemetryMs) {
        runtime.lastStateTelemetryMs = nowMs;
        sendStateTelemetry();
    }

    if (nowMs - runtime.lastSettingsTelemetryMs >= hw::kSettingsTelemetryMs) {
        runtime.lastSettingsTelemetryMs = nowMs;
        sendSettingsTelemetry();
    }

    if (nowMs - runtime.lastUsbEncoderTelemetryMs >= hw::kUsbEncoderTelemetryMs) {
        runtime.lastUsbEncoderTelemetryMs = nowMs;
        sendEncoderTelemetry(true, false);
    }

    if (nowMs - runtime.lastRelayEncoderTelemetryMs >= hw::kRelayEncoderTelemetryMs) {
        runtime.lastRelayEncoderTelemetryMs = nowMs;
        sendEncoderTelemetry(false, true);
    }

    if (nowMs - runtime.lastUsbRuntimeTelemetryMs >= hw::kUsbRuntimeTelemetryMs) {
        runtime.lastUsbRuntimeTelemetryMs = nowMs;
        telemetry.sendRuntime(
            imu.angleDeg(),
            imu.angleRateDegPerSec(),
            imu.yawRateDegPerSec(),
            runtime.lastOutput,
            runtime.dtLastUs,
            runtime.dtMinUs == 0xFFFFFFFFu ? 0 : runtime.dtMinUs,
            runtime.dtMaxUs,
            dtAverageUs(),
            runtime.dtCount,
            true,
            false);
        resetDtWindow();
    }

    if (nowMs - runtime.lastRelayRuntimeTelemetryMs >= hw::kRelayRuntimeTelemetryMs) {
        runtime.lastRelayRuntimeTelemetryMs = nowMs;
        telemetry.sendRuntime(
            imu.angleDeg(),
            imu.angleRateDegPerSec(),
            imu.yawRateDegPerSec(),
            runtime.lastOutput,
            runtime.dtLastUs,
            runtime.dtMinUs == 0xFFFFFFFFu ? 0 : runtime.dtMinUs,
            runtime.dtMaxUs,
            dtAverageUs(),
            runtime.dtCount,
            false,
            true);
        resetDtWindow();
    }
}

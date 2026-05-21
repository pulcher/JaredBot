#include <Arduino.h>
#include <NeoSWSerial.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "balance_controller.h"
#include "encoder_service.h"
#include "hardware_config.h"
#include "imu_filter.h"
#include "motor_driver.h"
#include "persistence.h"
#include "settings.h"
#include "telemetry.h"

namespace {

constexpr uint8_t kCommandBufferCapacity = 160;
constexpr unsigned long kRuntimeTelemetryGapWarnMs = 2000;

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
    unsigned long lastAnyRuntimeTelemetryMs = 0;
    unsigned long lastRuntimeGapWarnMs = 0;

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
    char relayCommandBuffer[kCommandBufferCapacity + 1] = {};
    char usbCommandBuffer[kCommandBufferCapacity + 1] = {};
    uint8_t relayCommandLength = 0;
    uint8_t usbCommandLength = 0;
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

void setFault(FaultCode code, const char *detail)
{
    runtime.fault = code;
    runtime.mode = hw::SystemMode::Fault;
    controller.disarm();
    motors.stop();
    telemetry.sendFault(faultName(code), detail);
}

void setFault(FaultCode code, const __FlashStringHelper *detail)
{
    runtime.fault = code;
    runtime.mode = hw::SystemMode::Fault;
    controller.disarm();
    motors.stop();
    telemetry.sendFault(faultName(code), detail);
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
    telemetry.sendState(runtime.mode, controller.armed(), faultName(runtime.fault));
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

void trimWhitespace(char *text)
{
    char *start = text;
    while (*start && isspace((unsigned char)*start)) {
        ++start;
    }
    if (start != text) {
        memmove(text, start, strlen(start) + 1);
    }

    size_t len = strlen(text);
    while (len > 0 && isspace((unsigned char)text[len - 1])) {
        text[--len] = '\0';
    }
}

char *nextToken(char *&payload)
{
    if (*payload == '\0') {
        return payload;
    }

    char *token = payload;
    char *comma = strchr(payload, ',');
    if (comma == nullptr) {
        payload += strlen(payload);
    } else {
        *comma = '\0';
        payload = comma + 1;
    }

    trimWhitespace(token);
    return token;
}

bool parseFloatToken(char *&payload, float &out)
{
    char *token = nextToken(payload);
    if (*token == '\0') {
        return false;
    }

    char *end = nullptr;
    out = (float)strtod(token, &end);
    while (end != nullptr && *end && isspace((unsigned char)*end)) {
        ++end;
    }
    if (end == token || (end != nullptr && *end != '\0')) {
        return false;
    }
    return true;
}

bool parseIntToken(char *&payload, long &out)
{
    char *token = nextToken(payload);
    if (*token == '\0') {
        return false;
    }

    char *end = nullptr;
    out = strtol(token, &end, 10);
    while (end != nullptr && *end && isspace((unsigned char)*end)) {
        ++end;
    }
    if (end == token || (end != nullptr && *end != '\0')) {
        return false;
    }
    return true;
}

bool tokenEquals(const char *token, const char *value)
{
    while (*token && *value) {
        if (toupper((unsigned char)*token) != toupper((unsigned char)*value)) {
            return false;
        }
        ++token;
        ++value;
    }
    return *token == '\0' && *value == '\0';
}

void formatSignedPair(char *buffer, size_t bufferSize, long left, long right)
{
    if (bufferSize == 0) {
        return;
    }

    ltoa(left, buffer, 10);
    size_t len = strlen(buffer);
    if (len + 1 >= bufferSize) {
        buffer[bufferSize - 1] = '\0';
        return;
    }

    buffer[len++] = ',';
    ltoa(right, buffer + len, 10);
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

bool setModeFromString(const char *mode)
{
    if (tokenEquals(mode, "IDLE")) {
        disarmController();
        runtime.mode = hw::SystemMode::Idle;
        return true;
    }
    if (tokenEquals(mode, "BALANCE")) {
        armControllerAtCurrentPosition();
        return true;
    }
    if (tokenEquals(mode, "TEST_IMU")) {
        disarmController();
        runtime.mode = hw::SystemMode::TestImu;
        return true;
    }
    if (tokenEquals(mode, "TEST_MOTORS")) {
        disarmController();
        runtime.mode = hw::SystemMode::TestMotors;
        return true;
    }
    if (tokenEquals(mode, "TEST_ENCODERS")) {
        disarmController();
        runtime.mode = hw::SystemMode::TestEncoders;
        return true;
    }
    if (tokenEquals(mode, "TEST_SERIAL")) {
        disarmController();
        runtime.mode = hw::SystemMode::TestSerial;
        return true;
    }
    return false;
}

bool handleCommandLine(char *line)
{
    trimWhitespace(line);
    if (line[0] != 'C' || line[1] != ':') {
        return false;
    }

    char *payload = line + 2;
    trimWhitespace(payload);
    char *command = nextToken(payload);

    if (tokenEquals(command, "ARM")) {
        long value = 0;
        if (!parseIntToken(payload, value)) return false;
        if (value != 0) {
            armControllerAtCurrentPosition();
        } else {
            disarmController();
        }
        return true;
    }

    if (tokenEquals(command, "MODE")) {
        char *mode = nextToken(payload);
        return setModeFromString(mode);
    }

    if (tokenEquals(command, "STATUS")) {
        sendFullStatus();
        return true;
    }

    if (tokenEquals(command, "SAVE")) {
        runtime.settingsDirty = true;
        runtime.settingsSaveDueAtMs = millis();
        return true;
    }

    if (tokenEquals(command, "RESET")) {
        controller.reset();
        return true;
    }

    if (tokenEquals(command, "KFRESET")) {
        imu.resetFilterState();
        return true;
    }

    if (tokenEquals(command, "CLEARFAULT")) {
        clearFault();
        return true;
    }

    if (tokenEquals(command, "STOP")) {
        runtime.rawMotorOverride = false;
        runtime.rawLeftPwm = 0;
        runtime.rawRightPwm = 0;
        motors.stop();
        if (!controller.armed() && runtime.mode != hw::SystemMode::Fault) {
            runtime.mode = hw::SystemMode::Idle;
        }
        return true;
    }

    if (tokenEquals(command, "ENCRESET")) {
        encoders.reset();
        runtime.lastUsbLeftCount = 0;
        runtime.lastUsbRightCount = 0;
        runtime.lastRelayLeftCount = 0;
        runtime.lastRelayRightCount = 0;
        runtime.lastAverageCount = 0;
        return true;
    }

    if (tokenEquals(command, "RAWPWM")) {
        long left = 0;
        long right = 0;
        if (!parseIntToken(payload, left) || !parseIntToken(payload, right)) return false;
        if (controller.armed() || runtime.mode == hw::SystemMode::Fault) return false;
        runtime.rawLeftPwm = clampPwmCommand(left);
        runtime.rawRightPwm = clampPwmCommand(right);
        runtime.rawMotorOverride = (runtime.rawLeftPwm != 0 || runtime.rawRightPwm != 0);
        runtime.mode = hw::SystemMode::TestMotors;
        applyRawMotorOverride();
        char detail[16];
        formatSignedPair(detail, sizeof(detail), runtime.rawLeftPwm, runtime.rawRightPwm);
        telemetry.sendEvent(F("RAWPWM"), detail);
        return true;
    }

    if (tokenEquals(command, "TRIM")) {
        float value = 0.0f;
        if (!parseFloatToken(payload, value)) return false;
        settings.trimDeg = clampf(value, -12.0f, 12.0f);
        applySettingsToSubsystems();
        scheduleSettingsSave();
        return true;
    }

    if (tokenEquals(command, "KP") || tokenEquals(command, "KI") || tokenEquals(command, "KD") ||
        tokenEquals(command, "HKP") || tokenEquals(command, "HKI") || tokenEquals(command, "VKP") ||
        tokenEquals(command, "QA") || tokenEquals(command, "QG") || tokenEquals(command, "RA") ||
        tokenEquals(command, "K1") || tokenEquals(command, "MINPWM")) {
        float value = 0.0f;
        if (!parseFloatToken(payload, value)) return false;
        if (tokenEquals(command, "KP")) settings.kp = clampf(value, 0.0f, 200.0f);
        else if (tokenEquals(command, "KI")) settings.ki = clampf(value, 0.0f, 50.0f);
        else if (tokenEquals(command, "KD")) settings.kd = clampf(value, 0.0f, 20.0f);
        else if (tokenEquals(command, "HKP")) settings.holdKp = clampf(value, -1.0f, 1.0f);
        else if (tokenEquals(command, "HKI")) settings.holdKi = clampf(value, -0.5f, 0.5f);
        else if (tokenEquals(command, "VKP")) settings.velocityKp = clampf(value, -1.0f, 1.0f);
        else if (tokenEquals(command, "QA")) settings.qAngle = clampf(value, 0.0f, 1.0f);
        else if (tokenEquals(command, "QG")) settings.qGyro = clampf(value, 0.0f, 1.0f);
        else if (tokenEquals(command, "RA")) settings.rAngle = clampf(value, 0.0f, 10.0f);
        else if (tokenEquals(command, "K1")) settings.k1 = clampf(value, 0.0f, 1.0f);
        else if (tokenEquals(command, "MINPWM")) settings.minPwm = (uint8_t)clampf(value, 0.0f, 255.0f);
        applySettingsToSubsystems();
        scheduleSettingsSave();
        return true;
    }

    if (tokenEquals(command, "SET")) {
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

void pollCommandStream(Stream &stream, char *buffer, uint8_t &length, Print &ackOut)
{
    while (stream.available()) {
        const char c = (char)stream.read();
        if (c == '\r' || c == '\n') {
            if (length > 0) {
                buffer[length] = '\0';
                const bool ok = handleCommandLine(buffer);
                acknowledge(ackOut, ok);
                if (!ok) {
                    runtime.fault = FaultCode::InvalidCommand;
                    telemetry.sendEvent(F("CMD"), buffer);
                }
            }
            length = 0;
            buffer[0] = '\0';
        } else if (length < kCommandBufferCapacity) {
            buffer[length++] = c;
            buffer[length] = '\0';
        } else {
            length = 0;
            buffer[0] = '\0';
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
            char detail[12];
            dtostrf(imu.angleDeg(), 0, 2, detail);
            trimWhitespace(detail);
            setFault(FaultCode::ExcessiveTilt, detail);
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
    runtime.lastAnyRuntimeTelemetryMs = millis();
    runtime.lastRuntimeGapWarnMs = 0;
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
    pollCommandStream(relaySerial, runtime.relayCommandBuffer, runtime.relayCommandLength, relaySerial);
    pollCommandStream(Serial, runtime.usbCommandBuffer, runtime.usbCommandLength, Serial);

    runControlLoop(nowUs);
    runBringupMode(nowMs);
    saveSettingsIfDue(nowMs);

    if (hw::kStateTelemetryMs > 0 && nowMs - runtime.lastStateTelemetryMs >= hw::kStateTelemetryMs) {
        runtime.lastStateTelemetryMs = nowMs;
        sendStateTelemetry();
    }

    if (hw::kSettingsTelemetryMs > 0 && nowMs - runtime.lastSettingsTelemetryMs >= hw::kSettingsTelemetryMs) {
        runtime.lastSettingsTelemetryMs = nowMs;

        // Avoid long settings frames while balancing; relay serial writes can block
        // enough to trigger control-loop jitter on AVR.
        const bool balancingArmed = (runtime.mode == hw::SystemMode::Balance) && controller.armed();
        if (!balancingArmed) {
            sendSettingsTelemetry();
        }
    }

    if (hw::kUsbEncoderTelemetryMs > 0 && nowMs - runtime.lastUsbEncoderTelemetryMs >= hw::kUsbEncoderTelemetryMs) {
        runtime.lastUsbEncoderTelemetryMs = nowMs;
        sendEncoderTelemetry(true, false);
    }

    if (hw::kRelayEncoderTelemetryMs > 0 && nowMs - runtime.lastRelayEncoderTelemetryMs >= hw::kRelayEncoderTelemetryMs) {
        runtime.lastRelayEncoderTelemetryMs = nowMs;
        sendEncoderTelemetry(false, true);
    }

    if (hw::kUsbRuntimeTelemetryMs > 0 && nowMs - runtime.lastUsbRuntimeTelemetryMs >= hw::kUsbRuntimeTelemetryMs) {
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
        runtime.lastAnyRuntimeTelemetryMs = nowMs;
        resetDtWindow();
    }

    if (hw::kRelayRuntimeTelemetryMs > 0 && nowMs - runtime.lastRelayRuntimeTelemetryMs >= hw::kRelayRuntimeTelemetryMs) {
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
        runtime.lastAnyRuntimeTelemetryMs = nowMs;
        resetDtWindow();
    }

    // If runtime frames stop flowing, emit explicit warning + state so clients can
    // tell the difference between a quiet bot and a telemetry stall.
    if (nowMs - runtime.lastAnyRuntimeTelemetryMs >= kRuntimeTelemetryGapWarnMs &&
        nowMs - runtime.lastRuntimeGapWarnMs >= kRuntimeTelemetryGapWarnMs) {
        runtime.lastRuntimeGapWarnMs = nowMs;
        telemetry.sendEvent(F("WARN"), F("runtime_gap"));
        sendStateTelemetry();
    }
}

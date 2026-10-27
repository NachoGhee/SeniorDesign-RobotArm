#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <soc/gpio_struct.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "CommandParser.h"

constexpr uint8_t JOINT_COUNT = 6;
constexpr uint8_t MOTOR_COUNT = 7;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
constexpr uint32_t SENSOR_PERIOD_US = 5000;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;
constexpr uint32_t STEP_PULSE_US = 15;
constexpr float MAX_STEP_RATE = 10000.0f;
constexpr uint32_t STEPPER_TASK_STACK_BYTES = 4096;
constexpr UBaseType_t STEPPER_TASK_PRIORITY = 3;
constexpr BaseType_t STEPPER_TASK_CORE = 0;
constexpr bool BENCH_TEST_MODE = true;
// Bit N enables motor channel N for bench commands; only enabled channels need pins.
constexpr uint8_t BENCH_MOTOR_MASK = 0x7F;
// Bit N enables encoder channel N; set to zero when testing motors without encoders.
constexpr uint8_t BENCH_ENCODER_MASK = 0x3F;
constexpr uint8_t J4_INDEX = 3;
constexpr float J4_GEAR_RATIO = 100.0f;
constexpr float J4_BACKLASH_ARC_MIN = 22.0f;
constexpr float J4_BACKLASH_MOTOR_DEG = J4_BACKLASH_ARC_MIN * J4_GEAR_RATIO / 60.0f;
// The single-turn motor encoder needs the arm placed at this known output pose at startup.
constexpr float J4_STARTUP_ANGLE_DEG = 0.0f;
constexpr uint8_t TCA9548A_ADDRESS = 0x70;
constexpr uint8_t MT6701_I2C_ADDRESS = 0x06;
constexpr uint8_t MT6701_ANGLE_REGISTER = 0x03;
constexpr uint8_t MT6701_MUX_CHANNEL[JOINT_COUNT] = {0, 1, 2, 5, 3, 4};
constexpr float DM556_PULSES_PER_REV = 1600.0f;
constexpr int8_t DM556_J2_STEP_PIN = 4;
constexpr int8_t DM556_J2_DIR_PIN = 5;
constexpr int8_t DM556_J2_SECOND_STEP_PIN = 6;
constexpr int8_t DM556_J2_SECOND_DIR_PIN = 7;
constexpr int8_t DM556_J1_STEP_PIN = 15;
constexpr int8_t DM556_J1_DIR_PIN = 16;
constexpr int8_t TMC2209_A_STEP_PIN = 17;
constexpr int8_t TMC2209_A_DIR_PIN = 18;
constexpr int8_t TMC2209_X_STEP_PIN = 8;
constexpr int8_t TMC2209_X_DIR_PIN = 9;
constexpr int8_t TMC2209_Y_STEP_PIN = 10;
constexpr int8_t TMC2209_Y_DIR_PIN = 11;
constexpr int8_t TMC2209_Z_STEP_PIN = 12;
constexpr int8_t TMC2209_Z_DIR_PIN = 13;
constexpr int8_t CNC_SHIELD_ENABLE_PIN = 14;
constexpr int8_t I2C_SDA_PIN = 1;
constexpr int8_t I2C_SCL_PIN = 2;
constexpr int8_t LIMIT_SWITCH_PIN = 39;
static_assert(BENCH_MOTOR_MASK != 0 && (BENCH_MOTOR_MASK & ~0x7F) == 0,
              "Bench mode requires a valid motor channel mask");
static_assert((BENCH_ENCODER_MASK & ~0x3F) == 0,
              "Bench encoder mask contains an invalid channel");

volatile bool I2C_MODE_CONFIRMED = false;
volatile bool CALIBRATION_COMPLETE = false;
volatile bool encoderHardwareReady = false;

enum class ArmState : uint8_t {
    BOOT,
    DISARMED,
    ARMED,
    IN_MOTION,
    BENCH_MOTION,
    FAULT_ESTOP,
    FAULT_ENCODER,
    FAULT_SYSTEM
};

volatile ArmState armState = ArmState::BOOT;

const float ENCODER_ZERO_DEG[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
const int8_t ENCODER_DIRECTION_SIGN[JOINT_COUNT] = {1, 1, 1, 1, 1, 1};
const float JOINT_MIN_DEG[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
const float JOINT_MAX_DEG[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
const float MAX_JOINT_SPEED_DEG_S[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
const float POSITION_KP_PER_S[JOINT_COUNT] = {1, 1, 1, 1, 1, 1};

struct EncoderState {
    uint16_t rawAngle = 0;
    uint8_t magneticStatus = 0;
    float angleDeg = NAN;
    bool valid = false;
};

struct StepperMotor {
    int8_t stepPin;
    int8_t dirPin;
    int8_t enablePin;
    uint8_t joint;
    int8_t directionSign;
    uint8_t positiveDirectionLevel;
    uint8_t enableActiveLevel;
    float fullStepsPerRev;
    float microsteps;
    float gearRatio;
    volatile float stepsPerSecond = 0;
    uint32_t lastStepUs = 0;
    uint32_t stepRiseUs = 0;
    uint32_t directionReadyUs = 0;
    int8_t direction = 0;
    bool stepHigh = false;
    volatile int32_t absoluteSteps = 0;

    StepperMotor(int8_t step, int8_t dir, int8_t enable, uint8_t jointIndex,
                 int8_t sign, uint8_t positiveLevel, uint8_t enableLevel,
                 float fullSteps, float microstepCount, float ratio)
        : stepPin(step), dirPin(dir), enablePin(enable), joint(jointIndex),
          directionSign(sign), positiveDirectionLevel(positiveLevel),
          enableActiveLevel(enableLevel), fullStepsPerRev(fullSteps),
          microsteps(microstepCount), gearRatio(ratio) {}

    void triggerStep(uint32_t now) {
        digitalWrite(stepPin, HIGH);
        stepHigh = true;
        stepRiseUs = now;
        lastStepUs = now;
        absoluteSteps += direction;
    }

    void setStepLow() {
        digitalWrite(stepPin, LOW);
        stepHigh = false;
    }
};

EncoderState encoders[JOINT_COUNT];
bool j4TrackerInitialized = false;
float j4PreviousMotorAngleDeg = 0;
float j4BacklashRemainingMotorDeg = 0;
int8_t j4LastMotorDirection = 0;
// Motor order: DM556 channels 1-3, then TMC2209 A/X/Y/Z; channels 1 and 2 both drive J2.
StepperMotor motors[MOTOR_COUNT] = {
    {DM556_J2_STEP_PIN, DM556_J2_DIR_PIN, -1, 0, 1, HIGH, LOW, DM556_PULSES_PER_REV, 1, 0},
    {DM556_J2_SECOND_STEP_PIN, DM556_J2_SECOND_DIR_PIN, -1, 1, 1, HIGH, LOW, DM556_PULSES_PER_REV, 1, 0},
    {DM556_J1_STEP_PIN, DM556_J1_DIR_PIN, -1, 1, -1, HIGH, LOW, DM556_PULSES_PER_REV, 1, 0},
    {TMC2209_A_STEP_PIN, TMC2209_A_DIR_PIN, CNC_SHIELD_ENABLE_PIN, 2, 1, HIGH, LOW, 0, 0, 0},
    {TMC2209_X_STEP_PIN, TMC2209_X_DIR_PIN, CNC_SHIELD_ENABLE_PIN, 3, 1, HIGH, LOW, 0, 0, J4_GEAR_RATIO},
    {TMC2209_Y_STEP_PIN, TMC2209_Y_DIR_PIN, CNC_SHIELD_ENABLE_PIN, 4, 1, HIGH, LOW, 0, 0, 0},
    {TMC2209_Z_STEP_PIN, TMC2209_Z_DIR_PIN, CNC_SHIELD_ENABLE_PIN, 5, 1, HIGH, LOW, 0, 0, 0}
};
float targetDeg[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
volatile uint8_t faultMask = 0;
uint32_t lastCommandMs = 0;
uint32_t lastSensorUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t commandSequence = 0;
volatile bool stepperTaskReady = false;
volatile int8_t benchMotorIndex = -1;
volatile bool limitSwitchTriggered = false;
uint32_t benchStopAtMs = 0;
uint32_t emergencyDisableSetLow = 0;
uint32_t emergencyDisableClearLow = 0;
uint32_t emergencyDisableSetHigh = 0;
uint32_t emergencyDisableClearHigh = 0;
bool stopServiced = false;
command_parser::CommandParser commandParser;

bool motionPermitted() {
    return armState == ArmState::ARMED || armState == ArmState::IN_MOTION ||
           armState == ArmState::BENCH_MOTION;
}

bool targetActive() {
    return armState == ArmState::IN_MOTION || armState == ArmState::BENCH_MOTION;
}

void disarm() {
    if (armState != ArmState::FAULT_ESTOP && armState != ArmState::FAULT_SYSTEM) {
        armState = ArmState::DISARMED;
    }
}

bool validPin(int8_t pin) {
    return pin >= 0 && pin <= 48 && pin != 0 && pin != 3 && pin != 19 && pin != 20 &&
           pin != 35 && pin != 36 && pin != 37 && pin != 43 && pin != 44 &&
           pin != 45 && pin != 46;
}

bool motorChannelConfigured(uint8_t index) {
    return index < MOTOR_COUNT &&
           (!BENCH_TEST_MODE || (BENCH_MOTOR_MASK & (1U << index)) != 0);
}

bool encoderChannelConfigured(uint8_t index) {
    return index < JOINT_COUNT &&
           (!BENCH_TEST_MODE || (BENCH_ENCODER_MASK & (1U << index)) != 0);
}

bool muxEncoderConfigured() {
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (encoderChannelConfigured(i)) return true;
    }
    return false;
}

bool encoderPinsReady() {
    if (!validPin(I2C_SDA_PIN) || !validPin(I2C_SCL_PIN) || I2C_SDA_PIN == I2C_SCL_PIN) return false;
    if (BENCH_TEST_MODE && BENCH_ENCODER_MASK == 0) return true;
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (!encoderChannelConfigured(i)) continue;
        if (MT6701_MUX_CHANNEL[i] >= 8) return false;
    }
    return true;
}

bool motorPinsReady() {
    if ((BENCH_TEST_MODE && BENCH_MOTOR_MASK == 0) || !validPin(LIMIT_SWITCH_PIN)) return false;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (!motorChannelConfigured(i)) continue;
        if (!validPin(motors[i].stepPin) || !validPin(motors[i].dirPin) ||
            (motors[i].enablePin != -1 && !validPin(motors[i].enablePin)) ||
            motors[i].stepPin == motors[i].dirPin ||
            motors[i].stepPin == LIMIT_SWITCH_PIN || motors[i].dirPin == LIMIT_SWITCH_PIN ||
            (motors[i].enablePin >= 0 &&
             (motors[i].stepPin == motors[i].enablePin || motors[i].dirPin == motors[i].enablePin ||
                            motors[i].enablePin == LIMIT_SWITCH_PIN))) return false;
        for (uint8_t j = 0; j < i; ++j) {
            if (!motorChannelConfigured(j)) continue;
            if (motors[i].stepPin == motors[j].stepPin || motors[i].stepPin == motors[j].dirPin ||
                (motors[j].enablePin >= 0 && motors[i].stepPin == motors[j].enablePin) ||
                motors[i].dirPin == motors[j].stepPin || motors[i].dirPin == motors[j].dirPin ||
                (motors[j].enablePin >= 0 && motors[i].dirPin == motors[j].enablePin) ||
                (motors[i].enablePin >= 0 && (motors[i].enablePin == motors[j].stepPin ||
                                               motors[i].enablePin == motors[j].dirPin))) return false;
        }
    }
    return true;
}

bool benchConfigReady() {
    if (!BENCH_TEST_MODE || !motorPinsReady()) return false;
    if (BENCH_ENCODER_MASK != 0 && (!encoderPinsReady() || !I2C_MODE_CONFIRMED)) return false;
    return true;
}

bool calibrationValuesReady() {
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (JOINT_MIN_DEG[i] >= JOINT_MAX_DEG[i] || MAX_JOINT_SPEED_DEG_S[i] <= 0 ||
            (ENCODER_DIRECTION_SIGN[i] != 1 && ENCODER_DIRECTION_SIGN[i] != -1)) return false;
    }
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (motors[i].fullStepsPerRev <= 0 || motors[i].microsteps <= 0 || motors[i].gearRatio <= 0 ||
            motors[i].joint >= JOINT_COUNT ||
            (motors[i].directionSign != 1 && motors[i].directionSign != -1)) return false;
    }
    return true;
}

bool motionConfigReady() {
    return I2C_MODE_CONFIRMED && encoderHardwareReady && CALIBRATION_COMPLETE && encoderPinsReady() &&
           motorPinsReady() && calibrationValuesReady() && stepperTaskReady;
}

bool allEncodersValid() {
    if (faultMask != 0) return false;
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (!encoderChannelConfigured(i)) continue;
        if (!encoders[i].valid) return false;
    }
    return true;
}

void disableDrivers() {
    benchMotorIndex = -1;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].stepsPerSecond = 0;
        if (motorChannelConfigured(i) && validPin(motors[i].enablePin)) {
            digitalWrite(motors[i].enablePin, !motors[i].enableActiveLevel);
        }
    }
}

void enableDrivers() {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (motorChannelConfigured(i) && validPin(motors[i].enablePin) &&
            (!BENCH_TEST_MODE || i == benchMotorIndex)) {
            digitalWrite(motors[i].enablePin, motors[i].enableActiveLevel);
        }
    }
}

void IRAM_ATTR emergencyStopISR() {
    armState = ArmState::FAULT_ESTOP;
    GPIO.out_w1ts = emergencyDisableSetLow;
    GPIO.out_w1tc = emergencyDisableClearLow;
    GPIO.out1_w1ts.val = emergencyDisableSetHigh;
    GPIO.out1_w1tc.val = emergencyDisableClearHigh;
}

void IRAM_ATTR limitSwitchISR() {
    limitSwitchTriggered = true;
    emergencyStopISR();
}

void prepareEmergencyDisableMasks() {
    emergencyDisableSetLow = 0;
    emergencyDisableClearLow = 0;
    emergencyDisableSetHigh = 0;
    emergencyDisableClearHigh = 0;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (!motorChannelConfigured(i) || !validPin(motors[i].enablePin)) continue;
        const bool disableLevel = !motors[i].enableActiveLevel;
        if (motors[i].enablePin < 32) {
            const uint32_t bit = 1UL << motors[i].enablePin;
            if (disableLevel) emergencyDisableSetLow |= bit;
            else emergencyDisableClearLow |= bit;
        } else {
            const uint32_t bit = 1UL << (motors[i].enablePin - 32);
            if (disableLevel) emergencyDisableSetHigh |= bit;
            else emergencyDisableClearHigh |= bit;
        }
    }
}

bool selectMuxChannel(uint8_t channel) {
    if (channel >= 8) return false;
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(static_cast<uint8_t>(1U << channel));
    return Wire.endTransmission() == 0;
}

bool readMt6701(uint8_t index) {
    if (!encoderChannelConfigured(index) || !selectMuxChannel(MT6701_MUX_CHANNEL[index])) {
        if (index < JOINT_COUNT) {
            encoders[index].valid = false;
            encoders[index].angleDeg = NAN;
        }
        return false;
    }
    Wire.beginTransmission(MT6701_I2C_ADDRESS);
    Wire.write(MT6701_ANGLE_REGISTER);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(static_cast<uint8_t>(MT6701_I2C_ADDRESS), static_cast<uint8_t>(2)) != 2) {
        encoders[index].valid = false;
        encoders[index].angleDeg = NAN;
        return false;
    }

    const uint16_t high = Wire.read();
    const uint16_t low = Wire.read();
    encoders[index].rawAngle = ((high << 6) | (low >> 2)) & 0x3FFF;
    encoders[index].magneticStatus = 0;
    encoders[index].valid = true;
    const float rawDegrees = encoders[index].rawAngle * (360.0f / 16384.0f);
    float corrected = (rawDegrees - ENCODER_ZERO_DEG[index]) * ENCODER_DIRECTION_SIGN[index];
    while (corrected < 0) corrected += 360.0f;
    while (corrected >= 360.0f) corrected -= 360.0f;
    if (index != J4_INDEX) encoders[index].angleDeg = corrected;
    return true;
}

float signedAngleDeltaDeg(float angleDeg) {
    while (angleDeg > 180.0f) angleDeg -= 360.0f;
    while (angleDeg < -180.0f) angleDeg += 360.0f;
    return angleDeg;
}

void updateJ4JointAngle() {
    const float motorAngleDeg = encoders[J4_INDEX].rawAngle * (360.0f / 16384.0f);
    if (!j4TrackerInitialized) {
        const float startupPhaseError = signedAngleDeltaDeg(
            motorAngleDeg - ENCODER_ZERO_DEG[J4_INDEX]) * ENCODER_DIRECTION_SIGN[J4_INDEX];
        encoders[J4_INDEX].angleDeg = J4_STARTUP_ANGLE_DEG + startupPhaseError / J4_GEAR_RATIO;
        j4PreviousMotorAngleDeg = motorAngleDeg;
        j4TrackerInitialized = true;
        return;
    }

    const float motorDeltaDeg = signedAngleDeltaDeg(motorAngleDeg - j4PreviousMotorAngleDeg) *
                                ENCODER_DIRECTION_SIGN[J4_INDEX];
    j4PreviousMotorAngleDeg = motorAngleDeg;
    if (motorDeltaDeg == 0) return;

    const int8_t direction = motorDeltaDeg > 0 ? 1 : -1;
    if (direction != j4LastMotorDirection) {
        j4LastMotorDirection = direction;
        j4BacklashRemainingMotorDeg = J4_BACKLASH_MOTOR_DEG;
    }

    const float motorTravelDeg = fabsf(motorDeltaDeg);
    const float backlashTakenUpDeg = fminf(motorTravelDeg, j4BacklashRemainingMotorDeg);
    j4BacklashRemainingMotorDeg -= backlashTakenUpDeg;
    const float outputDeltaDeg = (motorTravelDeg - backlashTakenUpDeg) * direction / J4_GEAR_RATIO;
    encoders[J4_INDEX].angleDeg += outputDeltaDeg;
}

bool readEncoder(uint8_t index) {
    if (!encoderChannelConfigured(index) || !encoderPinsReady() || !encoderHardwareReady) return false;
    if (!readMt6701(index)) return false;
    if (index == J4_INDEX) updateJ4JointAngle();
    return true;
}

/**
 * @brief Refreshes every configured encoder and latches sensor faults.
 * @note A failed read disables the drivers and exits active motion.
 */
void readAllEncoders() {
    faultMask = 0;
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (!encoderChannelConfigured(i)) continue;
        if (!readEncoder(i)) faultMask |= 1U << i;
    }
    if (faultMask != 0) {
        if (armState != ArmState::FAULT_SYSTEM) armState = ArmState::FAULT_ENCODER;
        disableDrivers();
    } else if (armState == ArmState::FAULT_ENCODER) {
        armState = ArmState::DISARMED;
    }
}

void stopAndLatch(const char *reason) {
    armState = ArmState::FAULT_ESTOP;
    benchStopAtMs = 0;
    stopServiced = true;
    disableDrivers();
    Serial.print("FAULT,");
    Serial.println(reason);
}

void sendTelemetry() {
    Serial.print("TEL | ");
    Serial.print((motionConfigReady() && motionPermitted() && allEncodersValid()) ? "ARMED | " : "DISARMED | ");
    Serial.printf("FAULT:0x%02X | ", faultMask);

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        Serial.printf("JM%u:%ld ", i + 1, static_cast<long>(motors[i].absoluteSteps));
    }
    Serial.print("| ");
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (isfinite(encoders[i].angleDeg)) {
            Serial.printf("JE%u:%.2f ", i + 1, encoders[i].angleDeg);
        } else {
            Serial.printf("JE%u:NONE ", i + 1);
        }
    }
    Serial.println();
}

void testAndDisplayEncoders() {
    Serial.println("\n=== POSE ROBOT ARM ENCODER SCAN ===");

    if (!encoderPinsReady()) {
        Serial.println("[FAIL] Encoder pins are not configured correctly.");
        Serial.println("==================================\n");
        return;
    }
    if (!encoderHardwareReady) {
        Serial.println("[FAIL] Encoder hardware initialization failed; check mux wiring.");
        Serial.println("==================================\n");
        return;
    }

    uint8_t failedCount = 0;
    uint8_t passedCount = 0;
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (!encoderChannelConfigured(i)) {
            Serial.printf("[SKIP] Joint %u: encoder disabled by bench mask\n", i);
            continue;
        }
        if (readEncoder(i)) {
            ++passedCount;
            Serial.printf("[OK] Joint %u: angle=%8.3f deg, mux channel=%u\n",
                          i, encoders[i].angleDeg, MT6701_MUX_CHANNEL[i]);
        } else {
            ++failedCount;
            Serial.printf("[FAIL] Joint %u: MT6701 I2C read failed, mux channel=%u\n",
                          i, MT6701_MUX_CHANNEL[i]);
        }
    }

    Serial.println("----------------------------------");
    Serial.printf("Scan result: %u readable, %u failed\n", passedCount, failedCount);
    if (failedCount != 0) {
        Serial.println("Correct the reported issue and scan again; this diagnostic does not set a fault latch.");
    } else {
        Serial.println("Interfaces initialized. Verify J4 angle and send CFG,I2C,1 after verification.");
    }
    Serial.println("==================================\n");
}

void acceptJointTarget(const float requested[JOINT_COUNT]) {
    if (BENCH_TEST_MODE) {
        Serial.println("ERR,BENCH_ONLY");
        return;
    }
    if (armState == ArmState::FAULT_ESTOP) {
        Serial.println("ERR,ESTOP");
        return;
    }
    if (armState == ArmState::FAULT_SYSTEM) {
        Serial.println("ERR,SYSTEM_FAULT");
        return;
    }
    if (!motionConfigReady()) {
        Serial.println("ERR,NOT_CONFIGURED");
        return;
    }
    if (armState != ArmState::ARMED && armState != ArmState::IN_MOTION) {
        Serial.println("ERR,NOT_ARMED");
        return;
    }
    if (!allEncodersValid()) {
        Serial.println("ERR,ENCODER");
        return;
    }
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (requested[i] < JOINT_MIN_DEG[i] || requested[i] > JOINT_MAX_DEG[i]) {
            Serial.println("ERR,LIMIT");
            return;
        }
    }

    memcpy(targetDeg, requested, sizeof(targetDeg));
    armState = ArmState::IN_MOTION;
    lastCommandMs = millis();
    enableDrivers();
    Serial.println("ok");
}

/**
 * @brief Parses a serial line and executes its typed firmware command.
 * @param line Mutable, null-terminated command received from the host.
 * @note Hardware validation and all hardware side effects remain in this dispatcher.
 */
void processCommand(char *line) {
    float currentAngles[JOINT_COUNT];
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) currentAngles[i] = encoders[i].angleDeg;
    const command_parser::ParsedCommand command = commandParser.parse(
        line, targetDeg, currentAngles, armState == ArmState::IN_MOTION);
    if (command.error != nullptr) {
        Serial.println(command.error);
        return;
    }

    switch (command.type) {
        case command_parser::CommandType::HOST_ESTOP:
            stopAndLatch("HOST_ESTOP");
            break;
        case command_parser::CommandType::GCODE_ESTOP:
            stopAndLatch("GCODE_ESTOP");
            break;
        case command_parser::CommandType::SCAN_ENCODERS:
            if (motionPermitted()) Serial.println("ERR,SCAN_REQUIRES_DISARMED");
            else testAndDisplayEncoders();
            break;
        case command_parser::CommandType::BENCH: {
            if (!BENCH_TEST_MODE) {
                Serial.println("ERR,BENCH_DISABLED");
                break;
            }
            if (!motorChannelConfigured(static_cast<uint8_t>(command.channel))) {
                Serial.println("ERR,BENCH_CHANNEL");
                break;
            }
            if (armState == ArmState::FAULT_ESTOP) {
                Serial.println("ERR,ESTOP");
                break;
            }
            if (armState == ArmState::FAULT_SYSTEM) {
                Serial.println("ERR,SYSTEM_FAULT");
                break;
            }
            if (!benchConfigReady() || !stepperTaskReady) {
                Serial.println("ERR,BENCH_NOT_CONFIGURED");
                break;
            }
            bool encodersReady = true;
            for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
                if (encoderChannelConfigured(i) && !encoders[i].valid) encodersReady = false;
            }
            if (!encodersReady) {
                Serial.println("ERR,BENCH_ENCODER");
                break;
            }
            disableDrivers();
            benchMotorIndex = static_cast<int8_t>(command.channel);
            motors[command.channel].stepsPerSecond = static_cast<float>(command.rate);
            armState = ArmState::BENCH_MOTION;
            lastCommandMs = millis();
            benchStopAtMs = lastCommandMs + command.durationMs;
            enableDrivers();
            Serial.println("OK,BENCH_STARTED");
            break;
        }
        case command_parser::CommandType::ARM:
            if (BENCH_TEST_MODE) Serial.println("ERR,BENCH_ONLY");
            else if (armState == ArmState::FAULT_ESTOP) Serial.println("ERR,ESTOP");
            else if (armState == ArmState::FAULT_SYSTEM) Serial.println("ERR,SYSTEM_FAULT");
            else if (!motionConfigReady()) Serial.println("ERR,NOT_CONFIGURED");
            else if (!allEncodersValid()) Serial.println("ERR,ENCODER");
            else {
                armState = ArmState::ARMED;
                lastCommandMs = millis();
                enableDrivers();
                Serial.println("OK,ARMED");
            }
            break;
        case command_parser::CommandType::DISARM:
            disarm();
            benchStopAtMs = 0;
            disableDrivers();
            if (armState == ArmState::FAULT_SYSTEM) Serial.println("ERR,SYSTEM_FAULT");
            else Serial.println("OK,DISARMED");
            break;
        case command_parser::CommandType::CONFIG_I2C:
            if (motionPermitted()) {
                Serial.println("ERR,CONFIG_REQUIRES_DISARMED");
                break;
            }
            if (command.enabled && (!encoderPinsReady() || !encoderHardwareReady || !allEncodersValid())) {
                Serial.println("ERR,I2C_NOT_VERIFIED");
                break;
            }
            I2C_MODE_CONFIRMED = command.enabled;
            disarm();
            disableDrivers();
            Serial.println("OK,CONFIG");
            break;
        case command_parser::CommandType::CONFIG_CALIBRATION:
            if (motionPermitted()) {
                Serial.println("ERR,CONFIG_REQUIRES_DISARMED");
                break;
            }
            if (command.enabled && (!motorPinsReady() || !calibrationValuesReady())) {
                Serial.println("ERR,CALIBRATION_VALUES");
                break;
            }
            CALIBRATION_COMPLETE = command.enabled;
            disarm();
            disableDrivers();
            Serial.println("OK,CONFIG");
            break;
        case command_parser::CommandType::MOTION_TARGET:
            if (command.hasSequence) commandSequence = command.sequence;
            else ++commandSequence;
            acceptJointTarget(command.target);
            break;
        case command_parser::CommandType::SET_RELATIVE_MODE:
        case command_parser::CommandType::SET_ABSOLUTE_MODE:
            Serial.println("ok");
            break;
        case command_parser::CommandType::INVALID:
            break;
    }
}

void readSerial() {
    static char serialBuffer[160];
    static size_t serialLength = 0;
    while (Serial.available() > 0) {
        const char value = static_cast<char>(Serial.read());
        if (value == '\r') continue;
        if (value == '\n') {
            serialBuffer[serialLength] = '\0';
            if (serialLength > 0) {
                
                // --- NEW ECHO CODE ---
                Serial.print("\n[RECEIVED COMMAND]: ");
                Serial.println(serialBuffer);
                // ---------------------
                
                processCommand(serialBuffer);
            }
            serialLength = 0;
        } else if (serialLength + 1 < sizeof(serialBuffer)) {
            serialBuffer[serialLength++] = value;
        } else {
            serialLength = 0;
            Serial.println("ERR,LINE_TOO_LONG");
        }
    }
}

/**
 * @brief Converts joint position error into bounded motor step rates.
 * @note Runs only while configured, fault-free, and in the IN_MOTION state.
 */
void updatePositionController() {
    if (BENCH_TEST_MODE) return;
    if (!motionConfigReady() || armState != ArmState::IN_MOTION || faultMask != 0) return;
    if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
        disarm();
        disableDrivers();
        return;
    }

    float jointSpeedDegS[JOINT_COUNT];
    for (uint8_t j = 0; j < JOINT_COUNT; ++j) {
        const float error = targetDeg[j] - encoders[j].angleDeg;
        const float limit = MAX_JOINT_SPEED_DEG_S[j];
        jointSpeedDegS[j] = constrain(error * POSITION_KP_PER_S[j], -limit, limit);
        if ((encoders[j].angleDeg <= JOINT_MIN_DEG[j] && jointSpeedDegS[j] < 0) ||
            (encoders[j].angleDeg >= JOINT_MAX_DEG[j] && jointSpeedDegS[j] > 0)) jointSpeedDegS[j] = 0;
    }
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        const uint8_t joint = motors[i].joint;
        const float stepsPerDegree = motors[i].fullStepsPerRev * motors[i].microsteps *
                         motors[i].gearRatio / 360.0f;
        const float rate = jointSpeedDegS[joint] * stepsPerDegree * motors[i].directionSign;
        motors[i].stepsPerSecond = constrain(rate, -MAX_STEP_RATE, MAX_STEP_RATE);
    }
}

/**
 * @brief Generates timed step pulses and direction changes for active motors.
 * @note Called continuously by the dedicated stepper task.
 */
void updateMotorOutputs() {
    const uint32_t now = micros();
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        StepperMotor &motor = motors[i];
        if (motor.stepHigh && static_cast<uint32_t>(now - motor.stepRiseUs) >= STEP_PULSE_US) {
            motor.setStepLow();
        }
        if (!motorChannelConfigured(i) || (BENCH_TEST_MODE && i != benchMotorIndex)) continue;
        if (!targetActive() || faultMask != 0) continue;
        const float rate = motors[i].stepsPerSecond;
        if (fabsf(rate) < 1.0f || motor.stepHigh) continue;
        const int8_t direction = rate > 0 ? 1 : -1;
        if (direction != motor.direction) {
            const uint8_t positive = motor.positiveDirectionLevel;
            digitalWrite(motor.dirPin, direction > 0 ? positive : !positive);
            motor.direction = direction;
            motor.directionReadyUs = now + 5;
            continue;
        }
        if (static_cast<int32_t>(now - motor.directionReadyUs) < 0) continue;
        const uint32_t calculatedPeriod = static_cast<uint32_t>(1000000.0f / fabsf(rate));
        const uint32_t period = calculatedPeriod == 0 ? 1U : calculatedPeriod;
        if (static_cast<uint32_t>(now - motor.lastStepUs) >= period) {
            motor.triggerStep(now);
        }
    }
}

void stepperTask(void *) {
    uint32_t lastYieldMs = millis();
    for (;;) {
        updateMotorOutputs();
        
        // Check if any motor is actively commanded to move
        bool activelyMoving = false;
        for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
            if (fabsf(motors[i].stepsPerSecond) >= 1.0f) {
                activelyMoving = true;
                break;
            }
        }

        if (activelyMoving) {
            // Need tight microsecond precision for smooth stepper pulsing
            delayMicroseconds(1); 
            
            // Briefly yield to the RTOS every 2 seconds to prevent a Watchdog panic during very long movements
            if (millis() - lastYieldMs > 2000) {
                vTaskDelay(1);
                lastYieldMs = millis();
            }
        } else {
            // When motors are stopped, yield to the RTOS constantly to feed the Watchdog
            vTaskDelay(1); 
            lastYieldMs = millis();
        }
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (!motorChannelConfigured(i)) continue;
        if (validPin(motors[i].stepPin)) {
            pinMode(motors[i].stepPin, OUTPUT);
            digitalWrite(motors[i].stepPin, LOW);
        }
        if (validPin(motors[i].dirPin)) {
            pinMode(motors[i].dirPin, OUTPUT);
            digitalWrite(motors[i].dirPin, LOW);
        }
        if (validPin(motors[i].enablePin)) {
            pinMode(motors[i].enablePin, OUTPUT);
            digitalWrite(motors[i].enablePin, !motors[i].enableActiveLevel);
        }
    }
    prepareEmergencyDisableMasks();
    if (validPin(LIMIT_SWITCH_PIN)) {
        pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, FALLING);
        if (digitalRead(LIMIT_SWITCH_PIN) == LOW) limitSwitchISR();
    }
    if (encoderPinsReady() && (!BENCH_TEST_MODE || BENCH_ENCODER_MASK != 0)) {
        bool muxReady = true;
        if (muxEncoderConfigured()) {
            Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
            Wire.setClock(400000);
            Wire.beginTransmission(TCA9548A_ADDRESS);
            Wire.write(static_cast<uint8_t>(0));
            muxReady = Wire.endTransmission() == 0;
        }
        encoderHardwareReady = muxReady;
        if (!encoderHardwareReady) Serial.println("ERR,ENCODER_INIT");
    }
    disableDrivers();
    if (armState == ArmState::BOOT) armState = ArmState::DISARMED;
    stepperTaskReady = xTaskCreatePinnedToCore(stepperTask, "stepper", STEPPER_TASK_STACK_BYTES, nullptr,
                                               STEPPER_TASK_PRIORITY, nullptr, STEPPER_TASK_CORE) == pdPASS;
    if (!stepperTaskReady) {
        armState = ArmState::FAULT_SYSTEM;
        Serial.println("ERR,STEPPER_TASK_CREATE");
    }
    if (BENCH_TEST_MODE) {
        Serial.println("BENCH mode: BENCH,<channel 0-6>,<signed steps/s>,<duration ms>; DISARM stops.");
        Serial.println("Legend: BENCH_TEL | JM1-JM7 = signed commanded STEP pulse totals; JE1-JE6 = encoder angles in degrees.");
        Serial.println("JE shows NONE when an encoder is disabled or unavailable.");
    } else if (stepperTaskReady) {
        Serial.println("POSE ready; verify encoders, CFG,I2C,1, CFG,CAL,1, ARM.");
    }
}

void loop() {
    readSerial();
    if (BENCH_TEST_MODE && benchStopAtMs != 0 &&
        static_cast<int32_t>(millis() - benchStopAtMs) >= 0) {
        disarm();
        benchStopAtMs = 0;
        disableDrivers();
        Serial.println("OK,BENCH_DONE");
    }
    if (armState == ArmState::FAULT_SYSTEM && !stopServiced) {
        stopServiced = true;
        disableDrivers();
        Serial.println("FAULT,SYSTEM,STEPPER_TASK_CREATE");
    }
    if (armState == ArmState::FAULT_ESTOP && !stopServiced) {
        stopServiced = true;
        disableDrivers();
        Serial.println("FAULT,LIMIT_SWITCH");
    }
    const uint32_t now = micros();
    if ((!BENCH_TEST_MODE || BENCH_ENCODER_MASK != 0) && encoderPinsReady() && encoderHardwareReady &&
        static_cast<uint32_t>(now - lastSensorUs) >= SENSOR_PERIOD_US) {
        lastSensorUs = now;
        readAllEncoders();
    }
    updatePositionController();
    if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
        lastTelemetryMs = millis();
        sendTelemetry();
    }
}
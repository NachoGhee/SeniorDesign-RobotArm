#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <soc/gpio_struct.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

constexpr uint8_t JOINT_COUNT = 6;
constexpr uint8_t MOTOR_COUNT = 7;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
constexpr uint32_t SENSOR_PERIOD_US = 5000;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;
constexpr uint32_t STEP_PULSE_US = 3;
constexpr float MAX_STEP_RATE = 10000.0f;
constexpr uint32_t STEPPER_TASK_STACK_BYTES = 4096;
constexpr UBaseType_t STEPPER_TASK_PRIORITY = 3;
constexpr BaseType_t STEPPER_TASK_CORE = 0;
constexpr bool BENCH_TEST_MODE = true;
// Bit N enables motor channel N for bench commands; only enabled channels need pins.
constexpr uint8_t BENCH_MOTOR_MASK = 0x11;
// Bit N enables encoder channel N; use zero when testing motors without encoders.
constexpr uint8_t BENCH_ENCODER_MASK = 0x08;
constexpr long BENCH_MAX_STEPS_PER_SECOND = 200;
constexpr uint32_t BENCH_MAX_DURATION_MS = 1000;
constexpr uint8_t J4_INDEX = 3;
constexpr uint8_t TCA9548A_ADDRESS = 0x70;
constexpr uint8_t MT6701_I2C_ADDRESS = 0x06;
constexpr uint8_t MT6701_ANGLE_REGISTER = 0x03;
constexpr uint8_t NO_MUX_CHANNEL = 0xFF;
constexpr uint8_t MT6701_MUX_CHANNEL[JOINT_COUNT] = {0, 1, 2, NO_MUX_CHANNEL, 3, 4};
constexpr int8_t I2C_SDA_PIN = 1;
constexpr int8_t I2C_SCL_PIN = 2;
constexpr int8_t AMT103_A_PIN = 21;
constexpr int8_t AMT103_B_PIN = 38;
constexpr int32_t AMT103_PPR = 1024;
constexpr int32_t AMT103_QUADRATURE_FACTOR = 4;
constexpr float J4_GEAR_RATIO = 1.0f;
static_assert(BENCH_MOTOR_MASK != 0 && (BENCH_MOTOR_MASK & ~0x7F) == 0,
              "Bench mode requires a valid motor channel mask");
static_assert((BENCH_ENCODER_MASK & ~0x3F) == 0,
              "Bench encoder mask contains an invalid channel");

// Motor order: DM556 channels 1-4, then TMC2209 X/Y/Z.
const int8_t STEP_PINS[MOTOR_COUNT] = {4, 6, 15, 17, 8, 10, 12};
const int8_t DIR_PINS[MOTOR_COUNT] = {5, 7, 16, 18, 9, 11, 13};
const int8_t ENABLE_PINS[MOTOR_COUNT] = {-1, -1, -1, -1, 14, 14, 14};
const int8_t EMERGENCY_STOP_PIN = 47;

// Motor indexes 1 and 2 are the two J2 motors. The second is mechanically reversed.
const uint8_t MOTOR_JOINT[MOTOR_COUNT] = {0, 1, 1, 2, 3, 4, 5};
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {1, 1, -1, 1, 1, 1, 1};
const uint8_t MOTOR_DIR_POSITIVE_LEVEL[MOTOR_COUNT] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
const uint8_t DRIVER_ENABLE_ACTIVE_LEVEL[MOTOR_COUNT] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW};

volatile bool I2C_MODE_CONFIRMED = false;
volatile bool CALIBRATION_COMPLETE = false;
volatile bool MOTION_ENABLED = false;
volatile bool encoderHardwareReady = false;
volatile bool j4Zeroed = false;

const float ENCODER_ZERO_DEG[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
const int8_t ENCODER_DIRECTION_SIGN[JOINT_COUNT] = {1, 1, 1, 1, 1, 1};
const float JOINT_MIN_DEG[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
const float JOINT_MAX_DEG[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
const float MAX_JOINT_SPEED_DEG_S[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
const float POSITION_KP_PER_S[JOINT_COUNT] = {1, 1, 1, 1, 1, 1};
const float MOTOR_FULL_STEPS_PER_REV[MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};
const float MOTOR_MICROSTEPS[MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};
const float MOTOR_GEAR_RATIO[MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};

struct EncoderState {
    uint16_t rawAngle = 0;
    uint8_t magneticStatus = 0;
    float angleDeg = NAN;
    bool valid = false;
};

struct MotorState {
    volatile float stepsPerSecond = 0;
    uint32_t lastStepUs = 0;
    uint32_t stepRiseUs = 0;
    uint32_t directionReadyUs = 0;
    int8_t direction = 0;
    bool stepHigh = false;
    volatile int32_t absoluteSteps = 0;
};

EncoderState encoders[JOINT_COUNT];
MotorState motors[MOTOR_COUNT];
float targetDeg[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
volatile uint8_t faultMask = 0;
uint32_t lastCommandMs = 0;
uint32_t lastSensorUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t commandSequence = 0;
volatile bool haveTarget = false;
volatile bool driversEnabled = false;
volatile bool eStopLatched = false;
volatile bool stepperTaskReady = false;
volatile int8_t benchMotorIndex = -1;
uint32_t benchStopAtMs = 0;
bool gcodeRelativeMode = false;
uint32_t emergencyDisableSetLow = 0;
uint32_t emergencyDisableClearLow = 0;
uint32_t emergencyDisableSetHigh = 0;
uint32_t emergencyDisableClearHigh = 0;
bool stopServiced = false;
char serialBuffer[160];
size_t serialLength = 0;
volatile int64_t hwEncoderCount = 0;
volatile uint8_t hwEncoderState = 0;
int64_t j4ZeroCount = 0;
portMUX_TYPE amt103CountMux = portMUX_INITIALIZER_UNLOCKED;

bool validPin(int8_t pin) {
    return pin >= 0 && pin <= 48 && pin != 0 && pin != 3 && pin != 45 && pin != 46;
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
        if (i != J4_INDEX && encoderChannelConfigured(i)) return true;
    }
    return false;
}

bool encoderPinsReady() {
    if (!validPin(I2C_SDA_PIN) || !validPin(I2C_SCL_PIN) || I2C_SDA_PIN == I2C_SCL_PIN ||
        !validPin(AMT103_A_PIN) || !validPin(AMT103_B_PIN) || AMT103_A_PIN == AMT103_B_PIN) return false;
    if (BENCH_TEST_MODE && BENCH_ENCODER_MASK == 0) return true;
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (!encoderChannelConfigured(i) || i == J4_INDEX) continue;
        if (MT6701_MUX_CHANNEL[i] >= 8) return false;
    }
    return true;
}

bool motorPinsReady() {
    if (BENCH_TEST_MODE && BENCH_MOTOR_MASK == 0) return false;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (!motorChannelConfigured(i)) continue;
        if (!validPin(STEP_PINS[i]) || !validPin(DIR_PINS[i]) ||
            (ENABLE_PINS[i] != -1 && !validPin(ENABLE_PINS[i])) ||
            STEP_PINS[i] == DIR_PINS[i] ||
            (ENABLE_PINS[i] >= 0 && (STEP_PINS[i] == ENABLE_PINS[i] || DIR_PINS[i] == ENABLE_PINS[i]))) return false;
        for (uint8_t j = 0; j < i; ++j) {
            if (!motorChannelConfigured(j)) continue;
            if (STEP_PINS[i] == STEP_PINS[j] || STEP_PINS[i] == DIR_PINS[j] ||
                (ENABLE_PINS[j] >= 0 && STEP_PINS[i] == ENABLE_PINS[j]) ||
                DIR_PINS[i] == STEP_PINS[j] || DIR_PINS[i] == DIR_PINS[j] ||
                (ENABLE_PINS[j] >= 0 && DIR_PINS[i] == ENABLE_PINS[j]) ||
                (ENABLE_PINS[i] >= 0 && (ENABLE_PINS[i] == STEP_PINS[j] ||
                                          ENABLE_PINS[i] == DIR_PINS[j]))) return false;
        }
    }
    return true;
}

bool benchConfigReady() {
    if (!BENCH_TEST_MODE || !motorPinsReady() || !validPin(EMERGENCY_STOP_PIN)) return false;
    if (BENCH_ENCODER_MASK != 0 && (!encoderPinsReady() || !I2C_MODE_CONFIRMED)) return false;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (!motorChannelConfigured(i)) continue;
        if (EMERGENCY_STOP_PIN == STEP_PINS[i] || EMERGENCY_STOP_PIN == DIR_PINS[i] ||
            EMERGENCY_STOP_PIN == ENABLE_PINS[i]) return false;
    }
    return true;
}

bool calibrationValuesReady() {
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (JOINT_MIN_DEG[i] >= JOINT_MAX_DEG[i] || MAX_JOINT_SPEED_DEG_S[i] <= 0 ||
            (ENCODER_DIRECTION_SIGN[i] != 1 && ENCODER_DIRECTION_SIGN[i] != -1)) return false;
    }
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (MOTOR_FULL_STEPS_PER_REV[i] <= 0 || MOTOR_MICROSTEPS[i] <= 0 || MOTOR_GEAR_RATIO[i] <= 0 ||
            MOTOR_JOINT[i] >= JOINT_COUNT ||
            (MOTOR_DIRECTION_SIGN[i] != 1 && MOTOR_DIRECTION_SIGN[i] != -1)) return false;
    }
    return true;
}

bool motionConfigReady() {
    return I2C_MODE_CONFIRMED && encoderHardwareReady && j4Zeroed && CALIBRATION_COMPLETE && encoderPinsReady() &&
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
    driversEnabled = false;
    benchMotorIndex = -1;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].stepsPerSecond = 0;
        if (motorChannelConfigured(i) && validPin(ENABLE_PINS[i])) {
            digitalWrite(ENABLE_PINS[i], !DRIVER_ENABLE_ACTIVE_LEVEL[i]);
        }
    }
}

void enableDrivers() {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (motorChannelConfigured(i) && validPin(ENABLE_PINS[i]) &&
            (!BENCH_TEST_MODE || i == benchMotorIndex)) {
            digitalWrite(ENABLE_PINS[i], DRIVER_ENABLE_ACTIVE_LEVEL[i]);
        }
    }
    driversEnabled = true;
}

void IRAM_ATTR emergencyStopISR() {
    eStopLatched = true;
    MOTION_ENABLED = false;
    driversEnabled = false;
    GPIO.out_w1ts = emergencyDisableSetLow;
    GPIO.out_w1tc = emergencyDisableClearLow;
    GPIO.out1_w1ts.val = emergencyDisableSetHigh;
    GPIO.out1_w1tc.val = emergencyDisableClearHigh;
}

void prepareEmergencyDisableMasks() {
    emergencyDisableSetLow = 0;
    emergencyDisableClearLow = 0;
    emergencyDisableSetHigh = 0;
    emergencyDisableClearHigh = 0;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (!motorChannelConfigured(i) || !validPin(ENABLE_PINS[i])) continue;
        const bool disableLevel = !DRIVER_ENABLE_ACTIVE_LEVEL[i];
        if (ENABLE_PINS[i] < 32) {
            const uint32_t bit = 1UL << ENABLE_PINS[i];
            if (disableLevel) emergencyDisableSetLow |= bit;
            else emergencyDisableClearLow |= bit;
        } else {
            const uint32_t bit = 1UL << (ENABLE_PINS[i] - 32);
            if (disableLevel) emergencyDisableSetHigh |= bit;
            else emergencyDisableClearHigh |= bit;
        }
    }
}

void IRAM_ATTR amt103SoftwareIsr() {
    const uint8_t channelA = digitalRead(AMT103_A_PIN);
    const uint8_t channelB = digitalRead(AMT103_B_PIN);
    const uint8_t currentState = (channelA << 1) | channelB;

    portENTER_CRITICAL_ISR(&amt103CountMux);
    const uint8_t transition = (hwEncoderState << 2) | currentState;
    switch (transition) {
        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
            ++hwEncoderCount;
            break;
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            --hwEncoderCount;
            break;
    }
    hwEncoderState = currentState;
    portEXIT_CRITICAL_ISR(&amt103CountMux);
}

bool initAmt103Pcnt() {
    pinMode(AMT103_A_PIN, INPUT);
    pinMode(AMT103_B_PIN, INPUT);
    hwEncoderState = (digitalRead(AMT103_A_PIN) << 1) | digitalRead(AMT103_B_PIN);
    attachInterrupt(digitalPinToInterrupt(AMT103_A_PIN), amt103SoftwareIsr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(AMT103_B_PIN), amt103SoftwareIsr, CHANGE);
    return true;
}

bool selectMuxChannel(uint8_t channel) {
    if (channel >= 8) return false;
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(static_cast<uint8_t>(1U << channel));
    return Wire.endTransmission() == 0;
}

bool readMt6701(uint8_t index) {
    if (!encoderChannelConfigured(index) || index == J4_INDEX ||
        !selectMuxChannel(MT6701_MUX_CHANNEL[index])) {
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
    encoders[index].angleDeg = corrected;
    return true;
}

bool getAmt103Count(int64_t *count) {
    portENTER_CRITICAL(&amt103CountMux);
    *count = hwEncoderCount;
    portEXIT_CRITICAL(&amt103CountMux);
    return true;
}

bool readAmt103() {
    if (!j4Zeroed) {
        encoders[J4_INDEX].angleDeg = NAN;
        encoders[J4_INDEX].valid = false;
        return false;
    }
    int64_t count = 0;
    if (!getAmt103Count(&count)) return false;
    encoders[J4_INDEX].rawAngle = 0;
    encoders[J4_INDEX].magneticStatus = 0;
    encoders[J4_INDEX].valid = true;
    encoders[J4_INDEX].angleDeg = static_cast<float>(count - j4ZeroCount) *
                                  (360.0f / (AMT103_PPR * AMT103_QUADRATURE_FACTOR * J4_GEAR_RATIO)) *
                                  ENCODER_DIRECTION_SIGN[J4_INDEX];
    return true;
}

bool readEncoder(uint8_t index) {
    if (!encoderChannelConfigured(index) || !encoderPinsReady() || !encoderHardwareReady) return false;
    if (index == J4_INDEX) return readAmt103();
    return readMt6701(index);
}

void readAllEncoders() {
    faultMask = 0;
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (!encoderChannelConfigured(i)) continue;
        if (!readEncoder(i)) faultMask |= 1U << i;
    }
    if (faultMask != 0) {
        haveTarget = false;
        MOTION_ENABLED = false;
        disableDrivers();
    }
}

void stopAndLatch(const char *reason) {
    eStopLatched = true;
    MOTION_ENABLED = false;
    haveTarget = false;
    benchStopAtMs = 0;
    stopServiced = true;
    disableDrivers();
    Serial.print("FAULT,");
    Serial.println(reason);
}

void sendTelemetry() {
    Serial.print("TEL | ");
    Serial.print((motionConfigReady() && MOTION_ENABLED && !eStopLatched && allEncodersValid()) ? "ARMED | " : "DISARMED | ");
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
        Serial.println("[FAIL] Encoder hardware initialization failed; check mux and PCNT wiring.");
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
        if (i == J4_INDEX) {
            int64_t count = 0;
            if (getAmt103Count(&count)) {
                ++passedCount;
                Serial.printf("[OK] Joint %u (J4): software counter available, count=%lld, GPIO%u/GPIO%u\n",
                              i, static_cast<long long>(count), AMT103_A_PIN, AMT103_B_PIN);
            } else {
                ++failedCount;
                encoders[i].valid = false;
                encoders[i].angleDeg = NAN;
                Serial.printf("[FAIL] Joint %u (J4): software counter read failed\n", i);
            }
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
    } else if (!j4Zeroed) {
        Serial.println("Interfaces initialized. Rotate J4 to verify counts change; align and send ZERO_J4 before arming.");
    } else {
        Serial.println("Interfaces initialized. Rotate J4 to verify counts change; send CFG,I2C,1 after verification.");
    }
    Serial.println("==================================\n");
}

bool processSetupCommand(char *line) {
    if (strcmp(line, "SCAN_ENCODERS") == 0) {
        if (MOTION_ENABLED || driversEnabled) {
            Serial.println("ERR,SCAN_REQUIRES_DISARMED");
        } else {
            testAndDisplayEncoders();
        }
        return true;
    }
    if (strcmp(line, "ZERO_J4") == 0) {
        if (MOTION_ENABLED || driversEnabled || !encoderHardwareReady) {
            Serial.println("ERR,J4_ZERO_STATE");
            return true;
        }
        int64_t count = 0;
        if (!getAmt103Count(&count)) {
            Serial.println("ERR,J4_COUNTER");
            return true;
        }
        j4ZeroCount = count;
        j4Zeroed = true;
        readAmt103();
        Serial.println("OK,J4_ZEROED");
        return true;
    }
    if (strncmp(line, "BENCH,", 6) == 0) {
        if (!BENCH_TEST_MODE) {
            Serial.println("ERR,BENCH_DISABLED");
            return true;
        }
        char *save = nullptr;
        strtok_r(line, ",", &save);
        char *channelField = strtok_r(nullptr, ",", &save);
        char *rateField = strtok_r(nullptr, ",", &save);
        char *durationField = strtok_r(nullptr, ",", &save);
        if (channelField == nullptr || rateField == nullptr || durationField == nullptr ||
            strtok_r(nullptr, ",", &save) != nullptr) {
            Serial.println("ERR,BENCH_FORMAT");
            return true;
        }
        char *end = nullptr;
        const long channel = strtol(channelField, &end, 10);
        if (end == channelField || *end != '\0' || channel < 0 || channel >= MOTOR_COUNT ||
            !motorChannelConfigured(static_cast<uint8_t>(channel))) {
            Serial.println("ERR,BENCH_CHANNEL");
            return true;
        }
        end = nullptr;
        const long rate = strtol(rateField, &end, 10);
        if (end == rateField || *end != '\0' || rate == 0 ||
            rate < -BENCH_MAX_STEPS_PER_SECOND || rate > BENCH_MAX_STEPS_PER_SECOND) {
            Serial.println("ERR,BENCH_RATE");
            return true;
        }
        end = nullptr;
        const long duration = strtol(durationField, &end, 10);
        if (end == durationField || *end != '\0' || duration < 1 ||
            duration > BENCH_MAX_DURATION_MS) {
            Serial.println("ERR,BENCH_DURATION");
            return true;
        }
        if (eStopLatched) {
            Serial.println("ERR,ESTOP");
            return true;
        }
        if (!benchConfigReady() || !stepperTaskReady) {
            Serial.println("ERR,BENCH_NOT_CONFIGURED");
            return true;
        }
        for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
            if (encoderChannelConfigured(i) && !encoders[i].valid) {
                Serial.println("ERR,BENCH_ENCODER");
                return true;
            }
        }

        disableDrivers();
        benchMotorIndex = static_cast<int8_t>(channel);
        motors[channel].stepsPerSecond = static_cast<float>(rate);
        haveTarget = true;
        MOTION_ENABLED = true;
        lastCommandMs = millis();
        benchStopAtMs = lastCommandMs + static_cast<uint32_t>(duration);
        enableDrivers();
        Serial.println("OK,BENCH_STARTED");
        return true;
    }
    if (strcmp(line, "ARM") == 0) {
        if (BENCH_TEST_MODE) {
            Serial.println("ERR,BENCH_ONLY");
        } else if (eStopLatched) {
            Serial.println("ERR,ESTOP");
        } else if (!motionConfigReady()) {
            Serial.println("ERR,NOT_CONFIGURED");
        } else if (!allEncodersValid()) {
            Serial.println("ERR,ENCODER");
        } else {
            MOTION_ENABLED = true;
            lastCommandMs = millis();
            enableDrivers();
            Serial.println("OK,ARMED");
        }
        return true;
    }
    if (strcmp(line, "DISARM") == 0) {
        MOTION_ENABLED = false;
        haveTarget = false;
        benchStopAtMs = 0;
        disableDrivers();
        Serial.println("OK,DISARMED");
        return true;
    }
    if (strncmp(line, "CFG,", 4) != 0) return false;

    char *save = nullptr;
    strtok_r(line, ",", &save);
    char *setting = strtok_r(nullptr, ",", &save);
    char *value = strtok_r(nullptr, ",", &save);
    if (setting == nullptr || value == nullptr || strtok_r(nullptr, ",", &save) != nullptr ||
        (strcmp(value, "0") != 0 && strcmp(value, "1") != 0)) {
        Serial.println("ERR,CFG_FORMAT");
        return true;
    }
    const bool enabled = value[0] == '1';
    if (strcmp(setting, "I2C") == 0) {
        if (enabled && (!encoderPinsReady() || !encoderHardwareReady || !allEncodersValid())) {
            Serial.println("ERR,I2C_NOT_VERIFIED");
            return true;
        }
        I2C_MODE_CONFIRMED = enabled;
    } else if (strcmp(setting, "CAL") == 0) {
        if (enabled && (!motorPinsReady() || !calibrationValuesReady())) {
            Serial.println("ERR,CALIBRATION_VALUES");
            return true;
        }
        CALIBRATION_COMPLETE = enabled;
    } else {
        Serial.println("ERR,CFG_SETTING");
        return true;
    }
    MOTION_ENABLED = false;
    haveTarget = false;
    disableDrivers();
    Serial.println("OK,CONFIG");
    return true;
}

void acceptJointTarget(const float requested[JOINT_COUNT]) {
    if (BENCH_TEST_MODE) {
        Serial.println("ERR,BENCH_ONLY");
        return;
    }
    if (eStopLatched) {
        Serial.println("ERR,ESTOP");
        return;
    }
    if (!motionConfigReady()) {
        Serial.println("ERR,NOT_CONFIGURED");
        return;
    }
    if (!MOTION_ENABLED) {
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
    haveTarget = true;
    lastCommandMs = millis();
    enableDrivers();
    Serial.println("ok");
}

void processCsvCommand(char *line) {
    if (strcmp(line, "X") == 0) {
        stopAndLatch("HOST_ESTOP");
        return;
    }
    if (processSetupCommand(line)) return;
    char *save = nullptr;
    char *field = strtok_r(line, ",", &save);
    if (field == nullptr || strcmp(field, "G") != 0) {
        Serial.println("ERR,COMMAND");
        return;
    }
    field = strtok_r(nullptr, ",", &save);
    if (field == nullptr) {
        Serial.println("ERR,FORMAT");
        return;
    }
    commandSequence = strtoul(field, nullptr, 10);
    float requested[JOINT_COUNT];
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        field = strtok_r(nullptr, ",", &save);
        if (field == nullptr) {
            Serial.println("ERR,FORMAT");
            return;
        }
        requested[i] = strtof(field, nullptr);
        if (!isfinite(requested[i])) {
            Serial.println("ERR,VALUE");
            return;
        }
    }
    if (strtok_r(nullptr, ",", &save) != nullptr) {
        Serial.println("ERR,FORMAT");
        return;
    }
    acceptJointTarget(requested);
}

bool parseGcodeInteger(const char *word, char prefix, int *value) {
    if (toupper(static_cast<unsigned char>(word[0])) != prefix) return false;
    char *end = nullptr;
    const long parsed = strtol(word + 1, &end, 10);
    if (end == word + 1 || *end != '\0') return false;
    *value = static_cast<int>(parsed);
    return true;
}

void processGcode(char *line) {
    char *comment = strchr(line, ';');
    if (comment != nullptr) *comment = '\0';
    char *save = nullptr;
    char *word = strtok_r(line, " \t", &save);
    if (word == nullptr) return;

    const char type = toupper(static_cast<unsigned char>(word[0]));
    int number = 0;
    if (!parseGcodeInteger(word, type, &number)) {
        Serial.println("ERR,GCODE_FORMAT");
        return;
    }
    if (type == 'M') {
        if (strtok_r(nullptr, " \t", &save) != nullptr) {
            Serial.println("ERR,GCODE_FORMAT");
        } else if (number == 17) {
            char command[] = "ARM";
            processSetupCommand(command);
        } else if (number == 18) {
            char command[] = "DISARM";
            processSetupCommand(command);
        } else if (number == 112) {
            stopAndLatch("GCODE_ESTOP");
        } else {
            Serial.println("ERR,GCODE_UNSUPPORTED");
        }
        return;
    }
    if (type != 'G') {
        Serial.println("ERR,GCODE_UNSUPPORTED");
        return;
    }
    if (number == 90 || number == 91) {
        if (strtok_r(nullptr, " \t", &save) != nullptr) {
            Serial.println("ERR,GCODE_FORMAT");
            return;
        }
        gcodeRelativeMode = number == 91;
        Serial.println("ok");
        return;
    }
    if (number != 0 && number != 1) {
        Serial.println("ERR,GCODE_UNSUPPORTED");
        return;
    }

    static const char jointAxes[JOINT_COUNT] = {'X', 'Y', 'Z', 'A', 'B', 'C'};
    float requested[JOINT_COUNT];
    bool axisSeen[JOINT_COUNT] = {false, false, false, false, false, false};
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        requested[i] = haveTarget ? targetDeg[i] : encoders[i].angleDeg;
    }

    bool hasAxis = false;
    while ((word = strtok_r(nullptr, " \t", &save)) != nullptr) {
        const char axis = toupper(static_cast<unsigned char>(word[0]));
        uint8_t joint = JOINT_COUNT;
        for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
            if (axis == jointAxes[i]) joint = i;
        }
        if (joint == JOINT_COUNT || axisSeen[joint]) {
            Serial.println("ERR,GCODE_AXIS");
            return;
        }
        char *end = nullptr;
        const float value = strtof(word + 1, &end);
        if (end == word + 1 || *end != '\0' || !isfinite(value)) {
            Serial.println("ERR,GCODE_VALUE");
            return;
        }
        if (gcodeRelativeMode) requested[joint] += value;
        else requested[joint] = value;
        axisSeen[joint] = true;
        hasAxis = true;
    }
    if (!hasAxis) {
        Serial.println("ERR,GCODE_NO_AXES");
        return;
    }
    ++commandSequence;
    acceptJointTarget(requested);
}

void processCommand(char *line) {
    if (strcmp(line, "X") == 0) {
        stopAndLatch("HOST_ESTOP");
        return;
    }
    if (processSetupCommand(line)) return;
    if (line[0] == 'M' || line[0] == 'm' ||
        ((line[0] == 'G' || line[0] == 'g') && line[1] != ',')) {
        processGcode(line);
    } else {
        processCsvCommand(line);
    }
}

void readSerial() {
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

void updatePositionController() {
    if (BENCH_TEST_MODE) return;
    if (!motionConfigReady() || !MOTION_ENABLED || !haveTarget || !driversEnabled || eStopLatched || faultMask != 0) return;
    if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
        haveTarget = false;
        MOTION_ENABLED = false;
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
        const uint8_t joint = MOTOR_JOINT[i];
        const float stepsPerDegree = MOTOR_FULL_STEPS_PER_REV[i] * MOTOR_MICROSTEPS[i] *
                                     MOTOR_GEAR_RATIO[i] / 360.0f;
        const float rate = jointSpeedDegS[joint] * stepsPerDegree * MOTOR_DIRECTION_SIGN[i];
        motors[i].stepsPerSecond = constrain(rate, -MAX_STEP_RATE, MAX_STEP_RATE);
    }
}

void updateMotorOutputs() {
    const uint32_t now = micros();
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        MotorState &motor = motors[i];
        if (motor.stepHigh && static_cast<uint32_t>(now - motor.stepRiseUs) >= STEP_PULSE_US) {
            digitalWrite(STEP_PINS[i], LOW);
            motor.stepHigh = false;
        }
        if (!motorChannelConfigured(i) || (BENCH_TEST_MODE && i != benchMotorIndex)) continue;
        if (!MOTION_ENABLED || !haveTarget || !driversEnabled || eStopLatched || faultMask != 0) continue;
        const float rate = motors[i].stepsPerSecond;
        if (fabsf(rate) < 1.0f || motor.stepHigh) continue;
        const int8_t direction = rate > 0 ? 1 : -1;
        if (direction != motor.direction) {
            const uint8_t positive = MOTOR_DIR_POSITIVE_LEVEL[i];
            digitalWrite(DIR_PINS[i], direction > 0 ? positive : !positive);
            motor.direction = direction;
            motor.directionReadyUs = now + 5;
            continue;
        }
        if (static_cast<int32_t>(now - motor.directionReadyUs) < 0) continue;
        const uint32_t calculatedPeriod = static_cast<uint32_t>(1000000.0f / fabsf(rate));
        const uint32_t period = calculatedPeriod == 0 ? 1U : calculatedPeriod;
        if (static_cast<uint32_t>(now - motor.lastStepUs) >= period) {
            digitalWrite(STEP_PINS[i], HIGH);
            motor.stepHigh = true;
            motor.stepRiseUs = now;
            motor.lastStepUs = now;
            motor.absoluteSteps += motor.direction;
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
        if (validPin(STEP_PINS[i])) {
            pinMode(STEP_PINS[i], OUTPUT);
            digitalWrite(STEP_PINS[i], LOW);
        }
        if (validPin(DIR_PINS[i])) {
            pinMode(DIR_PINS[i], OUTPUT);
            digitalWrite(DIR_PINS[i], LOW);
        }
        if (validPin(ENABLE_PINS[i])) {
            pinMode(ENABLE_PINS[i], OUTPUT);
            digitalWrite(ENABLE_PINS[i], !DRIVER_ENABLE_ACTIVE_LEVEL[i]);
        }
    }
    prepareEmergencyDisableMasks();
    if (validPin(EMERGENCY_STOP_PIN)) {
        pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencyStopISR, FALLING);
        if (digitalRead(EMERGENCY_STOP_PIN) == LOW) emergencyStopISR();
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
        const bool pcntReady = !encoderChannelConfigured(J4_INDEX) || initAmt103Pcnt();
        encoderHardwareReady = muxReady && pcntReady;
        if (!encoderHardwareReady) Serial.println("ERR,ENCODER_INIT");
    }
    disableDrivers();
    stepperTaskReady = xTaskCreatePinnedToCore(stepperTask, "stepper", STEPPER_TASK_STACK_BYTES, nullptr,
                                               STEPPER_TASK_PRIORITY, nullptr, STEPPER_TASK_CORE) == pdPASS;
    if (!stepperTaskReady) Serial.println("ERR,STEPPER_TASK_CREATE");
    if (BENCH_TEST_MODE) {
        Serial.println("BENCH mode: BENCH,<channel 0-6>,<signed steps/s>,<duration ms>; DISARM stops.");
        Serial.println("Legend: BENCH_TEL | JM1-JM7 = signed commanded STEP pulse totals; JE1-JE6 = encoder angles in degrees.");
        Serial.println("JE shows NONE when an encoder is disabled, unavailable, or J4 has not been zeroed.");
        Serial.println("Send ZERO_J4 at the J4 marker while disarmed; J4 angle is relative to that mark.");
    } else {
        Serial.println("POSE ready; align J4, ZERO_J4, verify encoders, CFG,I2C,1, CFG,CAL,1, ARM.");
    }
}

void loop() {
    readSerial();
    if (BENCH_TEST_MODE && benchStopAtMs != 0 &&
        static_cast<int32_t>(millis() - benchStopAtMs) >= 0) {
        MOTION_ENABLED = false;
        haveTarget = false;
        benchStopAtMs = 0;
        disableDrivers();
        Serial.println("OK,BENCH_DONE");
    }
    if (eStopLatched && !stopServiced) {
        stopServiced = true;
        haveTarget = false;
        MOTION_ENABLED = false;
        disableDrivers();
        Serial.println("FAULT,HARDWARE_ESTOP");
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
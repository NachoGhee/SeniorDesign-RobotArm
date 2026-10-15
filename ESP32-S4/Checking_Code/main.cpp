#include <Arduino.h>
#include <SPI.h>
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
constexpr uint32_t SSI_CLOCK_HZ = 500000;
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
constexpr uint32_t SENSOR_PERIOD_US = 5000;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;
constexpr uint32_t STEP_PULSE_US = 3;
constexpr float MAX_STEP_RATE = 10000.0f;
constexpr uint32_t STEPPER_TASK_STACK_BYTES = 4096;
constexpr UBaseType_t STEPPER_TASK_PRIORITY = 3;
constexpr BaseType_t STEPPER_TASK_CORE = 0;

// Set these GPIOs from the final ESP32-S3 board pinout.
const int8_t SSI_CLK_PIN = -1;
const int8_t SSI_DO_PIN = -1;
const int8_t ENCODER_CSN_PINS[JOINT_COUNT] = {-1, -1, -1, -1, -1, -1};
const int8_t EMERGENCY_STOP_PIN = -1;
const int8_t STEP_PINS[MOTOR_COUNT] = {-1, -1, -1, -1, -1, -1, -1};
const int8_t DIR_PINS[MOTOR_COUNT] = {-1, -1, -1, -1, -1, -1, -1};
const int8_t ENABLE_PINS[MOTOR_COUNT] = {-1, -1, -1, -1, -1, -1, -1};

// Motor indexes 1 and 2 are the two J2 motors. The second is mechanically reversed.
const uint8_t MOTOR_JOINT[MOTOR_COUNT] = {0, 1, 1, 2, 3, 4, 5};
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {1, 1, -1, 1, 1, 1, 1};
const uint8_t MOTOR_DIR_POSITIVE_LEVEL[MOTOR_COUNT] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
const uint8_t DRIVER_ENABLE_ACTIVE_LEVEL[MOTOR_COUNT] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW};

// Tie each MT6701 MODE pin high for I2C/SSI. Confirm this physically before enabling reads.
volatile bool SSI_MODE_CONFIRMED = false;
volatile bool CALIBRATION_COMPLETE = false;
volatile bool MOTION_ENABLED = false;

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
bool gcodeRelativeMode = false;
uint32_t emergencyDisableSetLow = 0;
uint32_t emergencyDisableClearLow = 0;
uint32_t emergencyDisableSetHigh = 0;
uint32_t emergencyDisableClearHigh = 0;
bool stopServiced = false;
char serialBuffer[160];
size_t serialLength = 0;

bool validPin(int8_t pin) {
    return pin >= 0 && pin <= 48 && pin != 0 && pin != 3 && pin != 45 && pin != 46;
}

bool encoderPinsReady() {
    if (!validPin(SSI_CLK_PIN) || !validPin(SSI_DO_PIN) || SSI_CLK_PIN == SSI_DO_PIN) return false;
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (!validPin(ENCODER_CSN_PINS[i]) || ENCODER_CSN_PINS[i] == SSI_CLK_PIN ||
            ENCODER_CSN_PINS[i] == SSI_DO_PIN) return false;
        for (uint8_t j = 0; j < i; ++j) {
            if (ENCODER_CSN_PINS[i] == ENCODER_CSN_PINS[j]) return false;
        }
    }
    return true;
}

bool motorPinsReady() {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (!validPin(STEP_PINS[i]) || !validPin(DIR_PINS[i]) || !validPin(ENABLE_PINS[i]) ||
            STEP_PINS[i] == DIR_PINS[i] || STEP_PINS[i] == ENABLE_PINS[i] || DIR_PINS[i] == ENABLE_PINS[i]) return false;
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
    return SSI_MODE_CONFIRMED && CALIBRATION_COMPLETE && encoderPinsReady() &&
           motorPinsReady() && calibrationValuesReady() && stepperTaskReady;
}

bool allEncodersValid() {
    if (faultMask != 0) return false;
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        if (!encoders[i].valid) return false;
    }
    return true;
}

void disableDrivers() {
    driversEnabled = false;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].stepsPerSecond = 0;
        if (validPin(ENABLE_PINS[i])) digitalWrite(ENABLE_PINS[i], !DRIVER_ENABLE_ACTIVE_LEVEL[i]);
    }
}

void enableDrivers() {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        digitalWrite(ENABLE_PINS[i], DRIVER_ENABLE_ACTIVE_LEVEL[i]);
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
        if (!validPin(ENABLE_PINS[i])) continue;
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

uint8_t calculateCrc6(uint32_t data18) {
    uint8_t crc = 0;
    for (int8_t bit = 17; bit >= 0; --bit) {
        const bool feedback = ((crc >> 5) & 1U) ^ ((data18 >> bit) & 1U);
        crc = (crc << 1) & 0x3F;
        if (feedback) crc ^= 0x03;
    }
    return crc;
}

bool readEncoder(uint8_t index) {
    if (!encoderPinsReady()) return false;
    SPI.beginTransaction(SPISettings(SSI_CLOCK_HZ, MSBFIRST, SPI_MODE2));
    digitalWrite(ENCODER_CSN_PINS[index], LOW);
    delayMicroseconds(1);
    uint32_t frame = static_cast<uint32_t>(SPI.transfer(0)) << 16;
    frame |= static_cast<uint32_t>(SPI.transfer(0)) << 8;
    frame |= SPI.transfer(0);
    digitalWrite(ENCODER_CSN_PINS[index], HIGH);
    SPI.endTransaction();

    const uint32_t data18 = frame >> 6;
    const uint8_t receivedCrc = frame & 0x3F;
    const uint8_t status = (frame >> 6) & 0x0F;
    encoders[index].rawAngle = (frame >> 10) & 0x3FFF;
    encoders[index].magneticStatus = status;
    encoders[index].valid = receivedCrc == calculateCrc6(data18) &&
                            (status & 0x03) == 0 && (status & 0x08) == 0;
    if (!encoders[index].valid) {
        encoders[index].angleDeg = NAN;
        return false;
    }

    const float rawDegrees = encoders[index].rawAngle * (360.0f / 16384.0f);
    float corrected = (rawDegrees - ENCODER_ZERO_DEG[index]) * ENCODER_DIRECTION_SIGN[index];
    while (corrected < 0) corrected += 360.0f;
    while (corrected >= 360.0f) corrected -= 360.0f;
    encoders[index].angleDeg = corrected;
    return true;
}

void readAllEncoders() {
    faultMask = 0;
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
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
    stopServiced = true;
    disableDrivers();
    Serial.print("FAULT,");
    Serial.println(reason);
}

void sendTelemetry() {
    Serial.print("S,");
    Serial.print(commandSequence);
    Serial.print(',');
    Serial.print(faultMask);
    Serial.print(',');
    Serial.print(motionConfigReady() && MOTION_ENABLED && !eStopLatched && allEncodersValid() ? 1 : 0);
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        Serial.print(',');
        if (isfinite(encoders[i].angleDeg)) Serial.print(encoders[i].angleDeg, 3);
        else Serial.print(-1);
    }
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        Serial.print(',');
        Serial.print(targetDeg[i], 3);
    }
    Serial.println();
}

bool processSetupCommand(char *line) {
    if (strcmp(line, "ARM") == 0) {
        if (eStopLatched) {
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
    if (strcmp(setting, "SSI") == 0) {
        if (enabled && (!encoderPinsReady() || !allEncodersValid())) {
            Serial.println("ERR,SSI_NOT_VERIFIED");
            return true;
        }
        SSI_MODE_CONFIRMED = enabled;
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
            if (serialLength > 0) processCommand(serialBuffer);
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
        }
    }
}

void stepperTask(void *) {
    for (;;) {
        updateMotorOutputs();
        delayMicroseconds(1);
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
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
    if (encoderPinsReady()) {
        for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
            pinMode(ENCODER_CSN_PINS[i], OUTPUT);
            digitalWrite(ENCODER_CSN_PINS[i], HIGH);
        }
        SPI.begin(SSI_CLK_PIN, SSI_DO_PIN, -1);
    }
    disableDrivers();
    stepperTaskReady = xTaskCreatePinnedToCore(stepperTask, "stepper", STEPPER_TASK_STACK_BYTES, nullptr,
                                               STEPPER_TASK_PRIORITY, nullptr, STEPPER_TASK_CORE) == pdPASS;
    if (!stepperTaskReady) Serial.println("ERR,STEPPER_TASK_CREATE");
    Serial.println("POSE ready; send CFG,SSI,1 and CFG,CAL,1 after verification, then ARM.");
}

void loop() {
    readSerial();
    if (eStopLatched && !stopServiced) {
        stopServiced = true;
        haveTarget = false;
        MOTION_ENABLED = false;
        disableDrivers();
        Serial.println("FAULT,HARDWARE_ESTOP");
    }
    const uint32_t now = micros();
    if (encoderPinsReady() && static_cast<uint32_t>(now - lastSensorUs) >= SENSOR_PERIOD_US) {
        lastSensorUs = now;
        readAllEncoders();
    }
    updatePositionController();
    if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
        lastTelemetryMs = millis();
        sendTelemetry();
    }
}
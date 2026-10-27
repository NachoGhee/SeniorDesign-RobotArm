#include "CommandParser.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace command_parser {
namespace {

constexpr long BENCH_MAX_STEPS_PER_SECOND = 10000;
constexpr long BENCH_MAX_DURATION_MS = 5000;

ParsedCommand invalid(const char *error) {
    ParsedCommand command;
    command.error = error;
    return command;
}

ParsedCommand commandOf(CommandType type) {
    ParsedCommand command;
    command.type = type;
    return command;
}

bool parseGcodeInteger(const char *word, char prefix, int *value) {
    if (toupper(static_cast<unsigned char>(word[0])) != prefix) return false;
    char *end = nullptr;
    const long parsed = strtol(word + 1, &end, 10);
    if (end == word + 1 || *end != '\0') return false;
    *value = static_cast<int>(parsed);
    return true;
}

}

ParsedCommand CommandParser::parse(char *line, const float currentTarget[JOINT_COUNT],
                                   const float currentAngles[JOINT_COUNT], bool targetActive) {
    if (strcmp(line, "X") == 0) return commandOf(CommandType::HOST_ESTOP);
    if (strcmp(line, "SCAN_ENCODERS") == 0) return commandOf(CommandType::SCAN_ENCODERS);
    if (strcmp(line, "ARM") == 0) return commandOf(CommandType::ARM);
    if (strcmp(line, "DISARM") == 0) return commandOf(CommandType::DISARM);

    if (strncmp(line, "BENCH,", 6) == 0) {
        char *save = nullptr;
        strtok_r(line, ",", &save);
        char *channelField = strtok_r(nullptr, ",", &save);
        char *rateField = strtok_r(nullptr, ",", &save);
        char *durationField = strtok_r(nullptr, ",", &save);
        if (channelField == nullptr || rateField == nullptr || durationField == nullptr ||
            strtok_r(nullptr, ",", &save) != nullptr) return invalid("ERR,BENCH_FORMAT");

        char *end = nullptr;
        const long channel = strtol(channelField, &end, 10);
        if (end == channelField || *end != '\0' || channel < 0 || channel >= MOTOR_COUNT) {
            return invalid("ERR,BENCH_CHANNEL");
        }
        end = nullptr;
        const long rate = strtol(rateField, &end, 10);
        if (end == rateField || *end != '\0' || rate == 0 ||
            rate < -BENCH_MAX_STEPS_PER_SECOND || rate > BENCH_MAX_STEPS_PER_SECOND) {
            return invalid("ERR,BENCH_RATE");
        }
        end = nullptr;
        const long duration = strtol(durationField, &end, 10);
        if (end == durationField || *end != '\0' || duration < 1 ||
            duration > BENCH_MAX_DURATION_MS) {
            return invalid("ERR,BENCH_DURATION");
        }

        ParsedCommand command;
        command.type = CommandType::BENCH;
        command.channel = static_cast<int>(channel);
        command.rate = rate;
        command.durationMs = static_cast<uint32_t>(duration);
        return command;
    }

    if (strncmp(line, "CFG,", 4) == 0) {
        char *save = nullptr;
        strtok_r(line, ",", &save);
        char *setting = strtok_r(nullptr, ",", &save);
        char *value = strtok_r(nullptr, ",", &save);
        if (setting == nullptr || value == nullptr || strtok_r(nullptr, ",", &save) != nullptr ||
            (strcmp(value, "0") != 0 && strcmp(value, "1") != 0)) {
            return invalid("ERR,CFG_FORMAT");
        }
        ParsedCommand command;
        if (strcmp(setting, "I2C") == 0) command.type = CommandType::CONFIG_I2C;
        else if (strcmp(setting, "CAL") == 0) command.type = CommandType::CONFIG_CALIBRATION;
        else return invalid("ERR,CFG_SETTING");
        command.enabled = value[0] == '1';
        return command;
    }

    if (line[0] == 'M' || line[0] == 'm' ||
        ((line[0] == 'G' || line[0] == 'g') && line[1] != ',')) {
        char *comment = strchr(line, ';');
        if (comment != nullptr) *comment = '\0';
        char *save = nullptr;
        char *word = strtok_r(line, " \t", &save);
        if (word == nullptr) return {};
        const char type = toupper(static_cast<unsigned char>(word[0]));
        int number = 0;
        if (!parseGcodeInteger(word, type, &number)) return invalid("ERR,GCODE_FORMAT");

        if (type == 'M') {
            if (strtok_r(nullptr, " \t", &save) != nullptr) return invalid("ERR,GCODE_FORMAT");
            if (number == 17) return commandOf(CommandType::ARM);
            if (number == 18) return commandOf(CommandType::DISARM);
            if (number == 112) return commandOf(CommandType::GCODE_ESTOP);
            return invalid("ERR,GCODE_UNSUPPORTED");
        }
        if (type != 'G') return invalid("ERR,GCODE_UNSUPPORTED");
        if (number == 90 || number == 91) {
            if (strtok_r(nullptr, " \t", &save) != nullptr) return invalid("ERR,GCODE_FORMAT");
            relativeMode_ = number == 91;
            return commandOf(relativeMode_ ? CommandType::SET_RELATIVE_MODE : CommandType::SET_ABSOLUTE_MODE);
        }
        if (number != 0 && number != 1) return invalid("ERR,GCODE_UNSUPPORTED");

        static const char jointAxes[JOINT_COUNT] = {'X', 'Y', 'Z', 'A', 'B', 'C'};
        ParsedCommand command;
        command.type = CommandType::MOTION_TARGET;
        bool axisSeen[JOINT_COUNT] = {};
        for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
            command.target[i] = targetActive ? currentTarget[i] : currentAngles[i];
        }

        bool hasAxis = false;
        while ((word = strtok_r(nullptr, " \t", &save)) != nullptr) {
            const char axis = toupper(static_cast<unsigned char>(word[0]));
            uint8_t joint = JOINT_COUNT;
            for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
                if (axis == jointAxes[i]) joint = i;
            }
            if (joint == JOINT_COUNT || axisSeen[joint]) return invalid("ERR,GCODE_AXIS");
            char *end = nullptr;
            const float value = strtof(word + 1, &end);
            if (end == word + 1 || *end != '\0' || !isfinite(value)) return invalid("ERR,GCODE_VALUE");
            if (relativeMode_) command.target[joint] += value;
            else command.target[joint] = value;
            axisSeen[joint] = true;
            hasAxis = true;
        }
        if (!hasAxis) return invalid("ERR,GCODE_NO_AXES");
        return command;
    }

    char *save = nullptr;
    char *field = strtok_r(line, ",", &save);
    if (field == nullptr || strcmp(field, "G") != 0) return invalid("ERR,COMMAND");
    field = strtok_r(nullptr, ",", &save);
    if (field == nullptr) return invalid("ERR,FORMAT");
    ParsedCommand command;
    command.type = CommandType::MOTION_TARGET;
    command.hasSequence = true;
    command.sequence = strtoul(field, nullptr, 10);
    for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
        field = strtok_r(nullptr, ",", &save);
        if (field == nullptr) return invalid("ERR,FORMAT");
        command.target[i] = strtof(field, nullptr);
        if (!isfinite(command.target[i])) return invalid("ERR,VALUE");
    }
    if (strtok_r(nullptr, ",", &save) != nullptr) return invalid("ERR,FORMAT");
    return command;
}

}
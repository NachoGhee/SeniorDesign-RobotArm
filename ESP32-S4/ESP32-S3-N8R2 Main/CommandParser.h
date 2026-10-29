#pragma once

#include <stdint.h>

namespace command_parser {

constexpr uint8_t JOINT_COUNT = 6;
constexpr uint8_t MOTOR_COUNT = 7;
constexpr int BENCH_ALL_MOTORS_CHANNEL = 9;

enum class CommandType : uint8_t {
    INVALID,
    HOST_ESTOP,
    GCODE_ESTOP,
    SCAN_ENCODERS,
    BENCH,
    ARM,
    DISARM,
    CONFIG_I2C,
    CONFIG_CALIBRATION,
    MOTION_TARGET,
    SET_RELATIVE_MODE,
    SET_ABSOLUTE_MODE
};

struct ParsedCommand {
    CommandType type = CommandType::INVALID;
    const char *error = nullptr;
    uint32_t sequence = 0;
    bool hasSequence = false;
    int channel = -1;
    long rate = 0;
    uint32_t durationMs = 0;
    bool enabled = false;
    float target[JOINT_COUNT] = {};
};

class CommandParser {
public:
    /**
     * @brief Converts a mutable serial line into a typed command or protocol error.
     * @param currentTarget Last accepted target, used for partial G-code moves.
     * @param currentAngles Latest encoder positions, used when no target is active.
     * @note Relative and absolute G-code mode is retained by this parser instance.
     */
    ParsedCommand parse(char *line, const float currentTarget[JOINT_COUNT],
                        const float currentAngles[JOINT_COUNT], bool targetActive);

private:
    bool relativeMode_ = false;
};

}
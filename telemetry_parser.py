import math
import re


def parse_human_readable_telemetry(line):
    angles = [math.nan] * 6
    steps = [0] * 7
    try:
        for token in line.split():
            if token.startswith("JM") and ":" in token:
                label, value = token.split(":", 1)
                motor_idx = int(label[2:]) - 1
                if 0 <= motor_idx < 7:
                    steps[motor_idx] = int(value)
            elif token.startswith("JE") and ":" in token:
                label, value = token.split(":", 1)
                joint_idx = int(label[2:]) - 1
                if 0 <= joint_idx < 6 and value != "NONE":
                    angles[joint_idx] = float(value)
    except (IndexError, ValueError):
        pass
    return angles, steps


def parse_telemetry_line(line):
    text = line.strip()
    if "TEL |" in text or "BENCH_TEL" in text or "JE1:" in text:
        angles, steps = parse_human_readable_telemetry(text)

        faults = 0
        match = re.search(r"FAULT:0x([0-9A-Fa-f]+)", text)
        if match:
            try:
                faults = int(match.group(1), 16)
            except ValueError:
                faults = 0

        state = "ARMED" if "| ARMED |" in text.upper() else "DISARMED"
        ready = state == "ARMED" and all(math.isfinite(angle) for angle in angles)
        return {"state": state, "faults": faults, "ready": ready, "angles": angles, "steps": steps}

    fields = text.split(",")
    if len(fields) == 16 and fields[0] == "S":
        try:
            return {
                "state": "ARMED" if fields[3] == "1" else "DISARMED",
                "faults": int(fields[2]),
                "ready": fields[3] == "1",
                "angles": [float(value) for value in fields[4:10]],
                "steps": [0] * 7,
            }
        except ValueError:
            return None

    return None
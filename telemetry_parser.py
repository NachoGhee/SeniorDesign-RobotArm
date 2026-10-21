import math
import re


def parse_human_readable_telemetry(line):
    angles = [math.nan] * 6
    if "JE1:" not in line:
        return angles
    try:
        encoder_section = line.split("|")[-1].strip()
        for token in encoder_section.split():
            if token.startswith("JE") and ":" in token:
                label, value = token.split(":", 1)
                joint_idx = int(label[2:]) - 1
                if 0 <= joint_idx < 6 and value != "NONE":
                    angles[joint_idx] = float(value)
    except (IndexError, ValueError):
        return [math.nan] * 6
    return angles


def parse_telemetry_line(line):
    text = line.strip()
    if "TEL |" in text or "BENCH_TEL" in text or "JE1:" in text:
        angles = parse_human_readable_telemetry(text)
        if all(math.isnan(angle) for angle in angles):
            return None

        faults = 0
        match = re.search(r"FAULT:0x([0-9A-Fa-f]+)", text)
        if match:
            try:
                faults = int(match.group(1), 16)
            except ValueError:
                faults = 0

        ready = "ARMED" in text.upper() or "BENCH_TEL" in text
        return {"faults": faults, "ready": ready, "angles": angles}

    fields = text.split(",")
    if len(fields) == 16 and fields[0] == "S":
        try:
            return {
                "faults": int(fields[2]),
                "ready": fields[3] == "1",
                "angles": [float(value) for value in fields[4:10]],
            }
        except ValueError:
            return None

    return None
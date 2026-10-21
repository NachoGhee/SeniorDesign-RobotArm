JOINT_GCODE_AXES = ("X", "Y", "Z", "A", "B", "C")


def format_joint_gcode(angles):
    words = " ".join(f"{axis}{angle:.3f}" for axis, angle in zip(JOINT_GCODE_AXES, angles))
    return f"G1 {words}"
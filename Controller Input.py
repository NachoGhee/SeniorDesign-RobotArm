import argparse
import os
import re
import sys
import time

import pygame
import numpy as np
import serial
import ikgeo_model

SERIAL_PORT = os.environ.get("POSE_SERIAL_PORT", "COM10" if os.name == "nt" else "/dev/ttyACM0")
BAUD_RATE = 115200
DEADZONE = 0.15
SEND_PERIOD_S = 0.05
# Edit these mappings/speeds to match the Pi's gamepad. Translation is in meters/s.
CARTESIAN_AXIS_TO_XYZ = {0: 0, 1: 1, 4: 2}
CARTESIAN_AXIS_SIGN = {0: 1, 1: -1, 4: 1}
CARTESIAN_SPEED_M_S = [0.10, 0.10, 0.10]
ESTOP_BUTTON = 1
JOINT_GCODE_AXES = ("X", "Y", "Z", "A", "B", "C")


def format_joint_gcode(angles):
    words = " ".join(f"{axis}{angle:.3f}" for axis, angle in zip(JOINT_GCODE_AXES, angles))
    return f"G1 {words}"


def apply_deadzone(value):
    if abs(value) < DEADZONE:
        return 0.0
    return (abs(value) - DEADZONE) / (1.0 - DEADZONE) * (1 if value > 0 else -1)


def parse_human_readable_telemetry(line):
    angles = np.full(6, np.nan)
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
        return np.full(6, np.nan)
    return angles


def parse_telemetry_line(line):
    text = line.strip()
    if "TEL |" in text or "BENCH_TEL" in text or "JE1:" in text:
        angles = parse_human_readable_telemetry(text)
        if np.all(np.isnan(angles)):
            return None

        faults = 0
        match = re.search(r"FAULT:0x([0-9A-Fa-f]+)", text)
        if match:
            try:
                faults = int(match.group(1), 16)
            except ValueError:
                faults = 0

        ready = "ARMED" in text.upper() or "BENCH_TEL" in text
        return {
            "faults": faults,
            "ready": ready,
            "angles": angles.tolist(),
        }

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


def read_telemetry(port, receive_buffer):
    latest = None
    receive_buffer.extend(port.read(port.in_waiting))
    while b"\n" in receive_buffer:
        raw_line, _, remainder = receive_buffer.partition(b"\n")
        receive_buffer[:] = remainder
        line = raw_line.decode("ascii", errors="replace").strip()
        telemetry = parse_telemetry_line(line)
        if telemetry is not None:
            latest = telemetry
        elif line.startswith(("ERR,", "FAULT,")):
            print(line)
    return latest


def send_gcode_line(port, command, receive_buffer, timeout_s=2.0):
    port.write((command + "\n").encode("ascii"))
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        receive_buffer.extend(port.read(port.in_waiting))
        while b"\n" in receive_buffer:
            raw_line, _, remainder = receive_buffer.partition(b"\n")
            receive_buffer[:] = remainder
            line = raw_line.decode("ascii", errors="replace").strip()
            telemetry = parse_telemetry_line(line)
            if telemetry is not None:
                if telemetry["faults"]:
                    raise RuntimeError(f"Encoder fault mask: 0x{telemetry['faults']:02X}")
            elif line == "ok" or line.startswith("OK,"):
                return
            elif line.startswith(("ERR,", "FAULT,")):
                raise RuntimeError(line)
        time.sleep(0.005)
    raise TimeoutError(f"Timed out waiting for acknowledgement to: {command}")


def run_gcode_stream(port, source):
    receive_buffer = bytearray()
    for raw_line in source:
        command = raw_line.split(";", 1)[0].strip()
        if not command:
            continue
        send_gcode_line(port, command, receive_buffer)
        print(f"ok: {command}")
        if command.upper() == "M112":
            return
    send_gcode_line(port, "M18", receive_buffer)

def main(argv=None):
    parser = argparse.ArgumentParser(description="POSE Raspberry Pi serial and joystick controller")
    parser.add_argument("--port", default=SERIAL_PORT, help="ESP32 serial port; defaults to POSE_SERIAL_PORT or /dev/ttyACM0")
    parser.add_argument("--gcode-file", help="send a G-code file, or '-' to read G-code from stdin")
    args = parser.parse_args(argv)

    print(f"Connecting to ESP32 on {args.port} at {BAUD_RATE} baud...")
    try:
        port = serial.Serial(args.port, BAUD_RATE, timeout=0)
    except serial.SerialException as error:
        print(f"Could not open serial port: {error}")
        return 1

    if args.gcode_file is not None:
        source = sys.stdin if args.gcode_file == "-" else None
        try:
            if source is None:
                with open(args.gcode_file, "r", encoding="ascii") as source_file:
                    run_gcode_stream(port, source_file)
            else:
                run_gcode_stream(port, source)
        except (OSError, RuntimeError, TimeoutError) as error:
            print(f"G-code stopped: {error}")
            port.close()
            return 1
        port.close()
        return 0

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller found.")
        pygame.quit()
        port.close()
        return 1

    screen = pygame.display.set_mode((500, 250))
    pygame.display.set_caption("POSE Arm Kinematics HUD")
    font = pygame.font.SysFont("consolas", 24)

    controller = pygame.joystick.Joystick(0)
    print(f"Controller: {controller.get_name()}")
    model_errors = ikgeo_model.model_configuration_errors()
    if model_errors:
        print("Cartesian IK disabled until ikgeo_model.py is configured:")
        for error in model_errors:
            print(f"  - {error}")
    print("Keys: S=confirm SSI, C=confirm calibration, A=arm, D=disarm; controller button 1=E-stop.")

    target_pose = None
    seed_angles_rad = None
    ik_ready = not model_errors
    last_send = 0.0
    last_telemetry_time = 0.0
    controller_ready = False
    last_not_ready_message = False
    last_ik_error_time = 0.0
    receive_buffer = bytearray()
    clock = pygame.time.Clock()
    running = True

    try:
        while running:
            dt = min(clock.tick(50) / 1000.0, 0.1)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.JOYBUTTONDOWN and event.button == ESTOP_BUTTON:
                    port.write(b"X\n")
                    print("E-stop sent; controller must be power-cycled to clear it.")
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                    port.write(b"CFG,SSI,1\n")
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                    port.write(b"CFG,CAL,1\n")
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_a:
                    if ik_ready:
                        port.write(b"G90\nM17\n")
                    else:
                        print("Arm blocked: configure the POE model and IK-Geo solver first.")
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_d:
                    port.write(b"DISARM\n")
                    target_pose = None
                    seed_angles_rad = None
                    controller_ready = False

            now = time.monotonic()
            telemetry = read_telemetry(port, receive_buffer)
            if telemetry is not None:
                last_telemetry_time = now
                if len(telemetry["angles"]) > 3 and not np.isnan(telemetry["angles"][3]):
                    print(f"--> [Bench Debug] Live J4 Angle: {telemetry['angles'][3]:.2f}°")
                if telemetry["faults"]:
                    print(f"Encoder fault mask: 0x{telemetry['faults']:02X}")
                    target_pose = None
                    seed_angles_rad = None
                    controller_ready = False
                elif telemetry["ready"] and all(angle >= 0 for angle in telemetry["angles"]):
                    controller_ready = True
                    last_not_ready_message = False
                    if target_pose is None and ik_ready:
                        try:
                            seed_angles_rad = ikgeo_model.encoder_degrees_to_model_radians(
                                np.asarray(telemetry["angles"], dtype=float)
                            )
                            target_pose = ikgeo_model.forward_camera_pose(seed_angles_rad)
                            print("Cartesian target initialized from measured camera pose.")
                        except (ValueError, ikgeo_model.RobotModelNotConfigured) as error:
                            ik_ready = False
                            print(f"Cartesian IK disabled: {error}")
                else:
                    controller_ready = False
                    target_pose = None
                    seed_angles_rad = None

            if now - last_telemetry_time > 0.5:
                controller_ready = False
                target_pose = None
                seed_angles_rad = None

            if target_pose is not None and seed_angles_rad is not None and controller_ready and ik_ready:
                for axis, coordinate in CARTESIAN_AXIS_TO_XYZ.items():
                    if axis >= controller.get_numaxes():
                        continue
                    value = apply_deadzone(controller.get_axis(axis))
                    sign = CARTESIAN_AXIS_SIGN.get(axis, 1)
                    target_pose[coordinate, 3] += value * sign * CARTESIAN_SPEED_M_S[coordinate] * dt
                    workspace = np.asarray(ikgeo_model.CARTESIAN_WORKSPACE_BOUNDS_M)
                    target_pose[coordinate, 3] = np.clip(
                        target_pose[coordinate, 3], workspace[coordinate, 0], workspace[coordinate, 1]
                    )

                if now - last_send >= SEND_PERIOD_S:
                    try:
                        solved_angles_rad = ikgeo_model.solve_camera_pose(target_pose, seed_angles_rad)
                        joint_targets_deg = ikgeo_model.model_radians_to_encoder_degrees(solved_angles_rad)
                        port.write((format_joint_gcode(joint_targets_deg) + "\n").encode("ascii"))
                        seed_angles_rad = solved_angles_rad
                    except (ValueError, ikgeo_model.RobotModelNotConfigured) as error:
                        if now - last_ik_error_time >= 1.0:
                            print(f"IK target rejected: {error}")
                            last_ik_error_time = now
                    last_send = now
            elif telemetry is not None and not telemetry["ready"]:
                if not last_not_ready_message:
                    print("Firmware is not armed: configure SSI pins and calibration before motion.")
                    last_not_ready_message = True

            screen.fill((30, 30, 30))

            status_text = "ARMED (LIVE)" if (controller_ready and ik_ready) else "DISARMED / FAULT"
            status_color = (50, 255, 50) if status_text == "ARMED (LIVE)" else (255, 50, 50)
            screen.blit(font.render(f"Status: {status_text}", True, status_color), (20, 20))

            if target_pose is not None:
                x_mm = target_pose[0, 3] * 1000
                y_mm = target_pose[1, 3] * 1000
                z_mm = target_pose[2, 3] * 1000

                screen.blit(font.render(f"Target X: {x_mm:>8.2f} mm", True, (255, 255, 255)), (20, 80))
                screen.blit(font.render(f"Target Y: {y_mm:>8.2f} mm", True, (255, 255, 255)), (20, 115))
                screen.blit(font.render(f"Target Z: {z_mm:>8.2f} mm", True, (255, 255, 255)), (20, 150))
            else:
                screen.blit(font.render("Target XYZ: Waiting for Telemetry...", True, (150, 150, 150)), (20, 80))

            pygame.display.flip()

    except KeyboardInterrupt:
        pass
    finally:
        port.close()
        pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())

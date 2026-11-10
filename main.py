import argparse
import os
import sys
import time

import numpy as np
import pygame
import serial

from gcode_formatter import format_joint_gcode
import ikgeo_model
from gamepad_input import GamepadController
from serial_link import SerialConnection


SERIAL_PORT = os.environ.get("POSE_SERIAL_PORT", "COM10" if os.name == "nt" else "/dev/ttyACM0")
BAUD_RATE = 115200
SEND_PERIOD_S = 0.05
J4_CALIBRATION_SEND_PERIOD_S = 0.10


def main(argv=None):
    parser = argparse.ArgumentParser(description="POSE Raspberry Pi serial and joystick controller")
    parser.add_argument("--port", default=SERIAL_PORT, help="ESP32 serial port; defaults to POSE_SERIAL_PORT or /dev/ttyACM0")
    parser.add_argument("--gcode-file", help="send a G-code file, or '-' to read G-code from stdin")
    args = parser.parse_args(argv)

    print(f"Connecting to ESP32 on {args.port} at {BAUD_RATE} baud...")
    try:
        connection = SerialConnection(args.port, BAUD_RATE)
    except serial.SerialException as error:
        print(f"Could not open serial port: {error}")
        return 1

    if args.gcode_file is not None:
        source = sys.stdin if args.gcode_file == "-" else None
        try:
            if source is None:
                with open(args.gcode_file, "r", encoding="ascii") as source_file:
                    connection.run_gcode_stream(source_file)
            else:
                connection.run_gcode_stream(source)
        except (OSError, RuntimeError, TimeoutError) as error:
            print(f"G-code stopped: {error}")
            connection.close()
            return 1
        connection.close()
        return 0

    try:
        controller = GamepadController()
    except RuntimeError as error:
        print(error)
        connection.close()
        return 1

    screen = pygame.display.set_mode((500, 250))
    pygame.display.set_caption("POSE Arm Kinematics HUD")
    font = pygame.font.SysFont("consolas", 24)

    print(f"Controller: {controller.get_name()}")
    model_errors = ikgeo_model.model_configuration_errors()
    if model_errors:
        print("Cartesian IK disabled until ikgeo_model.py is configured:")
        for error in model_errors:
            print(f"  - {error}")
    print("Keys: S=confirm I2C, C=confirm calibration, A=arm, D=disarm; controller button 1=E-stop.")

    target_pose = None
    seed_angles_rad = None
    ik_ready = not model_errors
    last_send = 0.0
    last_telemetry_time = 0.0
    controller_ready = False
    j4_calibration_mode = False
    startup_calibration_complete = False
    last_j4_jog_rate = 0
    last_j4_jog_send = 0.0
    last_not_ready_message = False
    last_ik_error_time = 0.0
    clock = pygame.time.Clock()
    running = True

    try:
        while running:
            dt = min(clock.tick(50) / 1000.0, 0.1)
            gamepad_update = controller.poll()
            if gamepad_update.quit_requested:
                running = False
            for action in gamepad_update.actions:
                if action == "estop":
                    connection.send_command("X")
                    print("E-stop sent; controller must be power-cycled to clear it.")
                    running = False
                elif action == "disarm":
                    connection.send_command("DISARM")
                    j4_calibration_mode = False
                    last_j4_jog_rate = 0
                    target_pose = None
                    seed_angles_rad = None
                    controller_ready = False

            if gamepad_update.j4_calibration_toggle:
                if j4_calibration_mode:
                    connection.send_command("DISARM")
                    j4_calibration_mode = False
                    last_j4_jog_rate = 0
                    target_pose = None
                    seed_angles_rad = None
                    controller_ready = False
                    print("J4 calibration mode stopped.")
                else:
                    connection.send_command("DISARM")
                    j4_calibration_mode = True
                    last_j4_jog_rate = 0
                    target_pose = None
                    seed_angles_rad = None
                    controller_ready = False
                    print("J4 calibration mode: align the witness marks, then press START or A.")

            now = time.monotonic()
            telemetry = connection.read_telemetry()
            if telemetry is not None:
                last_telemetry_time = now
                if len(telemetry["angles"]) > 3 and not np.isnan(telemetry["angles"][3]):
                    print(f"--> [Bench Debug] Live J4 Angle: {telemetry['angles'][3]:.2f}°")
                if telemetry["faults"]:
                    print(f"Encoder fault mask: 0x{telemetry['faults']:02X}")
                    target_pose = None
                    seed_angles_rad = None
                    controller_ready = False
                elif telemetry["ready"] and len(telemetry["angles"]) == 6 and np.all(
                    np.isfinite(telemetry["angles"])
                ):
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

            if j4_calibration_mode:
                jog_rate = gamepad_update.j4_jog_rate
                if gamepad_update.j4_calibration_confirm and jog_rate == 0:
                    connection.send_command("DISARM")
                    connection.send_command("ZERO4")
                    connection.send_command("CFG,I2C,1")
                    connection.send_command("CFG,CAL,1")
                    connection.send_command("ARM")
                    j4_calibration_mode = False
                    startup_calibration_complete = True
                    last_j4_jog_rate = 0
                    target_pose = None
                    seed_angles_rad = None
                    controller_ready = False
                    print("J4 zero latched; arming closed-loop control.")
                elif jog_rate != 0:
                    if now - last_j4_jog_send >= J4_CALIBRATION_SEND_PERIOD_S:
                        connection.send_command(f"JOG4,{int(jog_rate)},150")
                        last_j4_jog_send = now
                    last_j4_jog_rate = jog_rate
                elif last_j4_jog_rate != 0:
                    connection.send_command("JOG4,0,10")
                    last_j4_jog_rate = 0

                screen.fill((30, 30, 30))
                screen.blit(font.render("J4 CALIBRATION", True, (255, 220, 90)), (20, 20))
                screen.blit(font.render(f"Jog rate: {jog_rate:+d} steps/s", True, (255, 255, 255)), (20, 80))
                screen.blit(font.render("Align marks, then press START or A", True, (190, 190, 190)), (20, 125))
                pygame.display.flip()
                continue

            for action in gamepad_update.actions:
                if action == "confirm_i2c":
                    if not controller_ready:
                        connection.send_command("CFG,I2C,1")
                elif action == "confirm_calibration":
                    if not controller_ready:
                        connection.send_command("CFG,CAL,1")
                elif action == "arm":
                    if not startup_calibration_complete:
                        print("Arm blocked: complete J4 startup calibration first.")
                    elif ik_ready:
                        connection.send_command("G90")
                        connection.send_command("M17")
                    else:
                        print("Arm blocked: configure the POE model and IK-Geo solver first.")

            if target_pose is not None and seed_angles_rad is not None and controller_ready and ik_ready:
                for coordinate, velocity in enumerate(gamepad_update.desired_velocity):
                    target_pose[coordinate, 3] += velocity * dt
                    workspace = np.asarray(ikgeo_model.CARTESIAN_WORKSPACE_BOUNDS_M)
                    target_pose[coordinate, 3] = np.clip(
                        target_pose[coordinate, 3], workspace[coordinate, 0], workspace[coordinate, 1]
                    )

                if now - last_send >= SEND_PERIOD_S:
                    try:
                        solved_angles_rad = ikgeo_model.solve_camera_pose(target_pose, seed_angles_rad)
                        joint_targets_deg = ikgeo_model.model_radians_to_encoder_degrees(solved_angles_rad)
                        connection.send_command(format_joint_gcode(joint_targets_deg))
                        seed_angles_rad = solved_angles_rad
                    except (ValueError, ikgeo_model.RobotModelNotConfigured) as error:
                        if now - last_ik_error_time >= 1.0:
                            print(f"IK target rejected: {error}")
                            last_ik_error_time = now
                    last_send = now
            elif telemetry is not None and not telemetry["ready"]:
                if not last_not_ready_message:
                    print("Firmware is not ready: verify encoders and confirm I2C and calibration.")
                    last_not_ready_message = True

            screen.fill((30, 30, 30))
            status_text = "ARMED (LIVE)" if (controller_ready and ik_ready) else "DISARMED / FAULT"
            status_color = (50, 255, 50) if status_text == "ARMED (LIVE)" else (255, 50, 50)
            screen.blit(font.render(f"Status: {status_text}", True, status_color), (20, 20))

            if target_pose is not None:
                x_mm, y_mm, z_mm = target_pose[:3, 3] * 1000
                screen.blit(font.render(f"Target X: {x_mm:>8.2f} mm", True, (255, 255, 255)), (20, 80))
                screen.blit(font.render(f"Target Y: {y_mm:>8.2f} mm", True, (255, 255, 255)), (20, 115))
                screen.blit(font.render(f"Target Z: {z_mm:>8.2f} mm", True, (255, 255, 255)), (20, 150))
            else:
                screen.blit(font.render("Target XYZ: Waiting for Telemetry...", True, (150, 150, 150)), (20, 80))

            pygame.display.flip()

    except KeyboardInterrupt:
        pass
    finally:
        connection.close()
        controller.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
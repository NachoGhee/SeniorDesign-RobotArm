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
HUD_REFRESH_PERIOD_S = 0.10
REPLAY_START_TOLERANCE_DEG = 2.0
REPLAY_HOMING_TIMEOUT_S = 15.0
NUM_RECORD_SLOTS = 5
MOTOR_LABELS = ("M1:J1", "M2:LJ2", "M3:RJ2", "M4:J3", "M5:J4", "M6:J5", "M7:J6")
JOINT_LABELS = ("J1(X)", "J2(Y)", "J3(Z)", "J4(A)", "J5(B)", "J6(C)")


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

    screen = pygame.display.set_mode((840, 520))
    pygame.display.set_caption("POSE Arm Kinematics & Telemetry HUD")
    font = pygame.font.SysFont("consolas", 15)

    print(f"Controller: {controller.get_name()}")
    model_errors = ikgeo_model.model_configuration_errors()
    if model_errors:
        print("Cartesian IK disabled until ikgeo_model.py is configured:")
        for error in model_errors:
            print(f"  - {error}")
    print("Keys: S=confirm I2C, C=confirm calibration, A=arm, D=disarm, R=record, P=replay; START zeroes J4, L/R select slots, A locks/unlocks selected slot.")

    target_pose = None
    seed_angles_rad = None
    ik_ready = not model_errors
    last_send = 0.0
    last_telemetry_time = 0.0
    controller_ready = False
    j4_calibration_mode = False
    startup_calibration_complete = False
    latest_telemetry = None
    prev_steps = [0] * 7
    motor_step_deltas = [0] * 7
    joint_targets_deg = [0.0] * 6
    recording_active = False
    record_start_time = 0.0
    recorded_slots = [[] for _ in range(NUM_RECORD_SLOTS)]
    selected_slot = 0
    slot_confirmed = False
    replay_state = "IDLE"
    replay_start_time = 0.0
    replay_frame_index = 0
    last_j4_jog_rate = 0
    last_j4_jog_send = 0.0
    last_hud_draw = 0.0
    last_controller_button = "None"
    last_controller_button_time = 0.0
    last_not_ready_message = False
    last_ik_error_time = 0.0
    bench_test_mode = False
    last_bench_send = 0.0
    last_bench_rates = (0, 0, 0, 0, 0, 0, 0)
    clock = pygame.time.Clock()
    running = True

    def reset_motion_state(clear_calibration=False):
        nonlocal recording_active, replay_state, target_pose, seed_angles_rad, controller_ready
        nonlocal j4_calibration_mode, last_j4_jog_rate
        recording_active = False
        replay_state = "IDLE"
        target_pose = None
        seed_angles_rad = None
        controller_ready = False
        if clear_calibration:
            j4_calibration_mode = False
            last_j4_jog_rate = 0

    def send_bench_rates(rates):
        if not rates:
            rates = (0, 0, 0, 0, 0, 0, 0)
        payload = ",".join(str(int(round(rate))) for rate in rates)
        connection.send_command(f"BENCH_MULTI,{payload},200")

    try:
        while running:
            dt = min(clock.tick(50) / 1000.0, 0.1)
            now = time.monotonic()
            gamepad_update = controller.poll()
            if gamepad_update.quit_requested:
                running = False
            for action in gamepad_update.actions:
                if action == "estop":
                    connection.send_command("X")
                    reset_motion_state(clear_calibration=True)
                    print("E-stop sent; controller must be power-cycled to clear it.")
                    running = False
                elif action == "toggle_bench_mode":
                    bench_test_mode = not bench_test_mode
                    reset_motion_state(clear_calibration=True)
                    connection.send_command("BENCH_MULTI,0,0,0,0,0,0,0,10")
                    connection.send_command("DISARM")
                    if bench_test_mode:
                        print("Bench test debug mode enabled: encoders ignored; open-loop bench motion active.")
                    else:
                        print("Bench test debug mode disabled: returning to normal controller mode.")
                elif action == "disarm":
                    connection.send_command("DISARM")
                    reset_motion_state(clear_calibration=True)
                elif action == "slot_prev":
                    if recording_active or replay_state != "IDLE":
                        print("Slot change blocked while motion is active.")
                    elif slot_confirmed:
                        print("Slot is locked; press A to deselect before changing slots.")
                    else:
                        selected_slot = (selected_slot - 1) % NUM_RECORD_SLOTS
                        print(f"Selected slot {selected_slot + 1}.")
                elif action == "slot_next":
                    if recording_active or replay_state != "IDLE":
                        print("Slot change blocked while motion is active.")
                    elif slot_confirmed:
                        print("Slot is locked; press A to deselect before changing slots.")
                    else:
                        selected_slot = (selected_slot + 1) % NUM_RECORD_SLOTS
                        print(f"Selected slot {selected_slot + 1}.")
                elif action == "slot_confirm_toggle":
                    if j4_calibration_mode:
                        print("Slot selection is unavailable while J4 calibration mode is active.")
                    elif recording_active or replay_state != "IDLE":
                        print("Slot lock is blocked while motion is active.")
                    else:
                        slot_confirmed = not slot_confirmed
                        state_text = "confirmed" if slot_confirmed else "deselected"
                        print(f"Slot {selected_slot + 1} {state_text}.")
                elif action == "toggle_record":
                    active_trajectory = recorded_slots[selected_slot]
                    if j4_calibration_mode:
                        print("Recording disabled while J4 calibration mode is active.")
                    elif not slot_confirmed:
                        print("Press A to confirm the selected slot first.")
                    elif not controller_ready or not ik_ready or target_pose is None or seed_angles_rad is None:
                        print("Recording requires a ready controller and valid Cartesian target.")
                    elif recording_active:
                        recording_active = False
                        print(f"Recording stopped on slot {selected_slot + 1} after {len(active_trajectory)} frames.")
                    else:
                        replay_state = "IDLE"
                        active_trajectory.clear()
                        record_start_time = now
                        recording_active = True
                        seed_copy = np.array(seed_angles_rad, dtype=float, copy=True)
                        pose_copy = np.array(target_pose, dtype=float, copy=True)
                        initial_angles = np.asarray(
                            ikgeo_model.model_radians_to_encoder_degrees(seed_copy), dtype=float
                        ).copy()
                        active_trajectory.append({
                            "t": 0.0,
                            "angles_deg": initial_angles,
                            "pose": pose_copy,
                            "seed_rad": seed_copy,
                        })
                        joint_targets_deg = initial_angles.copy()
                        print(f"Recording started on slot {selected_slot + 1}.")
                elif action == "trigger_replay":
                    active_trajectory = recorded_slots[selected_slot]
                    if replay_state in {"HOMING", "PLAYING"}:
                        replay_state = "IDLE"
                        recording_active = False
                        target_pose = None
                        seed_angles_rad = None
                        print("Replay cancelled; re-locking onto current measured pose.")
                    elif not slot_confirmed:
                        print("Press A to confirm the selected slot first.")
                    elif not active_trajectory:
                        print("Replay blocked: selected slot is empty.")
                    elif not controller_ready or not ik_ready:
                        print("Replay blocked: controller telemetry is not ready.")
                    else:
                        recording_active = False
                        replay_state = "HOMING"
                        replay_start_time = now
                        replay_frame_index = 0
                        start_frame = active_trajectory[0]
                        target_pose = np.array(start_frame["pose"], dtype=float, copy=True)
                        seed_angles_rad = np.array(start_frame["seed_rad"], dtype=float, copy=True)
                        joint_targets_deg = np.asarray(start_frame["angles_deg"], dtype=float).copy()
                        last_send = now
                        connection.send_command(format_joint_gcode(joint_targets_deg))
                        print(f"Replay started on slot {selected_slot + 1}: homing to the recorded start pose.")

            if gamepad_update.j4_calibration_toggle:
                connection.send_command("DISARM")
                bench_test_mode = False
                last_bench_rates = (0, 0, 0, 0, 0, 0, 0)
                if j4_calibration_mode:
                    reset_motion_state(clear_calibration=True)
                    print("J4 calibration mode stopped.")
                else:
                    reset_motion_state(clear_calibration=False)
                    j4_calibration_mode = True
                    last_j4_jog_rate = 0
                    print("J4 calibration mode: align the witness marks, then press START.")

            now = time.monotonic()
            button_inputs = [
                control for control in gamepad_update.active_controls
                if "m/s" not in control and "steps/s" not in control
            ]
            if button_inputs:
                last_controller_button = " | ".join(button_inputs)
                last_controller_button_time = now

            if j4_calibration_mode:
                movement_inputs = []
                if gamepad_update.j4_jog_rate != 0:
                    movement_inputs.append(
                        f"{gamepad_update.j4_jog_source} -> J4 "
                        f"{gamepad_update.j4_jog_rate:+d} steps/s"
                    )
            else:
                movement_inputs = [
                    control for control in gamepad_update.active_controls
                    if "m/s" in control and "steps/s" not in control
                ]
            if now - last_controller_button_time < 1.0:
                movement_inputs.append(last_controller_button)
            last_controller_input = " | ".join(movement_inputs) if movement_inputs else "None"
            telemetry = connection.read_telemetry()
            if telemetry is not None:
                last_telemetry_time = now
                steps = telemetry["steps"]
                if latest_telemetry is not None:
                    motor_step_deltas = [curr - prev for curr, prev in zip(steps, prev_steps)]
                prev_steps = list(steps)
                latest_telemetry = telemetry
                if not bench_test_mode:
                    if telemetry["faults"]:
                        reset_motion_state()
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
                                joint_targets_deg = np.asarray(
                                    ikgeo_model.model_radians_to_encoder_degrees(seed_angles_rad), dtype=float
                                ).copy()
                                print("Cartesian target initialized from measured camera pose.")
                            except (ValueError, ikgeo_model.RobotModelNotConfigured) as error:
                                ik_ready = False
                                print(f"Cartesian IK disabled: {error}")
                    else:
                        reset_motion_state()

            if now - last_telemetry_time > 0.5 and not bench_test_mode:
                reset_motion_state()

            active_trajectory = recorded_slots[selected_slot]
            if replay_state == "HOMING" and controller_ready and ik_ready:
                if not active_trajectory:
                    replay_state = "IDLE"
                else:
                    start_error = (
                        np.max(
                            np.abs(
                                np.asarray(latest_telemetry["angles"], dtype=float)
                                - np.asarray(active_trajectory[0]["angles_deg"], dtype=float)
                            )
                        )
                        if latest_telemetry is not None
                        else float("inf")
                    )
                    runtime = now - replay_start_time
                    if runtime >= REPLAY_HOMING_TIMEOUT_S or start_error <= REPLAY_START_TOLERANCE_DEG:
                        replay_state = "PLAYING"
                        replay_start_time = now
                        replay_frame_index = 0
                        print(f"Replay homing complete on slot {selected_slot + 1}; trajectory playback started.")
                    else:
                        if now - last_send >= SEND_PERIOD_S:
                            connection.send_command(
                                format_joint_gcode(np.asarray(active_trajectory[0]["angles_deg"], dtype=float))
                            )
                            last_send = now
            elif replay_state == "PLAYING" and controller_ready and ik_ready:
                if not active_trajectory:
                    replay_state = "IDLE"
                else:
                    elapsed = now - replay_start_time
                    if elapsed >= active_trajectory[-1]["t"]:
                        frame = active_trajectory[-1]
                        target_pose = np.array(frame["pose"], dtype=float, copy=True)
                        seed_angles_rad = np.array(frame["seed_rad"], dtype=float, copy=True)
                        joint_targets_deg = np.asarray(frame["angles_deg"], dtype=float).copy()
                        connection.send_command(format_joint_gcode(joint_targets_deg))
                        last_send = now
                        replay_state = "IDLE"
                        recording_active = False
                        print(f"Replay complete on slot {selected_slot + 1}.")
                    else:
                        while (
                            replay_frame_index + 1 < len(active_trajectory)
                            and elapsed >= active_trajectory[replay_frame_index + 1]["t"]
                        ):
                            replay_frame_index += 1
                        frame = active_trajectory[replay_frame_index]
                        target_pose = np.array(frame["pose"], dtype=float, copy=True)
                        seed_angles_rad = np.array(frame["seed_rad"], dtype=float, copy=True)
                        joint_targets_deg = np.asarray(frame["angles_deg"], dtype=float).copy()
                        if now - last_send >= SEND_PERIOD_S or last_send == 0.0:
                            connection.send_command(format_joint_gcode(joint_targets_deg))
                            last_send = now

            if bench_test_mode:
                bench_rates = tuple(gamepad_update.bench_motor_rates)
                if bench_rates != last_bench_rates and any(rate != 0 for rate in bench_rates):
                    payload = ",".join(str(int(round(rate))) for rate in bench_rates)
                    connection.send_command(f"BENCH_MULTI,{payload},200")
                    last_bench_send = now
                elif any(rate != 0 for rate in bench_rates) and (
                    now - last_bench_send >= 0.08 or last_bench_send == 0.0
                ):
                    payload = ",".join(str(int(round(rate))) for rate in bench_rates)
                    connection.send_command(f"BENCH_MULTI,{payload},200")
                    last_bench_send = now
                elif not any(rate != 0 for rate in bench_rates) and any(
                    rate != 0 for rate in last_bench_rates
                ):
                    connection.send_command("BENCH_MULTI,0,0,0,0,0,0,0,10")
                    last_bench_send = 0.0
                last_bench_rates = bench_rates

            elif j4_calibration_mode:
                jog_rate = gamepad_update.j4_jog_rate
                if gamepad_update.j4_calibration_confirm and jog_rate == 0:
                    try:
                        connection.send_gcode_line("DISARM")
                        connection.send_gcode_line("ZERO4")
                        connection.send_gcode_line("CFG,I2C,1")
                        connection.send_gcode_line("CFG,CAL,1")
                        connection.send_gcode_line("ARM")
                    except (RuntimeError, TimeoutError) as error:
                        print(f"Firmware rejected calibration sequence: {error}")
                    else:
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
            else:
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

                if replay_state == "IDLE" and target_pose is not None and seed_angles_rad is not None and controller_ready and ik_ready:
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
                            if recording_active:
                                recorded_slots[selected_slot].append({
                                    "t": now - record_start_time,
                                    "angles_deg": np.asarray(joint_targets_deg, dtype=float).copy(),
                                    "pose": np.asarray(target_pose, dtype=float).copy(),
                                    "seed_rad": np.asarray(solved_angles_rad, dtype=float).copy(),
                                })
                        except (ValueError, ikgeo_model.RobotModelNotConfigured) as error:
                            if now - last_ik_error_time >= 1.0:
                                print(f"IK target rejected: {error}")
                                last_ik_error_time = now
                        last_send = now
                elif telemetry is not None and not telemetry["ready"]:
                    if not last_not_ready_message:
                        print("Firmware is not ready: verify encoders and confirm I2C and calibration.")
                        last_not_ready_message = True

            if now - last_hud_draw >= HUD_REFRESH_PERIOD_S:
                last_hud_draw = now
                screen.fill((24, 26, 30))

                if bench_test_mode:
                    header = "MODE: BENCH TEST DEBUG (ENCODERS IGNORED | OPEN-LOOP)"
                    header_color = (255, 180, 70)
                elif j4_calibration_mode:
                    header = f"MODE: J4 CALIBRATION | JOG: {gamepad_update.j4_jog_rate:+d} steps/s"
                    header_color = (255, 215, 70)
                elif replay_state != "IDLE":
                    header = f"MODE: REPLAY {replay_state}"
                    header_color = (118, 197, 255)
                elif recording_active:
                    header = "MODE: RECORDING TRAJECTORY"
                    header_color = (255, 160, 90)
                elif controller_ready and ik_ready:
                    header = "MODE: ARMED (CLOSED-LOOP IK LIVE)"
                    header_color = (60, 240, 90)
                else:
                    header = "MODE: DISARMED / OPEN-LOOP / WAITING"
                    header_color = (255, 90, 90)
                screen.blit(font.render(header, True, header_color), (16, 14))

                fault_hex = f"0x{latest_telemetry['faults']:02X}" if latest_telemetry else "N/A"
                age_ms = int((now - last_telemetry_time) * 1000) if last_telemetry_time > 0 else -1
                firmware_state = latest_telemetry["state"] if latest_telemetry else "NO LINK"
                subheader = f"ESP32: {firmware_state} | Fault: {fault_hex} | Telemetry age: {age_ms} ms"
                screen.blit(font.render(subheader, True, (190, 195, 205)), (16, 39))
                slot_bar_parts = []
                for index in range(NUM_RECORD_SLOTS):
                    slot_text = f"S{index + 1}:{len(recorded_slots[index])}"
                    if index == selected_slot:
                        slot_text = f"[{slot_text}]"
                        if not slot_confirmed:
                            slot_text = f">{slot_text}<"
                    slot_bar_parts.append(slot_text)
                screen.blit(font.render(f"Slots: {' '.join(slot_bar_parts)}", True, (255, 215, 120)), (390, 265))

                screen.blit(font.render("MOTOR CHANNELS (STEP TOTAL / 100 ms DELTA)", True, (120, 180, 255)), (16, 72))
                steps_list = latest_telemetry["steps"] if latest_telemetry else [0] * 7
                for index, label in enumerate(MOTOR_LABELS):
                    delta = motor_step_deltas[index]
                    pulse_state = f"PULSE {delta:+d}" if delta else "IDLE"
                    color = (80, 255, 140) if delta else (210, 210, 210)
                    row = f"{label:<6} | {steps_list[index]:>9d} | {pulse_state}"
                    screen.blit(font.render(row, True, color), (16, 98 + index * 25))

                screen.blit(font.render("JOINT ANGLES: HW MEASURED -> SW TARGET (ERR)", True, (120, 180, 255)), (390, 72))
                angles_list = latest_telemetry["angles"] if latest_telemetry else [float("nan")] * 6
                for index, label in enumerate(JOINT_LABELS):
                    measured = angles_list[index]
                    target = joint_targets_deg[index]
                    if np.isfinite(measured):
                        err_deg = target - measured
                        measured_text = f"{measured:>7.2f}"
                        err_text = f"{err_deg:+6.2f}"
                        joint_color = (80, 255, 140) if abs(err_deg) <= 2.0 else (255, 215, 90)
                    else:
                        measured_text = "   NONE"
                        err_text = "   N/A"
                        joint_color = (210, 210, 210)
                    target_text = f"{target:>7.2f}"
                    row = f"{label}: {measured_text} -> {target_text} ({err_text}) deg"
                    screen.blit(font.render(row, True, joint_color), (390, 98 + index * 25))

                input_text = (
                    " | ".join(gamepad_update.bench_controls)
                    if bench_test_mode and gamepad_update.bench_controls
                    else last_controller_input
                )
                screen.blit(font.render(f"INPUT IN USE: {input_text}", True, (255, 215, 120)), (16, 265))
                screen.blit(font.render(f"RECORD: {'ON' if recording_active else 'OFF'} | REPLAY: {replay_state}", True, (255, 215, 120)), (16, 286))
                if bench_test_mode:
                    screen.blit(font.render("BENCH: LSX=J1 | LSY=J2 | RSY=J3 | RSX=J4", True, (190, 195, 205)), (16, 307))
                    screen.blit(font.render("BENCH: LB/RB=J5 | LT/RT=J6 | 2xSTART=exit", True, (190, 195, 205)), (16, 328))
                else:
                    screen.blit(font.render("Cartesian: LS X/Y + RS Y -> XYZ", True, (190, 195, 205)), (16, 307))
                    screen.blit(font.render("J4: LS Y coarse | D-pad / LB / RB fine", True, (190, 195, 205)), (16, 328))
                screen.blit(font.render(f"Slot {selected_slot + 1}: {'LOCKED' if slot_confirmed else 'READY'}", True, (255, 215, 120)), (390, 286))
                screen.blit(font.render("L/R: slots | A: lock/unlock | START: zero", True, (190, 195, 205)), (390, 307))
                screen.blit(font.render("X/R: record | Y/P: replay | B: E-stop", True, (190, 195, 205)), (390, 328))

                target_xyz_mm = target_pose[:3, 3] * 1000.0 if target_pose is not None else None
                measured_xyz_mm = None
                if ik_ready and len(angles_list) == 6 and np.all(np.isfinite(angles_list)):
                    try:
                        meas_rad = ikgeo_model.encoder_degrees_to_model_radians(np.asarray(angles_list, dtype=float))
                        meas_pose = ikgeo_model.forward_camera_pose(meas_rad)
                        measured_xyz_mm = meas_pose[:3, 3] * 1000.0
                    except (ValueError, ikgeo_model.RobotModelNotConfigured):
                        measured_xyz_mm = None

                if target_xyz_mm is not None:
                    tx_mm, ty_mm, tz_mm = target_xyz_mm
                    target_xyz_text = f"SW Target 3D (mm):   X={tx_mm:7.2f}  Y={ty_mm:7.2f}  Z={tz_mm:7.2f}"
                else:
                    target_xyz_text = "SW Target 3D (mm):   waiting for valid telemetry / arming"

                if measured_xyz_mm is not None:
                    mx_mm, my_mm, mz_mm = measured_xyz_mm
                    if target_xyz_mm is not None:
                        err_mm = float(np.linalg.norm(target_xyz_mm - measured_xyz_mm))
                        meas_xyz_text = (
                            f"HW Measured 3D (mm): X={mx_mm:7.2f}  Y={my_mm:7.2f}  Z={mz_mm:7.2f}"
                            f"  | 3D Err: {err_mm:6.2f} mm"
                        )
                    else:
                        meas_xyz_text = f"HW Measured 3D (mm): X={mx_mm:7.2f}  Y={my_mm:7.2f}  Z={mz_mm:7.2f}"
                else:
                    meas_xyz_text = "HW Measured 3D (mm): waiting for 6 valid encoder readings"

                screen.blit(font.render(target_xyz_text, True, (255, 220, 120)), (16, 360))
                screen.blit(font.render(meas_xyz_text, True, (130, 235, 255)), (16, 384))
                screen.blit(
                    font.render(
                        f"Live G-Code TX (#{connection.tx_gcode_count}): {connection.last_gcode_command}",
                        True,
                        (140, 250, 160),
                    ),
                    (16, 416),
                )
                screen.blit(
                    font.render(
                        f"Last CMD TX: {connection.last_tx_command} | Last ESP32 RX: {connection.last_rx_reply}",
                        True,
                        (230, 190, 160),
                    ),
                    (16, 440),
                )
                pygame.display.flip()

    except KeyboardInterrupt:
        pass
    finally:
        connection.close()
        controller.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
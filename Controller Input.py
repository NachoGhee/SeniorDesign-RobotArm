import sys
import time

import pygame
import serial

SERIAL_PORT = "COM3"
BAUD_RATE = 115200
DEADZONE = 0.15
SEND_PERIOD_S = 0.05
JOINT_SPEED_DEG_S = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
JOINT_LIMITS_DEG = [(0.0, 360.0)] * 6
AXIS_TO_JOINT = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5}
AXIS_SIGN = {0: 1, 1: -1, 2: 1, 3: -1, 4: 1, 5: 1}
ESTOP_BUTTON = 1


def read_telemetry(port, receive_buffer):
    latest = None
    receive_buffer.extend(port.read(port.in_waiting))
    while b"\n" in receive_buffer:
        raw_line, _, remainder = receive_buffer.partition(b"\n")
        receive_buffer[:] = remainder
        line = raw_line.decode("ascii", errors="replace").strip()
        fields = line.split(",")
        if len(fields) == 16 and fields[0] == "S":
            try:
                latest = {
                    "faults": int(fields[2]),
                    "ready": fields[3] == "1",
                    "angles": [float(value) for value in fields[4:10]],
                }
            except ValueError:
                continue
        elif line.startswith(("ERR,", "FAULT,")):
            print(line)
    return latest


def main():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller found.")
        pygame.quit()
        return 1

    controller = pygame.joystick.Joystick(0)
    print(f"Controller: {controller.get_name()}")
    print(f"Connecting to ESP32 on {SERIAL_PORT} at {BAUD_RATE} baud...")

    try:
        port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
    except serial.SerialException as error:
        print(f"Could not open serial port: {error}")
        pygame.quit()
        return 1

    target_angles = None
    sequence = 0
    last_send = 0.0
    last_telemetry_time = 0.0
    controller_ready = False
    last_not_ready_message = False
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

            now = time.monotonic()
            telemetry = read_telemetry(port, receive_buffer)
            if telemetry is not None:
                last_telemetry_time = now
                if telemetry["faults"]:
                    print(f"Encoder fault mask: 0x{telemetry['faults']:02X}")
                    target_angles = None
                    controller_ready = False
                elif telemetry["ready"] and all(angle >= 0 for angle in telemetry["angles"]):
                    controller_ready = True
                    last_not_ready_message = False
                    if target_angles is None:
                        target_angles = telemetry["angles"][:]
                        print("Controller ready; targets initialized from encoder feedback.")
                else:
                    controller_ready = False

            if now - last_telemetry_time > 0.5:
                controller_ready = False
                target_angles = None

            if target_angles is not None and controller_ready:
                for axis, joint in AXIS_TO_JOINT.items():
                    if axis >= controller.get_numaxes():
                        continue
                    value = controller.get_axis(axis)
                    if abs(value) < DEADZONE:
                        value = 0.0
                    else:
                        value = (abs(value) - DEADZONE) / (1.0 - DEADZONE) * (1 if value > 0 else -1)
                    target_angles[joint] += value * AXIS_SIGN.get(axis, 1) * JOINT_SPEED_DEG_S[joint] * dt
                    lower, upper = JOINT_LIMITS_DEG[joint]
                    target_angles[joint] = max(lower, min(upper, target_angles[joint]))

                if now - last_send >= SEND_PERIOD_S:
                    sequence += 1
                    values = ",".join(f"{angle:.3f}" for angle in target_angles)
                    port.write(f"G,{sequence},{values}\n".encode("ascii"))
                    last_send = now
            elif telemetry is not None and not telemetry["ready"]:
                if not last_not_ready_message:
                    print("Firmware is not armed: configure SSI pins and calibration before motion.")
                    last_not_ready_message = True

    except KeyboardInterrupt:
        pass
    finally:
        port.close()
        pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())

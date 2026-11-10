from dataclasses import dataclass
import os

import pygame


DEADZONE = 0.15
CARTESIAN_RIGHT_Y_AXIS = int(os.environ.get("POSE_GAMEPAD_RIGHT_Y_AXIS", "4"))
CARTESIAN_AXIS_TO_XYZ = {0: 0, 1: 1, CARTESIAN_RIGHT_Y_AXIS: 2}
CARTESIAN_AXIS_LABELS = {0: "LS X", 1: "LS Y", CARTESIAN_RIGHT_Y_AXIS: "RS Y"}
CARTESIAN_AXIS_SIGN = {0: 1, 1: -1, CARTESIAN_RIGHT_Y_AXIS: 1}
CARTESIAN_SPEED_M_S = (0.10, 0.10, 0.10)
ESTOP_BUTTON = int(os.environ.get("POSE_GAMEPAD_ESTOP_BUTTON", "1"))
START_INIT_BUTTON = int(os.environ.get("POSE_GAMEPAD_START_BUTTON", "11"))
J4_BACK_BUTTON = int(os.environ.get("POSE_GAMEPAD_BACK_BUTTON", "10"))
J4_A_BUTTON = int(os.environ.get("POSE_GAMEPAD_A_BUTTON", "0"))
J4_LEFT_BUMPER = int(os.environ.get("POSE_GAMEPAD_LB_BUTTON", "6"))
J4_RIGHT_BUMPER = int(os.environ.get("POSE_GAMEPAD_RB_BUTTON", "7"))
J4_JOG_AXIS = 1
J4_JOG_MAX_RATE = 3200
J4_FINE_JOG_RATE = 250


@dataclass(frozen=True)
class GamepadUpdate:
    quit_requested: bool
    actions: tuple
    desired_velocity: tuple
    j4_calibration_toggle: bool = False
    j4_calibration_confirm: bool = False
    j4_jog_rate: int = 0
    j4_jog_source: str = "NONE"
    active_controls: tuple = ()


def apply_deadzone(value):
    if abs(value) < DEADZONE:
        return 0.0
    return (abs(value) - DEADZONE) / (1.0 - DEADZONE) * (1 if value > 0 else -1)


class GamepadController:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            pygame.quit()
            raise RuntimeError("No controller found.")
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def poll(self):
        quit_requested = False
        actions = []
        active_controls = []
        j4_calibration_toggle = False
        j4_calibration_confirm = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit_requested = True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == ESTOP_BUTTON:
                    actions.append("estop")
                    active_controls.append(f"Button {event.button}: E-stop")
                elif event.button == J4_BACK_BUTTON:
                    j4_calibration_toggle = True
                    active_controls.append("BACK/SELECT: toggle J4 calibration")
                elif event.button == J4_A_BUTTON:
                    j4_calibration_confirm = True
                    active_controls.append("A: confirm J4 zero")
                elif event.button == START_INIT_BUTTON:
                    actions.append("confirm_i2c")
                    actions.append("confirm_calibration")
                    j4_calibration_confirm = True
                    active_controls.append("START: confirm J4 zero / setup")
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    actions.append("confirm_i2c")
                    active_controls.append("Key S: confirm I2C")
                elif event.key == pygame.K_c:
                    actions.append("confirm_calibration")
                    active_controls.append("Key C: confirm calibration")
                elif event.key == pygame.K_a:
                    actions.append("arm")
                    active_controls.append("Key A: arm")
                elif event.key == pygame.K_d:
                    actions.append("disarm")
                    active_controls.append("Key D: disarm")

        velocity = [0.0, 0.0, 0.0]
        for axis, coordinate in CARTESIAN_AXIS_TO_XYZ.items():
            if axis >= self.joystick.get_numaxes():
                continue
            value = apply_deadzone(self.joystick.get_axis(axis))
            velocity[coordinate] = (
                value * CARTESIAN_AXIS_SIGN[axis] * CARTESIAN_SPEED_M_S[coordinate]
            )
            if value != 0:
                active_controls.append(
                    f"{CARTESIAN_AXIS_LABELS[axis]} {velocity[coordinate]:+.2f} m/s"
                )

        jog_axis = self.joystick.get_axis(J4_JOG_AXIS) if J4_JOG_AXIS < self.joystick.get_numaxes() else 0.0
        jog_rate = int(round(-apply_deadzone(jog_axis) * J4_JOG_MAX_RATE))
        j4_jog_source = "LS Y" if jog_rate != 0 else "NONE"
        hat_y = self.joystick.get_hat(0)[1] if self.joystick.get_numhats() > 0 else 0
        if hat_y != 0:
            jog_rate = J4_FINE_JOG_RATE if hat_y > 0 else -J4_FINE_JOG_RATE
            j4_jog_source = "D-PAD UP" if hat_y > 0 else "D-PAD DOWN"
        elif self.joystick.get_numbuttons() > max(J4_LEFT_BUMPER, J4_RIGHT_BUMPER):
            bumper_direction = int(self.joystick.get_button(J4_RIGHT_BUMPER)) - int(
                self.joystick.get_button(J4_LEFT_BUMPER)
            )
            if bumper_direction != 0:
                jog_rate = J4_FINE_JOG_RATE if bumper_direction > 0 else -J4_FINE_JOG_RATE
                j4_jog_source = "RB" if bumper_direction > 0 else "LB"

        if jog_rate != 0:
            active_controls.append(f"{j4_jog_source} -> J4 {jog_rate:+d} steps/s")

        return GamepadUpdate(
            quit_requested, tuple(actions), tuple(velocity), j4_calibration_toggle,
            j4_calibration_confirm, jog_rate, j4_jog_source, tuple(active_controls)
        )

    def get_name(self):
        return self.joystick.get_name()

    def close(self):
        pygame.quit()
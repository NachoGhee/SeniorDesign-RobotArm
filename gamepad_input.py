from dataclasses import dataclass

import pygame


DEADZONE = 0.15
CARTESIAN_AXIS_TO_XYZ = {0: 0, 1: 1, 4: 2}
CARTESIAN_AXIS_SIGN = {0: 1, 1: -1, 4: 1}
CARTESIAN_SPEED_M_S = (0.10, 0.10, 0.10)
ESTOP_BUTTON = 1


@dataclass(frozen=True)
class GamepadUpdate:
    quit_requested: bool
    actions: tuple
    desired_velocity: tuple


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
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit_requested = True
            elif event.type == pygame.JOYBUTTONDOWN and event.button == ESTOP_BUTTON:
                actions.append("estop")
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    actions.append("confirm_ssi")
                elif event.key == pygame.K_c:
                    actions.append("confirm_calibration")
                elif event.key == pygame.K_a:
                    actions.append("arm")
                elif event.key == pygame.K_d:
                    actions.append("disarm")

        velocity = [0.0, 0.0, 0.0]
        for axis, coordinate in CARTESIAN_AXIS_TO_XYZ.items():
            if axis >= self.joystick.get_numaxes():
                continue
            value = apply_deadzone(self.joystick.get_axis(axis))
            velocity[coordinate] = (
                value * CARTESIAN_AXIS_SIGN[axis] * CARTESIAN_SPEED_M_S[coordinate]
            )

        return GamepadUpdate(quit_requested, tuple(actions), tuple(velocity))

    def get_name(self):
        return self.joystick.get_name()

    def close(self):
        pygame.quit()
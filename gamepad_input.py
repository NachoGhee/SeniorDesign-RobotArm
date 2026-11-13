from dataclasses import dataclass
import os
import time

import pygame


DEADZONE = 0.15
DOMINANT_AXIS_RATIO = 1.5
CARTESIAN_RIGHT_Y_AXIS = int(os.environ.get("POSE_GAMEPAD_RIGHT_Y_AXIS", "4"))
RIGHT_X_AXIS = int(os.environ.get("POSE_GAMEPAD_RIGHT_X_AXIS", "3"))
RIGHT_Y_AXIS = CARTESIAN_RIGHT_Y_AXIS
LT_AXIS = int(os.environ.get("POSE_GAMEPAD_LT_AXIS", "2"))
RT_AXIS = int(os.environ.get("POSE_GAMEPAD_RT_AXIS", "5"))
LT_BUTTON = int(os.environ.get("POSE_GAMEPAD_LT_BUTTON", "8"))
RT_BUTTON = int(os.environ.get("POSE_GAMEPAD_RT_BUTTON", "9"))
START_DOUBLE_CLICK_WINDOW_S = 0.40
CARTESIAN_AXIS_TO_XYZ = {0: 0, 1: 1, CARTESIAN_RIGHT_Y_AXIS: 2}
CARTESIAN_AXIS_LABELS = {0: "LS X", 1: "LS Y", CARTESIAN_RIGHT_Y_AXIS: "RS Y"}
CARTESIAN_AXIS_SIGN = {0: 1, 1: -1, CARTESIAN_RIGHT_Y_AXIS: 1}
CARTESIAN_SPEED_M_S = (0.10, 0.10, 0.10)
ESTOP_BUTTON = int(os.environ.get("POSE_GAMEPAD_ESTOP_BUTTON", "1"))
RECORD_BUTTON = int(os.environ.get("POSE_GAMEPAD_RECORD_BUTTON", "3"))
REPLAY_BUTTON = int(os.environ.get("POSE_GAMEPAD_REPLAY_BUTTON", "4"))
START_INIT_BUTTON = int(os.environ.get("POSE_GAMEPAD_START_BUTTON", "11"))
J4_BACK_BUTTON = int(os.environ.get("POSE_GAMEPAD_BACK_BUTTON", "10"))
J4_A_BUTTON = int(os.environ.get("POSE_GAMEPAD_A_BUTTON", "0"))
J4_LEFT_BUMPER = int(os.environ.get("POSE_GAMEPAD_LB_BUTTON", "6"))
J4_RIGHT_BUMPER = int(os.environ.get("POSE_GAMEPAD_RB_BUTTON", "7"))
J4_JOG_AXIS = 1
J4_JOG_MAX_RATE = 3200
J4_FINE_JOG_RATE = 250
BENCH_J1_MAX_RATE = 2000
BENCH_J2_MAX_RATE = 5300
BENCH_J3_MAX_RATE = 5100
BENCH_J4_MAX_RATE = 3200
BENCH_J5_RATE = 1150
BENCH_J6_MAX_RATE = 900


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
    bench_motor_rates: tuple = (0, 0, 0, 0, 0, 0, 0)
    bench_controls: tuple = ()


def apply_deadzone(value):
    if abs(value) < DEADZONE:
        return 0.0
    return (abs(value) - DEADZONE) / (1.0 - DEADZONE) * (1 if value > 0 else -1)


def filter_dominant_axis(x_value, y_value):
    if abs(y_value) > abs(x_value) * DOMINANT_AXIS_RATIO:
        return 0.0, y_value
    if abs(x_value) > abs(y_value) * DOMINANT_AXIS_RATIO:
        return x_value, 0.0
    return x_value, y_value


class GamepadController:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            pygame.quit()
            raise RuntimeError("No controller found.")
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.prev_hat_x = 0
        self.last_start_press_time = 0.0
        self.num_axes = self.joystick.get_numaxes()
        self.num_buttons = self.joystick.get_numbuttons()
        mode = os.environ.get("POSE_GAMEPAD_MODE", "").strip().lower()
        mode = mode.replace("-", "").replace("_", "").replace(" ", "")
        force_dinput = mode in {"dinput", "directinput", "8bitdo"}
        self.dinput_mapping = self.num_buttons > 12 or force_dinput

        right_x_override = os.environ.get("POSE_GAMEPAD_RIGHT_X_AXIS")
        right_y_override = os.environ.get("POSE_GAMEPAD_RIGHT_Y_AXIS")
        lt_axis_override = os.environ.get("POSE_GAMEPAD_LT_AXIS")
        rt_axis_override = os.environ.get("POSE_GAMEPAD_RT_AXIS")
        if self.dinput_mapping:
            self.right_x_axis = (
                int(right_x_override) if right_x_override is not None else RIGHT_X_AXIS
            )
            self.right_y_axis = (
                int(right_y_override) if right_y_override is not None else RIGHT_Y_AXIS
            )
            self.lt_axis = int(lt_axis_override) if lt_axis_override is not None else -1
            self.rt_axis = int(rt_axis_override) if rt_axis_override is not None else -1
            self.lt_button = LT_BUTTON
            self.rt_button = RT_BUTTON
        else:
            self.right_x_axis = int(right_x_override) if right_x_override is not None else 2
            self.right_y_axis = int(right_y_override) if right_y_override is not None else 3
            self.lt_axis = int(lt_axis_override) if lt_axis_override is not None else 4
            self.rt_axis = int(rt_axis_override) if rt_axis_override is not None else 5
            self.lt_button = -1
            self.rt_button = -1

        self.cartesian_axis_to_xyz = {0: 0, 1: 1, self.right_y_axis: 2}
        self.cartesian_axis_labels = {0: "LS X", 1: "LS Y", self.right_y_axis: "RS Y"}
        self.cartesian_axis_sign = {0: 1, 1: -1, self.right_y_axis: 1}
        pygame.event.pump()
        self.lt_rest = (
            self.joystick.get_axis(self.lt_axis)
            if 0 <= self.lt_axis < self.num_axes
            else 0.0
        )
        self.rt_rest = (
            self.joystick.get_axis(self.rt_axis)
            if 0 <= self.rt_axis < self.num_axes
            else 0.0
        )
        mapping = "D-Input" if self.dinput_mapping else "XInput/SDL"
        print(
            f"Gamepad: {self.joystick.get_name()} ({mapping}, "
            f"{self.num_axes} axes, {self.num_buttons} buttons) | "
            f"J1=Axis 0 (LS X), J2=Axis 1 (LS Y), "
            f"J3=Axis {self.right_y_axis} (RS_Y), "
            f"J4=Axis {self.right_x_axis} (RS_X), "
            f"J5=LB Button {J4_LEFT_BUMPER}/RB Button {J4_RIGHT_BUMPER}, "
            f"J6=LT {self._mapping_label(self.lt_axis, self.lt_button)}/"
            f"RT {self._mapping_label(self.rt_axis, self.rt_button)}"
        )

    @staticmethod
    def _mapping_label(axis_index, button_index):
        mappings = []
        if axis_index >= 0:
            mappings.append(f"Axis {axis_index}")
        if button_index >= 0:
            mappings.append(f"Button {button_index}")
        return " + ".join(mappings) if mappings else "disabled"

    def _read_trigger(self, axis_index, button_index, rest_value):
        if (
            0 <= button_index < self.joystick.get_numbuttons()
            and self.joystick.get_button(button_index)
        ):
            return 1.0
        if (
            axis_index in {self.right_x_axis, self.right_y_axis}
            or axis_index < 0
            or axis_index >= self.joystick.get_numaxes()
        ):
            return 0.0

        raw = self.joystick.get_axis(axis_index)
        if rest_value < -0.5:
            value = (raw - rest_value) / (1.0 - rest_value)
        else:
            value = max(0.0, raw - rest_value)
        return apply_deadzone(max(0.0, min(1.0, value)))

    def poll(self):
        quit_requested = False
        actions = []
        active_controls = []
        bench_controls = []
        bench_motor_rates = [0, 0, 0, 0, 0, 0, 0]
        j4_calibration_toggle = False
        j4_calibration_confirm = False
        now = time.monotonic()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit_requested = True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == ESTOP_BUTTON:
                    actions.append("estop")
                    active_controls.append(f"Button {event.button}: E-stop")
                elif event.button == RECORD_BUTTON:
                    actions.append("toggle_record")
                    active_controls.append(f"Button {event.button} (X): Record toggle")
                elif event.button == REPLAY_BUTTON:
                    actions.append("trigger_replay")
                    active_controls.append(f"Button {event.button} (Y): Replay motion")
                elif event.button == J4_BACK_BUTTON:
                    j4_calibration_toggle = True
                    active_controls.append("BACK/SELECT: toggle J4 calibration")
                elif event.button == J4_A_BUTTON:
                    actions.append("slot_confirm_toggle")
                    active_controls.append("A: confirm/deselect slot")
                elif event.button == START_INIT_BUTTON:
                    elapsed = now - self.last_start_press_time
                    if 0.0 < elapsed <= START_DOUBLE_CLICK_WINDOW_S:
                        actions.append("toggle_bench_mode")
                        bench_controls.append("START x2: bench toggle")
                        active_controls.append("START double-click: toggle bench mode")
                    else:
                        actions.append("confirm_i2c")
                        actions.append("confirm_calibration")
                        j4_calibration_confirm = True
                        active_controls.append("START: confirm J4 zero / setup")
                    self.last_start_press_time = now
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
                elif event.key == pygame.K_r:
                    actions.append("toggle_record")
                    active_controls.append("Key R: Record toggle")
                elif event.key == pygame.K_p:
                    actions.append("trigger_replay")
                    active_controls.append("Key P: Replay motion")
                elif event.key == pygame.K_b:
                    actions.append("toggle_bench_mode")
                    bench_controls.append("Key B: bench toggle")
                    active_controls.append("Key B: toggle bench mode")
                elif event.key == pygame.K_LEFT:
                    actions.append("slot_prev")
                    active_controls.append("Key LEFT: prev slot")
                elif event.key == pygame.K_RIGHT:
                    actions.append("slot_next")
                    active_controls.append("Key RIGHT: next slot")
                elif event.key == pygame.K_RETURN:
                    actions.append("slot_confirm_toggle")
                    active_controls.append("Key ENTER: confirm/deselect slot")

        velocity = [0.0, 0.0, 0.0]
        for axis, coordinate in self.cartesian_axis_to_xyz.items():
            if axis >= self.joystick.get_numaxes():
                continue
            value = apply_deadzone(self.joystick.get_axis(axis))
            velocity[coordinate] = (
                value * self.cartesian_axis_sign[axis] * CARTESIAN_SPEED_M_S[coordinate]
            )
            if value != 0:
                active_controls.append(
                    f"{self.cartesian_axis_labels[axis]} {velocity[coordinate]:+.2f} m/s"
                )

        ls_x = apply_deadzone(self.joystick.get_axis(0)) if self.joystick.get_numaxes() > 0 else 0.0
        ls_y = apply_deadzone(self.joystick.get_axis(1)) if self.joystick.get_numaxes() > 1 else 0.0
        ls_x, ls_y = filter_dominant_axis(ls_x, ls_y)
        rs_y = apply_deadzone(self.joystick.get_axis(self.right_y_axis)) if self.joystick.get_numaxes() > self.right_y_axis else 0.0
        rs_x = apply_deadzone(self.joystick.get_axis(self.right_x_axis)) if self.joystick.get_numaxes() > self.right_x_axis else 0.0
        rs_x, rs_y = filter_dominant_axis(rs_x, rs_y)
        bench_motor_rates[0] = int(round(ls_x * BENCH_J1_MAX_RATE))
        bench_motor_rates[1] = int(round(-ls_y * BENCH_J2_MAX_RATE))
        bench_motor_rates[2] = int(round(-ls_y * BENCH_J2_MAX_RATE))
        bench_motor_rates[3] = int(round(-rs_y * BENCH_J3_MAX_RATE))
        bench_motor_rates[4] = int(round(rs_x * BENCH_J4_MAX_RATE))

        if self.joystick.get_numbuttons() > max(J4_LEFT_BUMPER, J4_RIGHT_BUMPER):
            bumper_direction = int(self.joystick.get_button(J4_RIGHT_BUMPER)) - int(
                self.joystick.get_button(J4_LEFT_BUMPER)
            )
            if bumper_direction > 0:
                bench_motor_rates[5] = BENCH_J5_RATE
                bench_controls.append("RB -> J5 +")
            elif bumper_direction < 0:
                bench_motor_rates[5] = -BENCH_J5_RATE
                bench_controls.append("LB -> J5 -")

        lt_signal = self._read_trigger(self.lt_axis, self.lt_button, self.lt_rest)
        rt_signal = self._read_trigger(self.rt_axis, self.rt_button, self.rt_rest)
        bench_motor_rates[6] = int(round((rt_signal - lt_signal) * BENCH_J6_MAX_RATE))
        if bench_motor_rates[6] != 0:
            bench_controls.append("LT/RT -> J6")

        if bench_motor_rates[0] != 0:
            bench_controls.append("LS X -> J1")
        if bench_motor_rates[1] != 0 or bench_motor_rates[2] != 0:
            bench_controls.append("-LS Y -> J2")
        if bench_motor_rates[3] != 0:
            bench_controls.append("-RS Y -> J3")
        if bench_motor_rates[4] != 0:
            bench_controls.append("RS X -> J4")

        hat_x, hat_y = self.joystick.get_hat(0) if self.joystick.get_numhats() > 0 else (0, 0)
        if hat_x != 0 and self.prev_hat_x == 0:
            if hat_x < 0:
                actions.append("slot_prev")
                active_controls.append("D-PAD LEFT: prev slot")
            elif hat_x > 0:
                actions.append("slot_next")
                active_controls.append("D-PAD RIGHT: next slot")
        self.prev_hat_x = hat_x

        jog_axis = self.joystick.get_axis(J4_JOG_AXIS) if J4_JOG_AXIS < self.joystick.get_numaxes() else 0.0
        jog_rate = int(round(-apply_deadzone(jog_axis) * J4_JOG_MAX_RATE))
        j4_jog_source = "LS Y" if jog_rate != 0 else "NONE"
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
            quit_requested,
            tuple(actions),
            tuple(velocity),
            j4_calibration_toggle,
            j4_calibration_confirm,
            jog_rate,
            j4_jog_source,
            tuple(active_controls),
            tuple(bench_motor_rates),
            tuple(bench_controls),
        )

    def get_name(self):
        return self.joystick.get_name()

    def close(self):
        pygame.quit()
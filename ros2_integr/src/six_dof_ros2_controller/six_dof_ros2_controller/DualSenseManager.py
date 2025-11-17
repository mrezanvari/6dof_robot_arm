from dualsense_controller import DualSenseController, Mapping, UpdateLevel


class DualSenseActionLayer(DualSenseController):

    class JoyStick:
        def __init__(self, x=0.0, y=0.0):
            self.x = x
            self.y = y
            self.btn: bool = False

    class JoySticks:
        def __init__(self):
            self.left = DualSenseActionLayer.JoyStick()
            self.right = DualSenseActionLayer.JoyStick()

    def __init__(
        self,
        device_index_or_device_info=0,
        left_joystick_deadzone=0.05,
        right_joystick_deadzone=0.05,
        left_trigger_deadzone=0,
        right_trigger_deadzone=0,
        gyroscope_threshold=0,
        accelerometer_threshold=0,
        orientation_threshold=0,
        mapping=Mapping.NORMALIZED,
        update_level=UpdateLevel.PAINSTAKING,
        microphone_initially_muted=True,
        microphone_invert_led=False,
    ):
        device_infos = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            raise Exception("No DualSense Controller available.")
        super().__init__(
            device_index_or_device_info,
            left_joystick_deadzone,
            right_joystick_deadzone,
            left_trigger_deadzone,
            right_trigger_deadzone,
            gyroscope_threshold,
            accelerometer_threshold,
            orientation_threshold,
            mapping,
            update_level,
            microphone_initially_muted,
            microphone_invert_led,
        )
        self.dpad_curve = 0
        self.dpad_inc = 0
        self.current_joysticks = DualSenseActionLayer.JoySticks()
        self.prev_joysticks = DualSenseActionLayer.JoySticks()
        self.prev_btns = None
        self.dpad_values = [0.0, 0.0, 0.0, 0.0]  ## up, down, lef, right
        self.prev_dpad_values = [0.0, 0.0, 0.0, 0.0]  ## up, down, lef, right
        self.values_changed: bool = False

        self.on_error(self._error_callback)
        self.activate()

    def _error_callback(self, error):
        self.deactivate()
        raise Exception(f"Opps! an error occured: {error}")

    def _clamp(value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def _update_joystick_values(current_stick: JoyStick, prev_stick: JoyStick, max_trace_vel, DEADZONE):
        xAxis_raw = current_stick.x
        yAxis_raw = current_stick.y

        xAxis = 0.0 if abs(xAxis_raw) <= DEADZONE else xAxis_raw
        yAxis = 0.0 if abs(yAxis_raw) <= DEADZONE else yAxis_raw

        changed = False

        if (xAxis != prev_stick.x) or (yAxis != prev_stick.y) or (prev_stick.btn != current_stick.btn):

            changed = True
            prev_stick.x = xAxis
            prev_stick.y = yAxis
            prev_stick.btn = current_stick.btn

        current_stick.x = max_trace_vel * DualSenseActionLayer._clamp(xAxis, -1.0, 1.0)
        current_stick.y = max_trace_vel * DualSenseActionLayer._clamp(yAxis, -1.0, 1.0)

        return changed

    def dpad_pressed(self):
        return self.btn_up.pressed | self.btn_down.pressed | self.btn_left.pressed | self.btn_right.pressed

    def _update_dpad_values(self, min_val=0.1, max_val=0.5):

        self.dpad_values = [0.0, 0.0, 0.0, 0.0]
        if not self.dpad_pressed():
            self.dpad_curve = 0
            self.dpad_inc = 0
            return

        self.dpad_curve += 0.01
        self.dpad_inc += 0.001 if self.dpad_curve > 1 else 0
        self.dpad_inc = max(self.dpad_inc, min_val)
        self.dpad_inc = min(self.dpad_inc, max_val)

        self.dpad_values[0] = round(self.btn_up.pressed * self.dpad_inc, 3)
        self.dpad_values[1] = round(self.btn_down.pressed * self.dpad_inc, 3)
        self.dpad_values[2] = round(self.btn_left.pressed * self.dpad_inc, 3)
        self.dpad_values[3] = round(self.btn_right.pressed * self.dpad_inc, 3)

    def poll(self, max_trace_vel=0.5, DEADZONE=0.15):

        self.current_joysticks.left.x = self.left_stick.value.x
        self.current_joysticks.left.y = self.left_stick.value.y
        self.current_joysticks.left.btn = self.btn_l3.pressed

        self.current_joysticks.right.x = self.right_stick.value.x
        self.current_joysticks.right.y = self.right_stick.value.y
        self.current_joysticks.right.btn = self.btn_r3.pressed

        changedL = DualSenseActionLayer._update_joystick_values(
            self.current_joysticks.left, self.prev_joysticks.left, max_trace_vel, DEADZONE
        )
        changedR = DualSenseActionLayer._update_joystick_values(
            self.current_joysticks.right, self.prev_joysticks.right, max_trace_vel, DEADZONE
        )

        self._update_dpad_values()
        self.values_changed = (self.prev_dpad_values != self.dpad_values) or (changedL | changedR)
        self.prev_dpad_values = self.dpad_values
        return self.values_changed

from numpy import pi
from time import sleep
import odrive
from odrive.enums import *

class HapticDevice():
    def __init__(self):
        self.odrive = odrive.find_any()
        self.arm0 = ArmSegment()
        self.arm1 = ArmSegment()
    def calibrate(self, axes=[0,1]):
        if 0 in axes:
            print('Calibrating axis 0')
            self.odrive.axis0.requested_state = \
                AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while self.odrive.axis0.current_state != AXIS_STATE_IDLE:
                sleep(.1)
        if 1 in axes:
            print('Calibrating axis 1')
            self.odrive.axis1.requested_state = \
                AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while self.odrive.axis1.current_state != AXIS_STATE_IDLE:
                sleep(.1)
    def home(self):
        input("Move arm to first position and press enter to continue")
        self.arm0.zero = self.odrive.axis0.encoder.shadow_count
        self.arm1.zero = self.odrive.axis1.encoder.shadow_count
        input("Move arm 30 deg left and press enter to continue")
        cal0 = self.odrive.axis0.encoder.shadow_count
        cal1 = self.odrive.axis1.encoder.shadow_count
        theta_dif = 30 * pi / 180
        cnt0_dif = cal0 - self.arm0.zero
        cnt1_dif = cal1 - self.arm1.zero
        self.arm0.cnt_per_rad = cnt0_dif / theta_dif
        self.arm1.cnt_per_rad = cnt1_dif / theta_dif
    def set_axis_state_closed_loop(self, axes=[0,1]):
        if 0 in axes:
            if (self.odrive.axis0.motor.is_calibrated and
                self.odrive.axis0.encoder.is_ready):
                self.odrive.axis0.requested_state = \
                    AXIS_STATE_CLOSED_LOOP_CONTROL
            else:
                raise Exception('Cannot set closed loop control for axis 0 if'
                                + ' it has not been calibrated')
        if 1 in axes:
            if (self.odrive.axis1.motor.is_calibrated and
                self.odrive.axis1.encoder.is_ready):
                self.odrive.axis1.requested_state = \
                    AXIS_STATE_CLOSED_LOOP_CONTROL
            else:
                raise Exception('Cannot set closed loop control for axis 1 if'
                                + ' it has not been calibrated')
    def set_ctrl_mode_torque(self, axes=[0,1]):
        if 0 in axes:
            self.odrive.axis0.controller.config.control_mode = \
                CONTROL_MODE_TORQUE_CONTROL
        if 1 in axes:
            self.odrive.axis1.controller.config.control_mode = \
                CONTROL_MODE_TORQUE_CONTROL
        self.set_axis_state_closed_loop(axes)
    def set_ctrl_mode_position(self, axes=[0,1]):
        if 0 in axes:
            self.odrive.axis0.controller.config.control_mode = \
                CONTROL_MODE_POSITION_CONTROL
        if 1 in axes:
            self.odrive.axis1.controller.config.control_mode = \
                CONTROL_MODE_POSITION_CONTROL
        self.set_axis_state_closed_loop(axes)
    def set_ctrl_mode_velocity(self, axes=[0,1]):
        if 0 in axes:
            self.odrive.axis0.controller.config.control_mode = \
                CONTROL_MODE_VELOCITY_CONTROL
        if 1 in axes:
            self.odrive.axis1.controller.config.control_mode = \
                CONTROL_MODE_VELOCITY_CONTROL
        self.set_axis_state_closed_loop(axes)
    def hold(self, axes=[0,1]):
        self.set_ctrl_mode_position(axes)
        if 0 in axes:
            self.odrive.axis0.controller.input_pos = \
                self.odrive.axis0.encoder.shadow_count
        if 1 in axes:
            self.odrive.axis1.controller.input_pos = \
                self.odrive.axis1.encoder.shadow_count
    def restart(self):
        self.odrive.reboot()

class ArmSegment():
    def __init__(self, length=200):
        self.length = length
        self.zero = None
        self.cnt_per_rad = None

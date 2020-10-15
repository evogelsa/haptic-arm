import odrive
from odrive.enums import *

class HapticDevice():
    def __init__(self):
        self.odrive = odrive.find_any()
        self.arm0 = ArmSegment()
        self.arm1 = ArmSegment()
    def calibrate(self, axes=[0,1]):
        if 0 in axes:
            self.odrive.axis0.requested_state = \
                AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        if 1 in axes:
            self.odrive.axis0.requested_state = \
                AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    def home(self):
        pass
    def set_ctrl_mode_torque(self, axes=[0,1]):
        if 0 in axes:
            self.odrive.axis0.controller.config.control_mode = \
                CONTROL_MODE_TORQUE_CONTROL
            if (self.odrive.axis0.motor.is_calibrated and
                self.odrive.axis0.encoder.is_ready):
                self.odrive.axis0.controller.requested_state = \
                    AXIS_STATE_CLOSED_LOOP_CONTROL
            else:
                raise Exception('Cannot set closed loop control for axis 0 if'
                                + ' it has not been calibrated')
        if 1 in axes:
            self.odrive.axis1.controller.config.control_mode = \
                CONTROL_MODE_TORQUE_CONTROL
            if (self.odrive.axis1.motor.is_calibrated and
                self.odrive.axis1.encoder.is_ready):
                self.odrive.axis1.controller.requested_state = \
                    AXIS_STATE_CLOSED_LOOP_CONTROL
            else:
                raise Exception('Cannot set closed loop control for axis 1 if'
                                + ' it has not been calibrated')

class ArmSegment():
    def __init__(self, length=200):
        self.length = length
        # store calibration info here

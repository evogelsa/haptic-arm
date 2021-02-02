from numpy import pi, radians
from time import sleep
import odrive
from odrive.enums import *

class HapticDevice():
    '''
    Wrapper for the SCARA arm and its associated utility functions. Stores
    objects for each arm segment and the odrive controller
    '''
    def __init__(self, init_with_device=True, arm_length=0.2):
        self.odrive = odrive.find_any() if init_with_device else None
        self.arm0 = ArmSegment(arm_length)
        self.arm1 = ArmSegment(arm_length)
    def calibrate(self, axes=[0,1]):
        '''
        Run the odrive axis/encoder calibration routinen on the specified axes
        '''
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
        '''
        Run the homing routine to calculate zero position and equivalent encoder
        counts to radians/degrees. In the future can be automated with endstops
        '''

        ## TODO CHANGE TO X POS DOWN AND Y POS RIGHT

        # move to zero position and record
        input("Move arm to first position and press enter to continue")
        self.arm0.zero = self.odrive.axis0.encoder.shadow_count
        self.arm1.zero = self.odrive.axis1.encoder.shadow_count
        print(self.arm0.zero, self.arm1.zero)

        # move to defined location and record
        input("Move arm 90 deg counterclockwise and press enter to continue")
        cal0 = self.odrive.axis0.encoder.shadow_count
        cal1 = self.odrive.axis1.encoder.shadow_count
        print(cal0, cal1)

        # calculate the difference in encoder counts and define conversion ratio
        # to radians
        theta_dif = radians(90)
        cnt0_dif = cal0 - self.arm0.zero
        cnt1_dif = cal1 - self.arm1.zero
        print(cnt0_dif, cnt1_dif)

        self.arm0.cnt_per_rad = cnt0_dif / theta_dif
        self.arm1.cnt_per_rad = cnt1_dif / theta_dif
        print(self.arm0.cnt_per_rad, self.arm1.cnt_per_rad)
    def set_axis_state_closed_loop(self, axes=[0,1]):
        '''
        Set the specified axes to closed loop control mode, requires
        calibration. Automatically called when setting control modes
        '''
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
        '''
        Set the specified axes to torque control mode and subsequently set to
        closed loop control
        '''
        if 0 in axes:
            self.odrive.axis0.controller.config.control_mode = \
                CONTROL_MODE_TORQUE_CONTROL
        if 1 in axes:
            self.odrive.axis1.controller.config.control_mode = \
                CONTROL_MODE_TORQUE_CONTROL
        self.set_axis_state_closed_loop(axes)
    def set_ctrl_mode_position(self, axes=[0,1]):
        '''
        Set the specified axes to position control mode and subsequently set to
        closed loop control
        '''
        if 0 in axes:
            self.odrive.axis0.controller.config.control_mode = \
                CONTROL_MODE_POSITION_CONTROL
        if 1 in axes:
            self.odrive.axis1.controller.config.control_mode = \
                CONTROL_MODE_POSITION_CONTROL
        self.set_axis_state_closed_loop(axes)
    def set_ctrl_mode_velocity(self, axes=[0,1]):
        '''
        Set the specified axes to velocity control mode and subsequently set to
        closed loop control
        '''
        if 0 in axes:
            self.odrive.axis0.controller.config.control_mode = \
                CONTROL_MODE_VELOCITY_CONTROL
        if 1 in axes:
            self.odrive.axis1.controller.config.control_mode = \
                CONTROL_MODE_VELOCITY_CONTROL
        self.set_axis_state_closed_loop(axes)
    def hold(self, axes=[0,1]):
        '''
        Changes control mode to position and makes motors hold current position
        '''
        self.set_ctrl_mode_position(axes)
        if 0 in axes:
            self.odrive.axis0.controller.input_pos = \
                self.odrive.axis0.encoder.shadow_count
        if 1 in axes:
            self.odrive.axis1.controller.input_pos = \
                self.odrive.axis1.encoder.shadow_count
    def restart(self):
        '''
        Reboots the associated odrive controller
        '''
        self.odrive.reboot()

class ArmSegment():
    '''
    Wrapper for data associated to each arm segment of the parent device.
    '''
    def __init__(self, length=0.2):
        self.length = length
        self.zero = None
        self.cnt_per_rad = None

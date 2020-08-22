import calculate
from time import sleep
import numpy as np
import odrive
from odrive.enums import *


# TODO
# 1. test if velocity control works
#   a. if velocity doesnt work convert everything to current control
# 2. discuss different vector funcs, method correct?

def _convert_text(var):
    """
    Converts provided variable to integer form if its a string
    """
    if type(var) is str:
        if var.upper() == 'BOTH':
            var = 2
        elif var.upper() == 'TWO':
            var = 2
        elif var.upper() == 'ONE':
            var = 1
        elif var.upper() == 'ZERO':
            var = 0
        elif var.upper() == 'NONE':
            var = -1
    return var

def clear_error(odrv):
    """
    Clear all errors on the provided axes. If not passed odrive it will attempt
    to connect to odrive by itself. This is slow. Returns 0 on success.
    """
    try:
        odrv.axis0.error = 0
        odrv.axis1.error = 0
        odrv.axis0.motor.error = 0
        odrv.axis1.motor.error = 0
        odrv.axis0.encoder.error = 0
        odrv.axis1.encoder.error = 0

        return 0
    except:
        return -1

def calibrate(odrv, axes):
    """
    Runs odrive calibration sequences on specified axes
    """
    axes = _convert_text(axes)

    if axes == 2:
        # Perform the calibration until completed
        odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while odrv.axis0.current_state != AXIS_STATE_IDLE:
            sleep(.1)
        odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while odrv.axis1.current_state != AXIS_STATE_IDLE:
            sleep(.1)
    elif axes == 1:
        odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while odrv.axis1.current_state != AXIS_STATE_IDLE:
            sleep(.1)
    elif axes == 0:
        odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while odrv.axis0.current_state != AXIS_STATE_IDLE:
            sleep(.1)
    elif axes == -1:
        return
    else:
        raise Exception("Error with specified axes for calibration, check"
                        + " arguments")

class haptic_arm():
    """
    Arm class to handle different arm parameters as versions change
    """
    def __init__(self):
        self.length0 = 200 # millimeters
        self.length1 = 200 # millimeters
        self.width0 = 30   # millimeters
        self.width1 = 30   # millimeters
        self.count0_zero = None
        self.count1_zero = None
        self.cnt_per_rad0 = None
        self.cnt_per_rad1 = None
        self.calibrated = False
        self.arm0 = None
        self.arm1 = None
        self.odrv = None

    def calibrate(self):
        """
        Calibrate the counts to radian for arm version
        """
        input("Move Odrive to first position")
        self.count0_zero = self.odrv.axis0.encoder.shadow_count
        self.count1_zero = self.odrv.axis1.encoder.shadow_count
        input("Move Odrive 30 degrees left to second position")
        count0_cal = self.odrv.axis0.encoder.shadow_count
        count1_cal = self.odrv.axis1.encoder.shadow_count
        theta_dif = 30 * np.pi / 180
        count0_dif = count0_cal - self.count0_zero
        count1_dif = count1_cal - self.count1_zero
        self.cnt_per_rad0 = count0_dif / theta_dif
        self.cnt_per_rad1 = count1_dif / theta_dif
        self.calibrated = True

    def setup(self, axes = 2, ctrl_modes = [0,0], calib = 2, remove_errors = True):
        """
        Modular function to connect to odrive device. Parameters specify whether
        or not to run a calibration sequence, what control modes the odrive should
        be initialized in, and how many axes are active.

            -1 | NONE |      : No axes
            0  | ZERO |      : Axis zero only
            1  | ONE  |      : Axis one only
            2  | TWO  | BOTH : Axis one and two

        """

        axes = _convert_text(axes)
        calib = _convert_text(calib)

        # Find and connect to ODrive
        if 2 >= axes >= 0:
            print("Connecting to ODrive")
            odrv = odrive.find_any()
            print("Connected")
        else:
            print("No or incorrect axes specified! ODrive will not be connected")
            return 0

        # Clear errors
        if remove_errors:
            clear_error(odrv)

        # Add axes to list
        if axes == 2:
            Axes = [odrv.axis0, odrv.axis1]
        elif axes == 1:
            Axes = [None, odrv.axis1]
        elif axes == 0:
            Axes = [odrv.axis0, None]
        else:
            Axes = [None, None]

        # calibrate axes if desired, skip going into func if calibration is none
        if 2 >= calib >= -1:
            calibrate(odrv, calib)

        # Set control mode
        # 0: CTRL_MODE_VOLTAGE_CONTROL
        # 1: CTRL_MODE_CURRENT_CONTROL
        # 2: CTRL_MODE_VELOCITY_CONTROL
        # 3: CTRL_MODE_POSITION_CONTROL
        for n in range(2):
            if Axes[n]:
                Axes[n].controller.config.control_mode = ctrl_modes[n]
                Axes[n].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # Check for errors in axes
        if calib == 2:
            for n in range(calib):
                if Axes[n].error:
                    print("Axis%d error: " %(n) + hex(Axes[n].error))
                    if Axes[n].encoder.error:
                        print("Encoder%d error: " %(n) + hex(Axes[n].encoder.error))
                    if Axes[n].motor.error:
                        print("Motor%d error: " %(n) + hex(Axes[n].motor.error))
                    quit()
        elif 0 <= calib <= 1:
            if Axes[calib].error:
                print("Axis%d error: " %(n) + hex(Axes[calib].error))
                if Axes[calib].encoder.error:
                    print("Encoder%d error: " %(n) + \
                          hex(Axes[calib].encoder.error))
                if Axes[calib].motor.error:
                    print("Motor%d error: " %(n) + hex(Axes[calib].motor.error))
                exit()

        self.odrv = odrv


def init():
    # instantiate arm class for version greece
    arm = haptic_arm()

    # initialize odrive in vel control mode
    mode = CTRL_MODE_VELOCITY_CONTROL
    arm.setup('both', [mode, mode], 'both')

    # run arm calibration routine
    arm.calibrate()

    # vectors = calculate.vector_field_t()
    # vectors.select_field_by_user()
    # vectors.select_field_by_args('restrict_circle', radius = .05, buffer = .005,
    #                              center_x = arm.l1, center_y = arm.l1)

    return arm

def step(arm):
    # get dx dy for desired vector field
    X = np.array([[arm.vf.dx], [arm.vf.dy]])

    # use inv jacobian to get dthetas
    dthetas = np.matmul(calculate.inv_jacobian(arm.gfx_seg0, arm.gfx_seg1), X)
    dtheta0 = dthetas[0,0]
    dtheta1 = dthetas[1,0]

    # convert rad/s to count/s
    dcount0, dcount1 = calculate.theta_to_count(arm, dtheta0, dtheta1)

    # vel setpoint in counts/s
    arm.odrv.axis0.controller.vel_setpoint = dcount0
    arm.odrv.axis1.controller.vel_setpoint = dcount1

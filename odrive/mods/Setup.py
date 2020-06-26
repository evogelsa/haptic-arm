import Calculate
import numpy as np
from time import sleep
import odrive
from odrive.enums import *


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


class haptic_arm():
    """
    Arm class to handle different arm parameters as versions change
    """
    def __init__(self, version = 'Greece'):
        version = str(version).upper()
        if version == 'GREECE':
            self.l1 = .1225
            self.l2 = .1225
            self.version = 'GREECE'
            self.count0_zero = None
            self.count1_zero = None
            self.cnt_per_rad0 = None
            self.cnt_per_rad1 = None
            self.calibrated = False

    def calibrate(self, odrv0, version = 'GREECE'):
        """
        Calibrate the counts to radian for arm version
        """
        if version == 'GREECE':
            input("Move Odrive to first position")
            self.count0_zero = Calculate.position(odrv0, 0)
            self.count1_zero = Calculate.position(odrv0, 1)
            input("Move Odrive 30 degrees left to second position")
            count0_cal = Calculate.position(odrv0, 0)
            count1_cal = Calculate.position(odrv0, 1)
            theta_dif = 30 * np.pi / 180
            count0_dif = count0_cal - self.count0_zero
            count1_dif = count1_cal - self.count1_zero
            self.cnt_per_rad0 = count0_dif / theta_dif
            self.cnt_per_rad1 = count1_dif / theta_dif
            self.calibrated = True


def setup(axes = 2, ctrl_modes = [0,0], calib = 2, remove_errors = True):
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
        odrv0 = odrive.find_any()
        print("Connected")
    else:
        print("No or incorrect axes specified! ODrive will not be connected")
        return 0

    # Clear errors
    if remove_errors:
        clear_error(odrv0)

    # Add axes to list
    if axes == 2:
        Axes = [odrv0.axis0, odrv0.axis1]
    elif axes == 1:
        Axes = [None, odrv0.axis1]
    elif axes == 0:
        Axes = [odrv0.axis0, None]
    else:
        Axes = [None, None]

    # calibrate axes if desired, skip going into func if calibration is none
    if calib != -1:
        calibrate(odrv0, calib)

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

    return odrv0


def calibrate(odrv0, axes):
    """
    Runs odrive calibration sequences on specified axes
    """
    axes = _convert_text(axes)

    if axes == 2:
        # Perform the calibration until completed
        odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while odrv0.axis0.current_state != AXIS_STATE_IDLE:
            sleep(.1)
        odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while odrv0.axis1.current_state != AXIS_STATE_IDLE:
            sleep(.1)
    elif axes == 1:
        odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while odrv0.axis1.current_state != AXIS_STATE_IDLE:
            sleep(.1)
    elif axes == 0:
        odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while odrv0.axis0.current_state != AXIS_STATE_IDLE:
            sleep(.1)
    elif axes == -1:
        return
    else:
        raise Exception("Error with specified axes for calibration, check"
                        + " arguments")


def theta_calibrate(odrv0, axes):
    """
    Calibrate the encoder counts to difference in theta using physical
    calibration bracket
    """
    axes = _convert_text(axes)

    print("Reading count configuration for zero state...")
    if axes == 2:
        count0_zero = Calculate.position(odrv0, 0)
        count1_zero = Calculate.position(odrv0, 1)

        print("Move encoders in the positive (counterclockwise) direction by" +
              " 30 degrees to second calibration detent. Calibration will" +
              " resume 5 seconds after the device detects movement.")

        has_moved = False
        while not has_moved:
            temp0 = Calculate.position(odrv0, 0)
            temp1 = Calculate.position(odrv0, 1)

            dif0 = abs(count0_zero - temp0)
            dif1 = abs(count1_zero - temp1)

            if (dif0 > 300) and (dif1 > 300):
                has_moved = True
            else:
                sleep(.005)

        print("Movement has been detected, resuming calibration in 5 seconds")
        sleep(5)
        print("Reading count configuration for 30 degrees of movement...")

        count0_cal = Calculate.position(odrv, 0)
        count1_cal = Calculate.position(odrv, 1)

        theta_dif = 30 * np.pi / 180

        count0_dif = count0_cal - count0_dif
        count1_dif = count1_cal - count1_dif

        cnt_per_rad0 = count0_dif / theta_dif
        cnt_per_rad1 = count1_dif / theta_dif

        print("Calibration complete")

        return cnt_per_rad0, cnt_per_rad1
    else:
        raise Exception("Theta calibration only currently supports both axes")


def tune_pid(odrv0, axis):
    """
    Plan is to integrate the pid calibration assistant into this function at
    some point but its low priority
    """
    pass


def clear_error(odrv0):
    """
    Clear all errors on the provided axes. If not passed odrive it will attempt
    to connect to odrive by itself. This is slow. Returns 0 on success.
    """
    try:
        odrv0.axis0.error = 0
        odrv0.axis1.error = 0
        odrv0.axis0.motor.error = 0
        odrv0.axis1.motor.error = 0
        odrv0.axis0.encoder.error = 0
        odrv0.axis1.encoder.error = 0

        return 0
    except:
        return -1

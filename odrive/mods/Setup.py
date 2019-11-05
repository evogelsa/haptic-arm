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


class arm():
    """
    Arm class to handle different arm parameters as versions change
    """
    def __init__(self, version = 'Greece'):
        version = str(version).upper()
        if version == 'GREECE':
            self.l1 = .1225
            self.l2 = .1225
            self.version = 'GREECE'
        else:
            self.l1 = .1225
            self.l2 = .1225
            self.version = 'GREECE'


def setup(axes = 2, ctrl_modes = [0,0], calib = 2):
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
                exit()
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
    Runs calibration sequences on specified axes
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


def tune_pid(odrv0, axis):
    """
    Plan is to integrate the pid calibration assistant into this function at
    some point but its low priority
    """
    pass


def clear_error(odrv0, axes):
    """
    Clear all errors on the provided axes
    """
    axes = _convert_text(axes)

    if axes == 2:
        odrv0.axis0.error = 0
        odrv0.axis1.error = 0
        odrv0.axis0.motor.error = 0
        odrv0.axis1.motor.error = 0
        odrv0.axis0.encoder.error = 0
        odrv0.axis1.encoder.error = 0
    elif axes == 1:
        odrv0.axis1.error = 0
        odrv0.axis1.motor.error = 0
        odrv0.axis1.encoder.error = 0
    elif axes == 0:
        odrv0.axis0.error = 0
        odrv0.axis0.motor.error = 0
        odrv0.axis0.encoder.error = 0
    else:
        raise Exception("Error with clearing errors, check specified axes")

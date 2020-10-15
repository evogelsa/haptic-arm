import device

import numpy as np


# instantiate a haptic device (connects to odrive)
arm = device.HapticDevice()
# calibrate encoders with odrive calibration routine
arm.calibrate()
# use custom homing routine to define counts/angle
arm.home()
# setup control mode
arm.set_ctrl_mode_torque()

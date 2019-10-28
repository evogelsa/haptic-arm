import Setup
import Calculate
import time
import numpy as np
# import odrive
from odrive.enums import *


# connect to odrive and setup for voltage control
mode = CTRL_MODE_VOLTAGE_CONTROL
odrv0 = Setup.setup('both', [mode, mode], 'none')
Axes = [odrv0.axis0, odrv0.axis1]

# set arm length constants
L1 = 250
L2 = 250

# set initial theta
theta_init1 = Calculate.position(odrv0, 0)
theta_init2 = Calculate.position(odrv0, 1)

# set initial cartesian coords


def j_inv(thetas):
    # create jacobian
    J = np.zeros((2,2))
    J[0,0] = -L1 * np.sin(theta[0])
    J[0,1] = -L2 * np.sin(theta[1])
    J[1,0] = L1 * np.cos(theta[0])
    J[1,1] = L1 * np.cos(theta[1])
    # inverse jacobian
    return np.linalg.inv(J)

def update_thetas():
    theta1 = Calculate.position(odrv0, 0) - theta_init1
    theta2 = Calculate.position(odrv0, 1) - theta_init2

    return theta1, theta2

def update_cartesian():
    pass

while True:
    thetas = update_thetas()

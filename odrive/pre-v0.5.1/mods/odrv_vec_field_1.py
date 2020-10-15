import Setup
import Calculate
from time import sleep
import numpy as np
from odrive.enums import *


# TODO
# 1. test if velocity control works
#   a. if velocity doesnt work convert everything to current control
# 2. discuss different vector funcs, method correct?


# instantiate arm class for version greece
arm = Setup.haptic_arm('greece')
l1 = arm.l1
l2 = arm.l2

# initialize odrive in pos control mode
mode = CTRL_MODE_VELOCITY_CONTROL
odrv0 = Setup.setup('both', [mode, mode], 'both')

# run arm calibration routine
arm.calibrate(odrv0)

vectors = Calculate.vector_fields()
vectors.select_field_by_user()
# vectors.select_field_by_args('restrict_circle', radius = .05, buffer = .005,
#                              center_x = l1, center_y = l1)

# control loop
while True:
    # get current config in counts
    count0 = Calculate.position(odrv0, 0)
    count1 = Calculate.position(odrv0, 1)

    # convert counts to radians
    theta0, theta1 = Calculate.count2theta(count0, count1, arm)

    # get end pos with kinematic equations
    x, y = Calculate.fwd_kinematics(theta0, theta1, arm)

    # calculate x_dot y_dot for desired vector field
    x_dot, y_dot = vectors.return_field(x, y)
    X = np.array([[x_dot], [y_dot]])

    # use inv jacobian to get theta dots
    theta_dots = np.matmul(Calculate.inv_jacobian(theta0, theta1, arm), X)
    theta0_dot = theta_dots[0,0]
    theta1_dot = theta_dots[1,0]

    # convert rad/s to count/s
    count0_dot, count1_dot = Calculate.theta2count(theta0_dot, theta1_dot, arm)

    # vel setpoint in counts/s
    odrv0.axis0.controller.vel_setpoint = count0_dot
    odrv0.axis1.controller.vel_setpoint = count1_dot

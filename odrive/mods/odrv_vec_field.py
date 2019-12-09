import Setup
import Calculate
from time import sleep
import numpy as np
from odrive.enums import *


# instantiate arm class for version greece
arm = Setup.haptic_arm('greece')
l1 = arm.l1
l2 = arm.l2

# initialize odrive in pos control mode
mode = CTRL_MODE_CURRENT_CONTROL
odrv0 = Setup.setup('both', [mode, mode], 'both')

# get zero pos
input("Move ODrive to \"zero\" position and press enter")

count0_zero = Calculate.position(odrv0, 0)
count1_zero = Calculate.position(odrv0, 1)

# get cal pos
input("Move ODrive to \"calibration\" position and press enter")

count0_cal = Calculate.position(odrv0, 0)
count1_cal = Calculate.position(odrv0, 1)

# get angle dif
theta0_dif = float(input("Input change in angle1 (degrees): ")) * np.pi / 180
theta1_dif = float(input("Input change in angle2 (degrees): ")) * np.pi / 180

# get dif in pos
count0_dif = count0_cal - count0_zero
count1_dif = count1_cal - count1_zero

# print different in counts
print(count0_dif)
print(count1_dif)

# calculate counts per radian
cnt_per_rad0 = count0_dif / theta0_dif
cnt_per_rad1 = count1_dif / theta1_dif

# print the counts per rad val
print(cnt_per_rad0)
print(cnt_per_rad1)

def count2theta(count0, count1):
    theta0 = (count0 - count0_zero) / cnt_per_rad0
    theta1 = (count1 - count1_zero) / cnt_per_rad1

    return theta0, theta1

def theta2count(theta0, theta1):
    count0 = theta0 * cnt_per_rad0 + count0_zero
    count1 = theta1 * cnt_per_rad1 + count1_zero

    return count0, count1

def fwd_kinematics(theta0, theta1):
    x = l1*np.cos(theta0) + l2*np.cos(theta1)
    y = l1*np.sin(theta0) + l2*np.sin(theta1)

    return x, y

def jacobian(theta0, theta1):
    # return array of fwd kinematic eqns
    return np.array([[-l1 * np.sin(theta0), -l2 * np.sin(theta1)],
                  [l1 * np.cos(theta0), l2 * np.cos(theta1)]])

def inv_jacobian(theta0, theta1):
    # return inverse of jacobian
    return np.linalg.inv(jacobian(theta0, theta1))

class vector_field_funcs():
    def __init__(self):
        self.fields = {
            'line_y': self._line_y,
            'circle': self._circle
        }

    def return_field(self, field, x, y):
        return self.fields[field](x,y)

    def _line_y(self, x, y):
        x_dot = 0
        # function of y maps y displacement to y_dot range [0 cm/s, 3 cm/s]
        y_dot = ((y) / (l1 + l2)) * .03

        return x_dot, y_dot

    def _circle(self, x, y, radius = .05, buffer = .005):
        # make circle w/ 5cm radius but give +- 1cm buffer zone
        outter = radius + buffer
        inner = radius - buffer
        # give circle center of arm lengths
        center_x = l1
        center_y = l1

        # vel is value mapped linearly with displacement from center of the
        # circle to range of 0 cm/s to 3 cm/s
        # (map pos to center) / (max val) * (max vel)
        if ((x - center_x) > outter):
            x_dot = ((x - center_x) / (l1 + l2 - center_x)) * .03
        elif ((x - center_x) < inner):
            x_dot = ((x - center_x) / (l1 + l2 - center_x)) * .03
        else:
            x_dot = 0

        if ((y - center_y) > outter):
            y_dot = ((y - center_y) / (l1 + l2 - center_y)) * .03
        elif ((y - center_y) < inner):
            y_dot = ((y - center_y) / (l1 + l2 - center_y)) * .03
        else:
            y_dot = 0

        return x_dot, y_dot

vectors = vector_field_funcs()

# get user to select vector field
print("Select vector field")
for key in vectors.fields.keys():
    print(key)
valid = False
while not valid:
    field = str(input("Selection: "))
    if field not in vectors.fields.keys():
        print("Invalid key")
    else:
        valid = True

# control loop
while True:
    # get current config in counts
    count0 = Calculate.position(odrv0, 0)
    count1 = Calculate.position(odrv0, 1)

    # convert counts to radians
    theta0, theta1 = count2theta(count0, count1)

    # get end pos with kinematic equations
    x, y = fwd_kinematics(theta0, theta1)

    # calculate x_dot y_dot for desired vector field
    x_dot, y_dot = vectors.return_field(field, x, y)
    X = np.array([[x_dot], [y_dot]])

    # use inv jacobian to get theta dots
    theta0_dot, theta1_dot = np.matmul(inv_jacobian(theta0, theta1), X)
    theta0_dot = theta0_dot[0]
    theta1_dot = theta1_dot[0]

    # convert rad/s to count/s
    count0_dot, count1_dot = theta2count(theta0_dot, theta1_dot)

    # vel setpoint in counts/s
    odrv0.axis0.controller.vel_setpoint = count0_dot
    odrv0.axis1.controller.vel_setpoint = count1_dot

import Setup
import Calculate
from time import sleep
import numpy as np
from odrive.enums import *


# instantiate arm class for version greece
arm = Setup.arm('greece')
l1 = arm.l1
l2 = arm.l2

# initialize odrive in pos control mode
mode = CTRL_MODE_POSITION_CONTROL
odrv0 = Setup.setup('both', [mode, mode], 'none')

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
    theta0 = count0 / cnt_per_rad0
    theta1 = count1 / cnt_per_rad1

    return theta0, theta1

def theta2count(theta0, theta1):
    count0 = theta0 * cnt_per_rad0
    count1 = theta1 * cnt_per_rad1

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

def circle_wp(center = [l1, -l2], radius = .05, num_wp = 10):
    x = center[0]
    y = center[1]
    phi = np.linspace(0, 2 * np.pi, num_wp)
    wps = np.zeros((num_wp, 2))
    for i, p in enumerate(phi):
        wps[i, 0] = x + radius * np.cos(p)
        wps[i, 1] = y + radius * np.sin(p)

    return wps

waypoints = circle_wp()

cur_wp = 0

while True:
    # get current config in counts
    count0 = Calculate.position(odrv0, 0)
    count1 = Calculate.position(odrv0, 1)

    # convert counts to radians
    theta0, theta1 = count2theta(count0, count1)

    # get end pos with kinematic equations
    x, y = fwd_kinematics(theta0, theta1)

    # calculate change in and and y to get to waypoint
    delta_x, delta_y = waypoints[cur_wp] - np.array([x, y])

    # calculate change in theta from change in x and y
    delta_theta0, delta_theta1 = (inv_jacobian(theta0, theta1) *
                                  np.array([delta_x, delta_y]))

    # get new thetas to move to
    n_theta0 = theta0 + delta_theta0
    n_theta1 = theta1 + delta_theta1

    # get new counts from new thetas
    n_count0, n_count1 = theta2count(n_theta0, n_theta1)

    # set odrive to move to new encoder counts
    odrv0.axis0.controller.pos_setpoint = n_count0
    odrv0.axis1.controller.pos_setpoint = n_count1

    # set to not in range to move to next wp yet
    in_range = False

    # start loop to check if we are close enough to move to next wp yet
    while not in_range:
        # get current config in counts
        count0 = Calculate.position(odrv0, 0)
        count1 = Calculate.position(odrv0, 1)

        # convert config in counts to rad
        theta0, theta1 = count2theta(count0, count1)

        # get x and y from thetas
        x, y = fwd_kinematics(theta0, theta1)

        # check if current position is within margin of error for desired wp
        if ((waypoints[cur_wp, 0] - x < .005) and
            (waypoints[cur_wp, 1] - y < .005)):
            # if it is set in range to true
            in_range = True
        else:
            # otherwise just wait a little bit so we dont do too many calcs
            sleep(.005)

    # increment so we move to next wp
    cur_wp += 1
    # modular math wp so we roll back and stay within num of waypoints
    cur_wp %= len(waypoints)

import Setup
import Calculate
from time import sleep
import numpy as np
from odrive.enums import *

N_WP = 100

# instantiate arm class for version greece
arm = Setup.haptic_arm('greece')
l1 = arm.l1
l2 = arm.l2

# initialize odrive in pos control mode
mode = CTRL_MODE_POSITION_CONTROL
odrv0 = Setup.setup('both', [mode, mode], 'both')

arm.calibrate(odrv0)

input('Press enter when ready')

def circle_wp(center = [l1, -l2], radius = .05, num_wp = N_WP):
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
    theta0, theta1 = Calculate.count2theta(count0, count1, arm)

    # get end pos with kinematic equations
    x, y = Calculate.fwd_kinematics(theta0, theta1, arm)

    # calculate change in and x and y to get to waypoint
    delta_x, delta_y = waypoints[cur_wp] - np.array([x, y])

    # calculate change in theta from change in x and y
    delta_thetas = np.matmul(Calculate.inv_jacobian(theta0, theta1, arm),
                             np.array([[delta_x], [delta_y]]))
    delta_theta0 = delta_thetas[0,0]
    delta_theta1 = delta_thetas[1,0]

    # get new thetas to move to
    n_theta0 = theta0 + delta_theta0
    n_theta1 = theta1 + delta_theta1

    # get new counts from new thetas
    n_count0, n_count1 = Calculate.theta2count(n_theta0, n_theta1, arm)

    # set odrive to move to new encoder counts
    odrv0.axis0.controller.pos_setpoint = n_count0
    odrv0.axis1.controller.pos_setpoint = n_count1

    # set to not in range to move to next wp yet
    in_range = False

    # start loop to check if we are close enough to move to next wp yet
    while not in_range:
        print("Current pos: (%f, %f) | Next WP: (%f, %f)" %(x, y,
               waypoints[cur_wp, 0], waypoints[cur_wp, 1]))

        # get current config in counts
        count0 = Calculate.position(odrv0, 0)
        count1 = Calculate.position(odrv0, 1)

        # convert config in counts to rad
        theta0, theta1 = Calculate.count2theta(count0, count1, arm)

        # get x and y from thetas
        x, y = Calculate.fwd_kinematics(theta0, theta1, arm)

        # check if current position is within margin of error for desired wp
        if ((abs(waypoints[cur_wp, 0] - x) < .015) and
            (abs(waypoints[cur_wp, 1] - y) < .015)):
            # if it is set in range to true
            in_range = True
            sleep(2 / N_WP)
        else:
            # otherwise just wait a little bit so we dont do too many calcs
            sleep(.005)

    # increment so we move to next wp
    cur_wp += 1
    # modular math wp so we roll back and stay within num of waypoints
    cur_wp %= len(waypoints)

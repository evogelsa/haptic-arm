import device
import calculate
import visualize
from sys import exit
import numpy as np


MAX_VEL0 = .5
MAX_VEL1 = .5

def step(arm, vf, vis=None):
    # get current arm configuration in encoder counts
    count0 = arm.odrive.axis0.encoder.shadow_count
    count1 = arm.odrive.axis1.encoder.shadow_count

    # convert counts to radians
    theta0 = calculate.count2rad(arm, count0, 0)
    theta1 = calculate.count2rad(arm, count1, 1)

    # get position of end effector with kinematic equations [units of meters]
    x, y = calculate.fwd_kinematics(arm, theta0, theta1)

    # get desired vectors from defined vector field [units of meters/s]
    dx, dy = vf.return_vectors(x, y)

    # create matrix of vectors
    vectors = np.array([[dx], [dy]])

    # calculate angular velocities with inv jacobian and matrix multiplication
    # [units of rad/s]
    jac = calculate.inv_jacobian(arm, theta0, theta1)
    thetas = np.matmul(jac, vectors)
    dtheta0 = thetas[0, 0]
    dtheta1 = thetas[1, 0]

    # convert rad/s to counts/s
    dcount0 = calculate.rad2count(arm, dtheta0, 0)
    dcount1 = calculate.rad2count(arm, dtheta1, 1)

    # convert counts/s to turns/s
    vel0 = dcount0 / arm.odrive.axis0.encoder.config.cpr
    vel1 = dcount1 / arm.odrive.axis1.encoder.config.cpr

    # limit velocities to set maximums to avoid overspeed errors
    vel0 = min(vel0, MAX_VEL0)
    vel1 = min(vel1, MAX_VEL1)
    vel0 = max(vel0, -MAX_VEL0)
    vel1 = max(vel1, -MAX_VEL1)

    # arm.odrive.axis0.controller.input_vel = vel0
    arm.odrive.axis0.controller.input_torque = vel0

    # arm.odrive.axis1.controller.input_vel = vel1
    arm.odrive.axis1.controller.input_torque = vel1

    if vis is not None:
        line1 = "count0 : {:9d} | count1 : {:9d}".format(count0, count1)
        line2 = "theta0 : {:9.2f} | theta1 : {:9.2f}".format(theta0, theta1)
        line3 = "x      : {:9.2f} | y      : {:9.2f}".format(x, y)
        line4 = "dx     : {:9.2f} | dy     : {:9.2f}".format(dx, dy)
        line5 = "dtheta0: {:9.2f} | dtheta1: {:9.2f}".format(dtheta0, dtheta1)
        line6 = "dcount0: {:9.2f} | dcount1: {:9.2f}".format(dcount0, dcount1)
        line7 = "vel0   : {:9.2f} | vel1   : {:9.2f}".format(vel0, vel1)
        paragraph = [[line1],
                     [line2],
                     [line3],
                     [line4],
                     [line5],
                     [line6],
                     [line7]]
        vis.text(paragraph, size=16)
        vis.update_device(theta0, theta1)
        vis.step()

def stop(arm):
    try:
        arm.restart()
    except:
        print('Device restarted')

def main():
    # instantiate a haptic device (connects to odrive)
    arm = device.HapticDevice()

    # redefine max velocities
    global MAX_VEL0, MAX_VEL1
    MAX_VEL0 = arm.odrive.axis0.controller.config.vel_limit / 4
    MAX_VEL1 = arm.odrive.axis1.controller.config.vel_limit / 4

    # calibrate encoders with odrive calibration routine
    arm.odrive.axis0.clear_errors()
    arm.odrive.axis1.clear_errors()
    arm.calibrate()

    # use custom homing routine to define counts/angle
    arm.home()

    input('press enter to continue...')

    # setup control mode
    arm.set_ctrl_mode_torque()

    # instantiate vector field for arm
    vf_args = {
            'xcenter': np.sqrt(2)*arm.arm0.length,
            'ycenter': 0,
            'dtheta' : .5,
            'radius' : arm.arm0.length/4,
            'buffer' : arm.arm0.length/4 * .05,
            'drmax'  : .5,
            }
    vf = calculate.VectorField(arm, field='spring', args=vf_args)

    # create visualization window
    vis = visualize.SDLWrapper()
    # and generate the arm segments
    vis.generate_device(arm)

    try:
        running = True
        while running:
            step(arm, vf, vis)
            running = visualize.check_running()
        stop(arm)
    except KeyboardInterrupt:
        stop(arm)

if __name__ == "__main__":
    exit(main())

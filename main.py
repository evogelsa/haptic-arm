import device
import calculate
import visualize

from argparse import ArgumentParser
from sys import exit, argv
import numpy as np


MAX_VEL0 = 4
MAX_VEL1 = 4

def step(arm, vf, vis=None):
    # get current arm configuration
    theta0, theta1 = arm.get_config()

    # get position of end effector with kinematic equations [units of meters]
    x, y = arm.fwd_kinematics(theta0, theta1)

    # get desired vectors from defined vector field [units of meters/s]
    dx, dy = vf.return_vectors(x, y)

    # create matrix of vectors
    vectors = np.array([dx, dy])

    # calculate angular velocities with inv jacobian and matrix multiplication
    # [units of rad/s]
    ijac = arm.inv_jacobian(theta0, theta1)
    thetas = ijac @ vectors
    dtheta0 = thetas[0]
    dtheta1 = thetas[1]

    # convert rad/s to counts/s
    dcount0 = arm.rad2count(dtheta0, 0)
    dcount1 = arm.rad2count(dtheta1, 1)

    # convert counts/s to turns/s
    vel0 = dcount0 / arm.odrive.axis0.encoder.config.cpr
    vel1 = dcount1 / arm.odrive.axis1.encoder.config.cpr

    # limit velocities to set maximums to avoid overspeed errors
    vel0 = min(vel0, MAX_VEL0)
    vel1 = min(vel1, MAX_VEL1)
    vel0 = max(vel0, -MAX_VEL0)
    vel1 = max(vel1, -MAX_VEL1)

    arm.odrive.axis0.controller.input_vel = vel0
    #  arm.odrive.axis0.controller.input_torque = vel0

    arm.odrive.axis1.controller.input_vel = vel1
    #  arm.odrive.axis1.controller.input_torque = vel1

    if vis is not None:
        count0 = arm.rad2count(theta0)
        count1 = arm.rad2count(theta1)
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
    arm.set_ctrl_mode_velocity()

    # parse cl arguments
    parser = ArgumentParser(description='Parse field parameters')
    parser.add_argument('-x', '--xcenter', dest='xcenter', type=float,
                        default=np.sqrt(2)*arm.arm0.length,
                        help='center of vector field in x direction')
    parser.add_argument('-y', '--ycenter', dest='ycenter', type=float,
                        default=0,
                        help='center of vector field in y direction')
    parser.add_argument('-dt', '--dtheta', dest='dtheta', type=float,
                        default=0.5,
                        help='rotational speed of vector field')
    parser.add_argument('-r', '--radius',  dest='radius', type=float,
                        default=arm.arm0.length/4,
                        help='radius from field center for deadband')
    parser.add_argument('-b', '--buffer',  dest='buffer', type=float,
                        default=arm.arm0.length/4 * .05,
                        help='size of deadband')
    parser.add_argument('-dr', '--drmax',  dest='drmax', type=float,
                        default=0.5,
                        help='the max radial speed towards center of field')
    parser.add_argument('-f', '--field',   dest='field', type=str,
                        default='spiralbound',
                        help='field type: circle, circlebound, spiral, ' +
                        'spiralbound, spring')
    args = parser.parse_args()

    # instantiate vector field for arm
    vf_args = {
            'xcenter': args.xcenter,
            'ycenter': args.ycenter,
            'dtheta' : args.dtheta,
            'radius' : args.radius,
            'buffer' : args.buffer,
            'drmax'  : args.drmax,
            }
    field=args.field
    vf = calculate.VectorField(arm, field=field, args=vf_args)

    # create visualization window
    vis = visualize.SDLWrapper()
    # and generate the arm segments
    vis.generate_device(arm)
    # and draw the vector field
    vis.vector_stream_plot(vf, arm)

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

import device
import calculate
import numpy as np

def step(arm, vf):
    # get current arm configuration in encoder counts
    count0 = arm.odrive.axis0.encoder.shadow_count
    count1 = arm.odrive.axis1.encoder.shadow_count

    # convert counts to radians
    theta0 = calculate.count2rad(arm, count0, 0)
    theta1 = calculate.count2rad(arm, count1, 1)

    # get position of end effector with kinematic equations
    x, y = calculate.fwd_kinematics(arm, theta0, theta1)

    # get desired vectors from defined vector field
    dx, dy = vf.return_vectors(x, y)

    # create matrix of vectors
    vectors = np.array([[dx], [dy]])

    # calculate desired angular velocities by inverting jacobian
    jac = calculate.inv_jacobian(arm, theta0, theta1)
    thetas = np.matmul(jac, vectors)
    dtheta0 = thetas[0, 0]
    dtheta1 = thetas[1, 0]

    # convert rad/s to counts/s
    dcount0 = calculate.rad2count(arm, dtheta0, 0)
    dcount1 = calculate.rad2count(arm, dtheta1, 1)

    # set odrive axis velocities
    arm.odrive.axis0.controller.input_vel = dcount0
    arm.odrive.axis1.controller.input_vel = dcount1

def main():
    # instantiate a haptic device (connects to odrive)
    arm = device.HapticDevice()
    # calibrate encoders with odrive calibration routine
    arm.calibrate()
    # use custom homing routine to define counts/angle
    arm.home()
    # setup control mode
    arm.set_ctrl_mode_velocity()
    # instantiate vector field for arm
    vf_args = {
            'xcenter': arm.arm0.length,
            'ycenter': arm.arm1.length,
            'dtheta' : np.pi/4,
            'radius' : arm.arm0.length/4,
            'buffer' : arm.arm0.length/4 * .05,
            }
    vf = calculate.VectorField(arm, field='spiralbound', args=vf_args)
    try:
        while True:
            step(arm, vf)
    except KeyboardInterrupt:
        arm.restart()
        print('Device stopped and restarted')

if __name__ == "__main__":
    main()

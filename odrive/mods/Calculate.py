import numpy as np

try:
    Axes = [odrv0.axis0, odrv0.axis1]
except:
    try:
        try:
            Axes = [odrv0.axis0, None]
        except:
            Axes = [None, odrv0.axis1]
    except:
        Axes = [None, None]


def torque(odrv0, axis):
    """
    torque measurements give best fit estimate for peak torque when motor cool:
    .417(amps) - 4.2*10^-3
    Estimate only, probably accurate within about 10% error
    """
    if axis == 0:
        return (.417 * odrv0.axis0.motor.current_control.Iq_measured
                - 4.2*10**-3)
    elif axis == 1:
        return (.417 * odrv0.axis1.motor.current_control.Iq_measured
                - 4.2*10**-3)
    else:
        raise Exception("Trying to calculate torque of nonexistant axis %d"
                        %axis)


def position(odrv0, axis):
    """
    Return position of axis using built in odrive shadow count
    """
    if axis == 0:
        return odrv0.axis0.encoder.shadow_count
    elif axis == 1:
        return odrv0.axis1.encoder.shadow_count
    else:
        raise Exception("Trying to calculate position of nonexistant axis %d"
                        %axis)


def velocity(odrv0, axis):
    """
    Use built in odrive velocity estimator to return velocity in radians
    """
    if axis == 0:
        velcps = odrv0.axis0.encoder.vel_estimate
        velrad = velcps * 2 * np.pi / 8192
        return velrad
    elif axis == 1:
        velcps = odrv0.axis1.encoder.vel_estimate
        velrad = velcps * 2 * np.pi / 8192
        return velrad
    else:
        raise Exception("Trying to calculate velocity of nonexistant axis %d"
                        %axis)


def count2theta(count0, count1, arm):
    """
    Convert encoder counts to theta, must be passed current arm class
    """
    if arm.calibrated:
        theta0 = (count0 - arm.count0_zero) / arm.cnt_per_rad0
        theta1 = (count0 - arm.count1_zero) / arm.cnt_per_rad1

        return theta0, theta1
    else:
        raise Exception("Calibration routine for arm must be run first")


def theta2count(theta0, theta1, arm):
    """
    Convert radians to encoder counts, must be passed arm class
    """
    if arm.calibrated:
        count0 = theta0 * arm.cnt_per_rad0 + arm.count0_zero
        count1 = theta1 * arm.cnt_per_rad1 + arm.count1_zero

        return count0, count1
    else:
        raise Exception("Calibration routine for arm must be run first")


def fwd_kinematics(theta0, theta1, arm):
    """
    Use forward kinematics to calculate end effector cartesian coordinate
    """
    x = arm.l1*np.cos(theta0) + arm.l2*np.cos(theta1)
    y = arm.l1*np.sin(theta0) + arm.l2*np.sin(theta1)

    return x, y


def jacobian(theta0, theta1, arm):
    """
    Return array of fwd kinematic eqns
    """
    return np.array([[-arm.l1 * np.sin(theta0), -arm.l2 * np.sin(theta1)],
                  [arm.l1 * np.cos(theta0), arm.l2 * np.cos(theta1)]])


def inv_jacobian(theta0, theta1, arm):
    """
    Returns inverse of jacobian
    """
    return np.linalg.inv(jacobian(theta0, theta1, arm))

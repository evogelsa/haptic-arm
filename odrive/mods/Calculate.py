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

# torque measurements give best fit estimate for peak torque when motor cool:
# .417(amps) - 4.2*10^-3
# Estimate only, probably accurate within about 10% error
def torque(odrv0, axis):
    if axis == 0:
        return (.417 * odrv0.axis0.motor.current_control.Iq_measured
                - 4.2*10**-3)
    elif axis == 1:
        return (.417 * odrv0.axis1.motor.current_control.Iq_measured
                - 4.2*10**-3)
    else:
        raise Exception("Trying to calculate torque of nonexistant axis %d"
                        %axis)

# grab position of specified axis
def position(odrv0, axis):
    if axis == 0:
        return odrv0.axis0.encoder.shadow_count
    elif axis == 1:
        return odrv0.axis1.encoder.shadow_count
    else:
        raise Exception("Trying to calculate position of nonexistant axis %d"
                        %axis)

# uses odrive built in velocity estimate
def velocity(odrv0, axis):
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

def counts_to_rad(counts):
    return counts * 2 * 14 / 24 * np.pi / 8192

def rad_to_xy(rad1, rad2):
    pass

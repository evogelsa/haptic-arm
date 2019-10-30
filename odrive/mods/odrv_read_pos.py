import Setup
import Calculate
import numpy as np
from time import sleep

# define constants
l1 = .1225
l2 = .1225

odrv0 = Setup.setup('both', [3,3], 'none')
Setup.clear_error(odrv0, 'both')

# get zero pos
input("Move ODrive to \"zero\" position and press enter")

theta1_zero = Calculate.position(odrv0, 0)
theta2_zero = Calculate.position(odrv0, 1)

# get cal pos
input("Move ODrive to \"calibration\" position and press enter")

theta1_cal = Calculate.position(odrv0, 0)
theta2_cal = Calculate.position(odrv0, 1)

# get delta angle
delta_angle1 = int(input("Input change in angle1 (degrees): "))
delta_angle2 = int(input("Input change in angle2 (degrees): "))

# get dif in pos
delta_theta1 = theta1_cal - theta1_zero
delta_theta2 = theta2_cal - theta2_zero

print(delta_theta1)
print(delta_theta2)

# counts per degree
cnt_per_deg1 = delta_theta1 / delta_angle1
cnt_per_deg2 = delta_theta2 / delta_angle2

print(cnt_per_deg1)
print(cnt_per_deg2)

# loop to poll for pos
while True:
    pos1 = Calculate.position(odrv0, 0) - theta1_zero
    pos2 = Calculate.position(odrv0, 1) - theta2_zero

    theta1 = Calculate.counts_to_rad(pos1)
    theta2 = Calculate.counts_to_rad(pos2)

    print("Angle1 deg: %.2f" %(pos1 / cnt_per_deg1))
    print("Angle2 deg: %.2f" %(pos2 / cnt_per_deg2))

    print("Angle1 rad2deg: %.2f" %(theta1 * 180 / np.pi))
    print("Angle2 rad2deg: %.2f" %(theta2 * 180 / np.pi))

    x = l1*np.cos(theta1) + l2*np.cos(theta2)
    y = l1*np.sin(theta1) + l2*np.sin(theta2)

    print("x: %.4f" %x)
    print("y: %.4f" %y)
    print("----------------")

    sleep(1.5)

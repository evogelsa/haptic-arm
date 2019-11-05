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

count0_zero = Calculate.position(odrv0, 0)
count1_zero = Calculate.position(odrv0, 1)

# get cal pos
input("Move ODrive to \"calibration\" position and press enter")

count0_cal = Calculate.position(odrv0, 0)
count1_cal = Calculate.position(odrv0, 1)

# get delta angle
theta0_dif = float(input("Input change in angle1 (degrees): ")) * np.pi / 180
theta1_dif = float(input("Input change in angle2 (degrees): ")) * np.pi / 180

# get dif in pos
count0_dif = count0_cal - count0_zero
count1_dif = count1_cal - count1_zero

print(count0_dif)
print(count1_dif)

# counts per degree
cnt_per_rad0 = count0_dif / theta0_dif
cnt_per_rad1 = count1_dif / theta1_dif

print(cnt_per_rad0)
print(cnt_per_rad1)

# loop to poll for pos
while True:
    count0 = Calculate.position(odrv0, 0) - count0_zero
    count1 = Calculate.position(odrv0, 1) - count1_zero

    theta0 = count0 / cnt_per_rad0
    theta1 = count1 / cnt_per_rad1

    print("Angle1 rad: %.2f" %(count0 / cnt_per_rad0))
    print("Angle2 rad: %.2f" %(count1 / cnt_per_rad1))

    print("Angle1 rad2deg: %.2f" %(theta0 * 180 / np.pi))
    print("Angle2 rad2deg: %.2f" %(theta1 * 180 / np.pi))

    x = l1*np.cos(theta0) + l2*np.cos(theta1)
    y = l1*np.sin(theta0) + l2*np.sin(theta1)

    print("x: %.4f" %x)
    print("y: %.4f" %y)
    print("----------------")

    sleep(1.5)

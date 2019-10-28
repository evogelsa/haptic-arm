import Setup
import Calculate
import time
import numpy as np
# import odrive
# from odrive.enums import *


# connect to odrive and setup for position control
odrv0 = Setup.setup('both', [3, 3], 2)

Axes = [odrv0.axis0, odrv0.axis1]
Torques = [0, 0]
Positions = [0, 0]
t0 = time.monotonic()

while True:
    Positions[0] = 900.0 * np.sin((time.monotonic() - t0) * 3)
    Positions[1] = 900.0 * np.cos((time.monotonic() - t0) * 3)

    for i in range(2):
        Axes[i].controller.pos_setpoint = Positions[i]
        Torques[i] = Calculate.torque(odrv0, i)

    print('Axis0: %.1f | Axis1: %.1f | Torque0: %.3f | Torque1: %.3f'
          %(Positions[0], Positions[1], Torques[0], Torques[1]))

    time.sleep(0.01)

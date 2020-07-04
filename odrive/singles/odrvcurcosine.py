import odrive
import numpy as np
import time
import math
from odrive.enums import *

print("Connecting to ODrizzle...")
odrv = odrive.find_any()
print("Connected")

print("Callibrating...")
odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(.1)
print("Callibrated")

print("Enabling voltage control...")
odrv.axis0.controller.config.control_mode = CTRL_MODE_VOLTAGE_CONTROL
print("Enabled")

print("Enabling closed loop control...")
odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Enabled")


print("Performing sine wave...")
t0 = time.monotonic()
print("{0:9s} | {1:9s} | {2:9s}".format("Volts","Current","Torque"))
while True:
    freq = float(input('Frequency (Hz): '))
    t0 = time.monotonic()
    elapsed_time = 0
    while elapsed_time < 5:
        elapsed_time = time.monotonic() - t0
        voltage = 10.0 * (math.sin(2*np.pi*freq*elapsed_time) + 1)
        odrv.axis0.controller.current_setpoint = voltage
        I = voltage / 11
        T = 8.269933431 * I / 41
        print(" {0:9f} | {1:9f} | {2:9f}".format(voltage, I, T))
        time.sleep(0.01)

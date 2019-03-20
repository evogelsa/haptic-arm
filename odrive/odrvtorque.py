import odrive
import time
import math
from decimal import *
from odrive.enums import *

print("Connecting to ODrizzle...")
odrv = odrive.find_any()
print("Connected")

print()
print("Callibrating...")
odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv.axis0.current_state != AXIS_STATE_IDLE:
   time.sleep(.1)
print("Callibrated")

#print()
#print("Enabling closed loop control...")
#odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
#print("Enabled")
#
#
#print()
#print("Enabling position control...")
#odrv.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
#print("Enabled")

print()
print("Enabling current control...")
odrv.axis0.controller.config.control_mode = CTRL_MODE_VOLTAGE_CONTROL
print("Enabled")

print()
print("Setpoint = 0.50")
odrv.axis0.controller.current_setpoint = 0.50
print("Received setpoint = " + str(odrv.axis0.motor.current_control.Iq_setpoint))
time.sleep(.5)

print()
I = odrv.axis0.motor.current_control.Iq_measured
print("Measured current = " + str(I))

while 1:
   I = odrv.axis0.motor.current_control.Iq_measured
   print("Measured current = " + str(I))
   torque = 8.27 * I / 41
   print("Estimated torque = " + str(torque))
   time.sleep(0.05)

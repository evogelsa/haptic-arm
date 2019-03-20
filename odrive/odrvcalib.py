import odrive
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

print("Enabling position controll...")
odrv.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
print("Enabled")

print("Enabling closed loop control...")
odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Enabled")

while True:
   print("Current pos_gain is " + str(odrv.axis0.controller.config.pos_gain))
   try:
      pos_gain = float(input("Input pos_gain: "))
   except ValueError:
      pos_gain = float(odrv.axis0.controller.config.pos_gain)
   odrv.axis0.controller.config.pos_gain = pos_gain

   print("Current vel_gain is " + str(odrv.axis0.controller.config.vel_gain))
   try:
      vel_gain = float(input("Input vel_gain: "))
   except ValueError:
      vel_gain = float(odrv.axis0.controller.config.vel_gain)
   odrv.axis0.controller.config.vel_gain = vel_gain

   print("Current vel_integrator_gain is " + str(odrv.axis0.controller.config.vel_integrator_gain))
   try:
      vel_integrator_gain = float(input("Input vel_integrator_gain: "))
   except ValueError:
      vel_integrator_gain = float(odrv.axis0.controller.config.vel_integrator_gain)
   odrv.axis0.controller.config.vel_integrator_gain = vel_integrator_gain

   for _ in range(3):
      print("Moving to pos 0")
      odrv.axis0.controller.pos_setpoint = 0
      time.sleep(1)
      print("Moving to pos 10000")
      odrv.axis0.controller.pos_setpoint = 10000
      time.sleep(1)

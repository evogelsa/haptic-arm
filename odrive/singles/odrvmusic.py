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
odrv.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
print("Enabled")

print("Enabling closed loop control...")
odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Enabled")

while True:
   delay = float(input("Delay: "))
   speed = float(input("Speed: "))

   for _ in range(1000):
      odrv.axis0.controller.vel_setpoint = speed
      time.sleep(delay)
      odrv.axis0.controller.vel_setpoint = -speed

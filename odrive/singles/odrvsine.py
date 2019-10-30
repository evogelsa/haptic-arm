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

print("Enabling closed loop control...")
odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Enabled")


print("Performing sine wave...")
t0 = time.monotonic()
while True:
   goto = 10000.0 * math.sin(time.monotonic() - t0)
   print(goto)
   odrv.axis0.controller.pos_setpoint = goto
   time.sleep(0.01)


from time import sleep
import odrive
from odrive.enums import *

print("Connecting to ODrive...")
odrv = odrive.find_any()
print("Connected")
print("Clearing all errors")

odrv.axis0.error = 0
odrv.axis1.error = 0
odrv.axis0.motor.error = 0
odrv.axis1.motor.error = 0
odrv.axis0.encoder.error = 0
odrv.axis1.encoder.error = 0

print("Cleared!")

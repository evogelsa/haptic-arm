#Author: Ethan Vogelsang
#Last Modified: 04/02/2019
from os.path import isfile, isdir
from os import mkdir
import math
import time
import numpy as np
import matplotlib.pyplot as plot
import matplotlib.animation as animation
from matplotlib import style
import odrive
from odrive.enums import *


def torqueSine(t0, odrv):
   # get time difference since start of program
   t = time.monotonic() - t0
   # calculate a voltage based on sine wave
   voltage = 5.0 * math.sin(np.pi*t) + 5
   # set odrv to calculated voltage
   odrv.axis0.controller.current_setpoint = voltage
   # calculate amps
   I = voltage / 11
   # calculate torque
   T = 8.269933431 * I / 41
   # get pos
   pos = odrv.axis0.encoder.count_in_cpr
   # convert pos to deg
   pos = (pos1 / 8192 * 360) + (360 * k)
   #print some debugging vals to console
   print("%-9f | %-9f | %-9f | %-9f | %-9f" %(t, voltage, I, T, pos))

 
def odrvSetup():
   print("Connecting to ODrive...")
   # searches for odrive to connect to
   odrv = odrive.find_any()
   print("Connected")
   print("Callibrating...")
   # perform a quick calibration and check for completion
   odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
   while odrv.axis0.current_state != AXIS_STATE_IDLE:
      time.sleep(.1)
   # handle errors that may occur in setup sequence by notifying user and quitting
   if odrv.axis0.error:
      if odrv.axis0.encoder.error:
         print("Encoder error!")
         print(hex(odrv.axis0.encoder.error))
         exit()
      elif odrv.axis0.motor.error:
         print("Motor error!")
         print(hex(odrv.axis0.motor.error))
         exit()
      else:
         print("Axis error!")
         print(hex(odrv.axis0.error))
         exit()

   print("Callibrated")
   print("Enabling voltage control...")
   # set control mode to voltage control we can measure amps from this mode
   odrv.axis0.controller.config.control_mode = CTRL_MODE_VOLTAGE_CONTROL
   
   print("Enabled")
   print("Enabling closed loop control...")
   # enable closed loop control which makes us able to send commands to odrive
   odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
   print("Enabled")

   return odrv


# main()
#
# starts the live plotting program
#
def main():
   # setup ODrive
   odrv = odrvSetup()
   # setup plot
   fig, ax1, ax2, torquefile, X, Y, A, alphafile, posfile, P, ax3 = plotSetup()
   print("Performing sine wave...")
   # set our starting time to current sys time, used for calculating time passed
   t0 = time.monotonic()


# run program
main()

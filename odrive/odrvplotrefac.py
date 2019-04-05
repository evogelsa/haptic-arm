#Author: Ethan Vogelsang
#Last Modified: 04/03/2019
from os.path import isfile, isdir
from os import mkdir
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import odrive
from odrive.enums import *


# odrvSetup(connect)
#
# odrvSetup() is called at beginning of program and initializes the odrive. A 
# full calibration routine is run and the control mode is set appropriately.
# setting connect = 0 allows running program without having odrive present
# Returns the odrive object
#
def odrvSetup(connect):
   if (connect == 0):
      return 0
   else:
      print("Connecting to ODrive...")
      # searches for odrive to connect to
      odrv = odrive.find_any()
      print("Connected")
      print("Callibrating...")
      # perform a quick calibration and check for completion
      odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
      while odrv.axis0.current_state != AXIS_STATE_IDLE:
         time.sleep(.1)
      # handle errors that may occur in setup sequence 
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
      # enable closed loop control allowing ability to send commands to odrive
      odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
      print("Enabled")

      return odrv


# plotSetup() 
#
# plotSetup initializes the live plot and creates the log file. returns the 
# figure, object containing plot elements as myFig, and name of logfile
def plotSetup():
   # set matplotlib style to fivethirtyeight
   style.use("fivethirtyeight")
   # add 3 axes to fig all sharing an x axis
   fig, (ax1, ax2, ax3) = plt.subplots(nrows = 1, ncols = 3, sharex = True)
   # add plot elements to class for clean passing
   class figure:
      def __init__(self, ax1, ax2, ax3):
         self.ax1 = ax1
         self.ax2 = ax2
         self.ax3 = ax3
      def clrAx(self):
         self.ax1.clear()
         self.ax2.clear()
         self.ax3.clear()
   myFig = figure(ax1, ax2, ax3)
   # create logs folder if it doesn't already exist
   if not isdir("logs"):
      mkdir("logs")
   # create log file
   n = 0
   while True:
      logfile = "logs/logout" + str(n) + ".txt"
      if isfile(logfile):
         n += 1
      else:
         with open(logfile, 'w') as file:
            # x,y1,y2,y3 -- x is time
            file.write("0,0,0,0\n")
            break

   return fig, myFig, logfile


# calcAll(logfile path, start time, odrv object)
#
# calcAll calls upon other functions to calculate time, torque, accel, and pos.
# These values are stored in data array and returned to caller
def calcAll(logfile, t0, odrv):
   data = [0] * 4
   # get time difference since start of program
   data[0] = time.monotonic() - t0
   # get new values
   data[1] = calcTorque(data[0], odrv)
   data[2] = calcAccel()
   data[3] = calcPos()
   # create a string of format 'time,torque,accel,pos' with newline
   val = ""
   for i in range(4):
      if i < 3:
         val += str(data[i]) + ','
      else:
         val += str(data[i]) + '\n'
   # write the string to logfile
   with open(logfile, 'a') as file:
      file.write(val)
   #print vals to terminal for debugging
   print("%-9f | %-9f | %-9f | %-9f" %(data[0], data[1], data[2], data[3]))

   return data

# calcTorque(call time, odrv object)
#
# called by calcAll, returns torque
def calcTorque(t, odrv):
   # calculate a voltage based on sine wave
   voltage = 5.0 * math.sin(t) + 5
   # set odrv to calculated voltage
   if (odrv != 0):
      odrv.axis0.controller.current_setpoint = voltage
   # calculate amps I = V/R
   I = voltage / 11
   # calculate torque T = 8.269933431 * I / motor_kv
   T = 8.269933431 * I / 41
   
   return T


# calcAccel
#
#
def calcAccel():
   if (odrv == 0 ):
        return 0


# calcPos
#
#
def calcPos():
   if (odrv == 0):
       return 0


# plotUpdate(interval, figure object, log path, start time, new data, odrv obj)
#
# plotUpdate is the animation function for matplotlib.FuncAnimation. It gets new
# data using calcAll and updates logfile and array that stores the most recent
# data. It then updates the plot with the new data
def plotUpdate(i, myFig, logfile, t0, data, odrv):
   # calculate the new data
   newData = calcAll(logfile, t0, odrv)
   # add new data to list and get rid of the oldest data
   for i in range(4):
      data[i].append(newData[i])
      data[i] = data[i][-50:]
   # clear all axes and replot with new data
   myFig.clrAx()
   myFig.ax1.plot(data[0], [round(T, 4) for T in data[1]])
   myFig.ax2.plot(data[0], [round(a, 4) for a in data[2]])
   myFig.ax3.plot(data[0], [round(p, 4) for p in data[3]])
   myFig.ax1.set_ylim(-.25,.25)
   myFig.ax1.set_title("Torque (Nm)")
   myFig.ax2.set_title("Acceleration (idk units yet)")
   myFig.ax3.set_title("Position (also dk units yet)")


def main():
   # init odrive, pass 0 to skip setup for demo without odrive
   odrv = odrvSetup(0)
   fig, myFig, logfile = plotSetup()
   data = [[0]*200,[0]*200,[0]*200,[0]*200]
   t0 = time.monotonic()
   ani = animation.FuncAnimation(fig, plotUpdate, fargs = (myFig, logfile, t0,
                                 data, odrv),  interval = 10)
   plt.show()

main()

#Author: Ethan Vogelsang
#Last Modified: 04/03/2019
from os.path import isfile, isdir
from os import mkdir
import math
import time
import numpy as np
import odrive
from odrive.enums import *
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg


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


# logFile() 
#
# checks if logs directory exists and creates it if it doesnt. then safely makes
# a new logfile that data will be outputted to.
def logFile():
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
            file.write("0,0,0,0,0\n")
            break

   return logfile


# calcAll(logfile path, start time, odrv object)
#
# calcAll calls upon other functions to calculate time, torque, acc, pos, & vel.
# These values are stored in data array and returned to caller
def calcAll(logfile, t0, odrv):
   data = [0] * 5
   # get time difference since start of program
   data[0] = time.monotonic() - t0
   # get new values
   data[1] = calcTorque(data[0], odrv)
   data[2] = calcAccel(odrv)
   data[3] = calcPos(odrv)
   data[4] = calcVel(odrv)
   # create a string of format 'time,torque,accel,pos' with newline
   val = ""
   for i in range(5):
      if i < 4:
         val += str(data[i]) + ','
      else:
         val += str(data[i]) + '\n'
   # write the string to logfile
   with open(logfile, 'a') as file:
      file.write(val)
   #print vals to terminal for debugging
   print("%-9f %-9f %-9f %-9f %-9f" %(data[0], data[1], data[2], data[3], data[4]))

   return data


# calcTorque(call time, odrv object)
#
# called by calcAll, returns torque
def calcTorque(t, odrv):
   # calculate a voltage based on sine wave
   voltage = 5 * math.sin(t)
   # set odrv to calculated voltage
   if (odrv != 0):
      odrv.axis0.controller.current_setpoint = voltage
   # calculate amps I = V/R
   I = voltage / 11
   # calculate torque T = 8.269933431 * I / motor_kv
   T = 8.269933431 * I / 41
   
   return T


# calcVel
#
#
def calcVel(odrv):
   if (odrv == 0):
      return 0
   velcps = odrv.axis0.motor.rotor.pll_vel
   velrad = velcps * 2 * np.pi / 8192
   return velrad


# calcAccel
#
#
def calcAccel(odrv):
   if (odrv == 0 ):
      return 0
   return 0


# calcPos
#
#
def calcPos(odrv):
   if (odrv == 0):
      return 0
   counts = odrv.axis0.encoder.shadow_count
   rad = counts * 2 * np.pi / 8192
   return rad


# connect to odrive if non zero, otherwise simulate torque
odrv = odrvSetup(0)

# create window 
win = pg.GraphicsWindow()
win.setWindowTitle("ODrive")

# init data dictionary
data = {
   "time": np.zeros(200),
   "torque": np.zeros(200),
   "pos": np.zeros(200),
   "accel": np.zeros(200),
   "vel": np.zeros(200)
}


# add plots to window and set appropriate ranges
p1 = win.addPlot()
p1.setYRange(-0.1, 0.1)
p1.setTitle("Torque (nm)")

p2 = win.addPlot()
p2.setYRange(-0.1, 0.1)
p2.setTitle("Position (rad)")

win.nextRow()
p3 = win.addPlot()
p3.setYRange(-0.1, 0.1)
p3.setTitle("Velocity (rad/s)")

p4 = win.addPlot()
p4.setYRange(-0.1, 0.1)
p4.setTitle("Acceleration (rad/s/s)")

# add curves and initialize with empty data
curve1 = p1.plot(data["time"], data["torque"])
curve2 = p2.plot(data["time"], data["pos"])
curve3 = p3.plot(data["time"], data["vel"])
curve4 = p4.plot(data["time"], data["accel"])

# get logfile
logfile = logFile() 

# record time 0
t0 = time.monotonic()

def update():
   newData = calcAll(logfile, t0, odrv)
   # add new data to list and get rid of the oldest data
   tags = ["time","torque","accel","vel", "pos"]
   for j in range(5):
      data[tags[j]][0] = newData[j]
      data[tags[j]] = np.roll(data[tags[j]],-1)
   curve1.setData(data["time"], data["torque"])
   curve2.setData(data["time"], data["pos"])
   curve3.setData(data["time"], data["vel"])
   curve4.setData(data["time"], data["accel"])

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

QtGui.QApplication.instance().exec_()   

#if __name__ == '__main__':
#   import sys
#   if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
#      QtGui.QApplication.instance().exec_()   
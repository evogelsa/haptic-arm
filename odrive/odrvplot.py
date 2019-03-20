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
 
# 
# start sine wave function of torque
#
def torqueSine(torqfile, t0):
   # get time difference since start of program
   t = time.monotonic() - t0
   # calculate a voltage based on sine wave
   voltage = 10.0 * math.sin(t)
   # set odrv to calculated voltage
   odrv.axis0.controller.current_setpoint = voltage
   # calculate amps
   I = voltage / 11
   # calculate torque
   T = 8.269933431 * I / 41
   # create a string of format 'time,torque' with newline
   coord = str(t) + ',' + str(T) + '\n'
   # write coord to the torqfile
   data = open(torqfile,'a')
   data.write(coord)
   # close the file
   data.close()
   #print some debugging vals to console
   print(" {0:9f} | {0:9f} | {1:9f} | {2:9f}".format(t, voltage, I, T))
# 
# end sine wave function of torque
#

#
# start calc angular vel and accel func
#
def calcInputs(alphafile):
   # make list of size 2 to store angular vel
   omegas = [0,0]
   for _ in range(2):
      # read two encoder counts 5ms after each other
      c1 = odrv.axis0.encoder.count_in_cpr
      time.sleep(0.005)
      c2 = odrv.axis0.encoder.count_in_cpr
      # calc change in encoder counts "delta count"
      dc = c2 - c1
      # convert counts to degrees
      deg = dc * 360 / 8192 
      # convert degrees to radians
      theta = deg * math.pi / 180
      # store two measurements of angular vel inside omegas
      omegas[_] = theta / 0.005
   # calc change in omegas "delta omega"
   domega = omegas[0] - omegas[1]
   # angular acceleration = change in vel / change in time
   alpha = domega / 0.01
   # output the omegas and the alpha to a text file "aout#.txt"
   vals = str(omegas[0]) + ',' + str(omegas[1]) + ',' + str(alpha) + '\n'
   data = open(alphafile,'a')
   data.write(vals)
   data.close()

   return omegas, alpha
#
# end calc angular vel and accel func
#

#
# Start plot setup
#
def plotSetup():
   # choose style of plot
   style.use('fivethirtyeight')
   # create figure for plot
   fig = plot.figure()
   # add our axes, nrows = 1, ncols = 1, index = 1
   ax1 = fig.add_subplot(1,1,1)
   # make second axis with same x axis as ax1
   ax2 = ax1.twinx()
   # check for logs folder, if not exist make it
   if not isdir('logs'):
      mkdir('logs')
   # check if tout0 exists and if it does create file with correct number
   if isfile('logs/tout0.txt'):
      n = 1
      # iterate through files 0 to x until x doesn't exist
      while n > 0:
         torqfile = 'logs/tout' + str(n) + '.txt'
         if isfile(torqfile):
            n += 1
         else:
            data = open(torqfile,'w')
            data.write('0,0\n')
            data.close()
            n = 0
   else:
      torqfile = 'logs/tout0.txt'
      data = open(torqfile,'w')
      data.write('0,0\n')
      data.close()
   # check if aout0 exists and if it does create file with correct number
   if isfile('logs/aout0.txt'):
      n = 1
      # iterate through files 0 to x until x doesn't exist
      while n > 0:
         alphafile = 'logs/aout' + str(n) + '.txt'
         if isfile(alphafile):
            n += 1
         else:
            data = open(alphafile,'w')
            data.write('0,0,0\n')
            data.close()
            n = 0
   else:
      alphafile = 'logs/aout0.txt'
      data = open(torqfile,'w')
      data.write('0,0,0\n')
      data.close()
   # make some empty arrays
   X = []
   Y = []
   W1 = []
   W2 = []
   A = []

   return fig, ax1, ax2, torqfile, X, Y, W1, W2, A, alphafile
#
# End plot setup
#

#
# Start animation function
# 
def animate(i,X,Y,W1,W2,A):
   # run torque sine to generate new point and write to file
   torqueSine(torqfile, t0)
   omegas, alpha = calcInputs(alphafile)
   # read the file to see the new data
   data = open(torqfile,'r').read()
   # split each coord with \n
   lines = data.split('\n')
   # make empty data arrays
   for line in lines:
   # check that line isnt empty
      if len(line) > 1:
         # split x and y with ,
         x, y = line.split(',')
         X.append(float(x))
         Y.append(float(y))
   # read the alpha file to see the new data
   data = open(alphafile,'r').read()
   # split each coord with \n
   lines = data.split('\n')
   # make empty data arrays
   for line in lines:
   # check that line isnt empty
      if len(line) > 1:
         # split vals with ,
         omega1, omega2, alpha = line.split(',')
         #W1.append(float(omega1))
         #W2.append(float(omega2))
         A.append(float(alpha))
   # limit X and Y and A to the last 2 seconds of data
   Y = Y[-200:]
   X = X[-200:]
   A = A[-200:]
   # clear the old stuff from graph
   ax1.clear() 
   ax2.clear()
   # set current axis to ax1
   plot.sca(ax1)
   # draw new data onto plot
   ax1.plot(X,[round(y, 3) for y in Y])
   plot.xticks(rotation=45, ha='right')
   plot.ylim((-.3,.3))
   #plot.yticks(np.arange(-0.25,0.25,0.05))
   plot.title('Torque vs Time')
   plot.xlabel('Time')
   plot.ylabel('Torque (nm)')
   # set current axis to ax2
   plot.sca(ax2)
   plot.plot(color='r')
   ax2.plot(X,[round(a, 3) for a in A], color='red',linewidth=0.8)
   plot.ylim((-2500,2500))
   plot.ylabel('Angular Acceleration (rads/s/s)')
# 
# End animation function
#

#
# Start ODrive setup
#
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
#
# End ODrive setup
#

# setup ODrive
odrv = odrvSetup()
# setup plot
fig, ax1, ax2, torqfile, X, Y, W1, W2, A, alphafile = plotSetup()
# set our starting time to current sys time, used for calculating time passed
t0 = time.monotonic()
print("Performing sine wave...")
# print formatted headers 
print(" {0:9s} | {0:9s} | {1:9s} | {2:9s}".format("Time","Volts","Current","Torque"))
# matplot lib animation function, figure to use, func to use for animation, interval between updates
ani = animation.FuncAnimation(fig, animate, fargs=(X,Y,W1,W2,A), interval=10)
# show our plot
plot.show()

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

 
# torqueSine(data_file_path, init_time)
# 
# This function calculates a voltage to apply to the motor based on a sine wave
# Function writes a pair of coordinates for time and the calculated torque at 
# that time to the file provided to the function. The time is the current time
# minus the initialization time
#
def torqueSine(torquefile, t0, odrv):
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
   # create a string of format 'time,torque' with newline
   coord = str(t) + ',' + str(T) + '\n'
   # write coord to the torquefile
   data = open(torquefile,'a')
   data.write(coord)
   # close the file
   data.close()
   #print some debugging vals to console
   print("%-9f | %-9f | %-9f | %-9f" %(t, voltage, I, T))


# calcPos(data_file_path)
#
# this function writes the position in degrees to a data file provided in args
#
def calcPos(posfile, odrv, k):
   pos1 = odrv.axis0.encoder.count_in_cpr
   pos1 = (pos1 / 8192 * 360) + (360 * k)
   with open(posfile) as data:
      for i, lines in enumerate(data):
         pass
   with open(posfile) as data:
      for line in range(i):
         data.readline()
      val = data.readline()
   print(val)
   pos2 = float(val)
   if (pos1 - pos2) > 30:
      k += 1
   elif (pos1 - pos2) < 30:
      k -= 1
   pos1 = pos1 + 360 * k
   val = str(pos1) + '\n'
   data = open(posfile, 'a')
   data.write(val)
   data.close()
   return k


# calcWA(data_file_path)
#
# this function takes a path to a data file and writes two values for an
# estimated angular velocity in rad/s and an estimated angular acceleration
# from those two velocities.
#
def calcWA(alphafile, odrv):
   # make list of size 2 to store angular vel
   omegas = [0,0]
   for _ in range(2):
      # read two encoder counts 10ms after each other
      c1 = odrv.axis0.encoder.count_in_cpr
      time.sleep(0.01)
      c2 = odrv.axis0.encoder.count_in_cpr
      # calc change in encoder counts "delta count"
      dc = c2 - c1
      # convert counts to degrees
      deg = dc * 360 / 8192 
      # convert degrees to radians
      theta = deg * math.pi / 180
      # store two measurements of angular vel inside omegas
      omegas[_] = theta / 0.01
   # calc change in omegas "delta omega"
   domega = omegas[0] - omegas[1]
   # angular acceleration = change in vel / change in time
   alpha = domega / 0.02
   # output the omegas and the alpha to a text file "aout#.txt"
   vals = str(omegas[0]) + ',' + str(omegas[1]) + ',' + str(alpha) + '\n'
   data = open(alphafile,'a')
   data.write(vals)
   data.close()


# plotSetup()
#
# plotSetup is called at the beginning of the program and creates the parameters
# to be used on the live plot. It also creates the data out files for torque and
# angular accelerations called tout#.txt and aout#.txt. Returns the paths for
# the data files and other parameters used in the animation function
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
   ax3 = ax1.twinx()
   # adjust the padding of plots
   plot.subplots_adjust(left=.140,right=.84,bottom=.14,top=.92)
   # check for logs folder, if not exist make it
   if not isdir('logs'):
      mkdir('logs')
   # check if tout0 exists and if it does create file with correct number
   if isfile('logs/tout0.txt'):
      n = 1
      # iterate through files 0 to x until x doesn't exist
      while n > 0:
         torquefile = 'logs/tout' + str(n) + '.txt'
         if isfile(torquefile):
            n += 1
         else:
            data = open(torquefile,'w')
            data.write('0,0\n')
            data.close()
            n = 0
   else:
      torquefile = 'logs/tout0.txt'
      data = open(torquefile,'w')
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
      data = open(torquefile,'w')
      data.write('0,0,0\n')
      data.close()
   # check if pout0 exists and if it does create file with correct number
   if isfile('logs/pout0.txt'):
      n = 1
      # iterate through files 0 to x until x doesn't exist
      while n > 0:
         posfile = 'logs/pout' + str(n) + '.txt'
         if isfile(posfile):
            n += 1
         else:
            data = open(posfile,'w')
            data.write('0.0\n')
            data.close()
            n = 0
   else:
      posfile = 'logs/pout0.txt'
      data = open(posfile,'w')
      data.write('0.0\n')
      data.close()
   # make some empty arrays
   X = []
   Y = []
   A = []
   P = []

   return fig, ax1, ax2, torquefile, X, Y, A, alphafile, posfile, P, ax3


# animate(interval, float_array_time, float_array_torque, float_array_alpha)
#
# animate is called by FuncAnimation which passes in above args. This function
# reads in data from the files and plots it on the live plot. Data is trimmed
# to only show the last 200 values. The the time window is determined by the 
# amount of data trimemed * the interval of measurements. I.e. measurements 
# every 10 ms * 200 data points = 2 seconds of data shown on the plot
# 
def animate(i,t0,X,Y,A,torquefile,alphafile,odrv,ax1,ax2,posfile,P,ax3,k):
   # run torque sine to generate new point and write to file
   torqueSine(torquefile, t0, odrv)
   calcWA(alphafile, odrv)
   k = calcPos(posfile, odrv, k)
   # read the file to see the new data
   data = open(torquefile,'r').read()
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
         A.append(float(alpha))
   # read the pos file to see the new data
   data = open(posfile,'r').read()
   # split each coord with \n
   lines = data.split('\n')
   # make empty data arrays
   for line in lines:
   # check that line isnt empty
      if len(line) > 1:
         pos = line
         P.append(float(pos))
   # limit X and Y and A to the last 2 seconds of data
   Y = Y[-20:]
   X = X[-20:]
   A = A[-20:]
   P = P[-20:]
   # clear the old stuff from graph
   ax1.clear() 
   ax2.clear()
   ax3.clear()
   # set current axis to ax1
   plot.sca(ax1)
   # draw new data onto plot and round to four decimal places
#   ax1.plot(X,[round(y, 4) for y in Y])
#   plot.xticks(rotation=45, ha='right')
#   plot.ylim((-.3,.3))
#   plot.title('Torque vs Time')
#   plot.xlabel('Time')
#   plot.ylabel('Torque (nm)')
   # set current axis to ax2
   plot.sca(ax2)
   plot.plot(color='r')
   ax2.plot(X,[round(p,4) for p in P], color='red',linewidth=0.8)
   plot.ylim((-500,500))
   plot.ylabel('Pos (deg)')
   # set current axis to ax3
#   plot.sca(ax3)
#   plot.plot(color='g')
#   ax3.plot(X,[round(a,4) for a in A], color='green',linewidth=0.8)
#   plot.ylim((-350,350))
#   plot.ylabel('Angular Acceleration (rads/s/s)')


# odrvSetup()
#
# odrvSetup() is called at beginning of program and initializes the odrive. A 
# full calibration routine is run and the control mode is set appropriately.
# Returns the odrive object
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
   k = 0
   # setup ODrive
   odrv = odrvSetup()
   # setup plot
   fig, ax1, ax2, torquefile, X, Y, A, alphafile, posfile, P, ax3 = plotSetup()
   # set our starting time to current sys time, used for calculating time passed
   t0 = time.monotonic()
   print("Performing sine wave...")
   # print formatted headers 
   print("%-9s | %-9s | %-9s | %-9s" %(" Time","Volts","Current","Torque"))
   # animation function takes figure to use, func for animation, update interval 
   ani = animation.FuncAnimation(fig,animate,fargs=(t0,X,Y,A,torquefile,alphafile,odrv,ax1,ax2,posfile,P,ax3,k),interval=10)
   # show our plot
   plot.show()


# run program
main()

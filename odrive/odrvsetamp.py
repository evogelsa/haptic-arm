from time import sleep, monotonic
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
         sleep(.1)
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
      odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
      while odrv.axis1.current_state != AXIS_STATE_IDLE:
         sleep(.1)
      # handle errors that may occur in setup sequence
      if odrv.axis1.error:
         if odrv.axis1.encoder.error:
            print("Encoder error!")
            print(hex(odrv.axis1.encoder.error))
            exit()
         elif odrv.axis1.motor.error:
            print("Motor error!")
            print(hex(odrv.axis1.motor.error))
            exit()
         else:
            print("Axis error!")
            print(hex(odrv.axis1.error))
            exit()

      print("Callibrated")
      print("Enabling voltage control...")
      # set control mode to voltage control we can measure amps from this mode
      odrv.axis0.controller.config.control_mode = CTRL_MODE_VOLTAGE_CONTROL
      odrv.axis1.controller.config.control_mode = CTRL_MODE_VOLTAGE_CONTROL

      print("Enabled")
      print("Enabling closed loop control...")
      # enable closed loop control allowing ability to send commands to odrive
      odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
      odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
      print("Enabled")

      return odrv


odrv = odrvSetup(1)
while True:
   axis = int(input("Select axis: "))
   I = float(input("Input amps: "))
   V = float(I * 11)
   print("Axis = %d | V = %-9f | I = %-9f" %(axis, V, I))
   if axis == 0:
      odrv.axis0.controller.current_setpoint = V
   elif axis == 1:
      odrv.axis1.controller.current_setpoint = V
   else:
      print("Unknown axis")


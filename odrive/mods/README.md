## File descriptions

### Calculate.py
Contains functions to either calculate useful values or grab values from odrive
functions.

### Setup.py
Contains functions useful for initializing ODrive such as setup routines, custom
and built-in calibration routines, and error clearing.

### odrv_read_pos.py
Initializes ODrive and runs through a demo theta calibration routine. Reads out
position of end effector and angles of encoders.

### odrv_circle_pos.py
Draws a circle with the end effector using position control. Program calculates
waypoints necessary to draw a circle and uses inverse kinematics to find the
necessary thetas that the motors must move to get to given waypoint.

### odrv_circle_cur.py
WIP to draw circle with end effector using current control. Program will make a
vector field to describe the velocity of the end effector at any given point in
space.

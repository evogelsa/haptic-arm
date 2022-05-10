# Haptic Arm

## Overview

This repository contains all the code and files related to the SCARA haptic arm
I built while working as an undergraduate researcher. The arm is a 2DOF
"revolute-revolute" robot. What makes this project special is the control
methods used on the arm. A combination of inverse and forward kinematics are
used to carefully apply forces to the end-effector depending on the current arm
configuration. These forces are determined by a vector field that maps out the
desired magnitudes and directions for points on a cartesian plane.

## Design

### Software

The code takes a class based approach to just about everything. The
`HapticDevice` class is the primary interface to the hardware and wraps the
odrive instance as well as methods and parameters to help in calibration and
manipulation of the device. The `VectorField` class defines the various
vector fields that are responsible for calculating the forces to apply to the
robot. The `Coord` class is a helper class to translate points between cartesian
coordinates (x and y in meters) to polar coordinates (r and theta also in
meters) and to "window" coordinates (matrix like i and j in pixels). Finally all
of the classes in `visualize.py` are used for rendering out the visualization
and simulation of the arm, but are not essential for robot operation.

### Hardware

The current robot design uses 3D printed arms with timing pulleys. The motors
are stacked at the base of the robot, and they manipulate the arms through
timing belts. Motor control is done through a 24V ODrive motor controller
(hardware v3.5). Two CUI AMT102-V encoders are used in conjunction with two
ODrive D5065-270K motors to drive the robot.

{{DEVICE PICTURE}}

Mechanical design is not my strong suit, but the upside is that this robot
should be relatively hardware agnostic. A person with more hardware skill and
knowledge should be able to take this project and develop a new design that
applies the same SCARA-like principles and then successfully run the code with
minimal modification. The motor locations and distances should not matter as
long as the arm lengths are adjusted in the `HapticDevice` class instances.

### Bill of Materials

If you do want to replicate the design I used, here is the bill of materials.

{{BOM}}


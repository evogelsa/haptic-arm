# Haptic Arm

## Overview

This repository contains all the code and files related to the SCARA haptic arm
I built while working as an undergraduate researcher. The arm is a 2DOF
"revolute-revolute" robot. What makes this project special is the control
methods used on the arm. A combination of inverse and forward kinematics are
used to carefully apply forces to the end-effector depending on the current arm
configuration. These forces are determined by a vector field that maps out the
desired magnitudes and directions for points on a cartesian plane.

A simulation of the arm using this code can be seen here.

![arm sim](https://i.imgur.com/62qTkmU.gifv)

### File Descriptions

The files in this repository are as follows.

```txt
├── cad
│   └── assembly.stp                --> 3D CAD files to replicate design
├── LICENSE.md                      --> MIT license this project is under
├── pics
│   ├── arm_base.jpg                --> Picture of the ARM base assembly
│   └── odrive.jpg                  --> Picture of the ODrive on the arm base
├── README.md                       --> This file
├── references.md                   --> Some references I found useful
└── src
    ├── calculate.py                --> Helper functions and classes for math
    ├── condition-number-heatmap
    │   └── condition_number.png    --> Heat map of condition number for arm
    ├── device.py                   --> Hardware interface classes and methods
    ├── font
    │   └── Inconsolata-Regular.ttf --> Font file for visualization
    ├── lib
    │   ├── libfreetype-6.dll       --> DLL required for running vis on Windows
    │   ├── SDL2.dll                --> DLL required for running vis on Windows
    │   ├── SDL2_ttf.dll            --> DLL required for running vis on Windows
    │   └── zlib1.dll               --> DLL required for running vis on Windows
    ├── main.py                     --> Runs haptic simulation on the robot
    ├── pyproject.toml              --> Rules for formatting with Black
    ├── requirements.txt            --> Project requirements
    ├── theta-heatmap
    │   ├── circle_dtheta0.png      --> Heat map of motor 0 vel for circle VF
    │   ├── circle_dtheta1.png      --> Heat map of motor 1 vel for circle VF
    │   ├── circle_theta0.png       --> Heat map of motor 0 pos for circle VF
    │   ├── circle_theta1.png       --> Heat map of motor 1 pos for circle VF
    │   ├── spring_dtheta0.png      --> Heat map of motor 0 vel for spring VF
    │   ├── spring_dtheta1.png      --> Heat map of motor 1 vel for spring VF
    │   ├── spring_theta0.png       --> Heat map of motor l pos for spring VF
    │   └── spring_theta1.png       --> Heat map of motor 1 pos for spring VF
    └── visualize.py                --> Visualization and simulation for program
```

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

![arm](pics/arm.png)

![odrive](pics/odrive.png)

Mechanical design is not my strong suit, but the upside is that this robot
should be relatively hardware agnostic. A person with more hardware skill and
knowledge should be able to take this project and develop a new design that
applies the same SCARA-like principles and then successfully run the code with
minimal modification. The motor locations and distances should not matter as
long as the arm lengths are adjusted in the `HapticDevice` class instances.

STEP files of the current design are provided in the [cad](cad) folder.
The device is not properly assembled or organized, but all the parts for 3D
printing can be found there.

### Bill of Materials

If you do want to replicate the design I used, here is a rough bill of
materials.

| Part                              | Part Number         | Quantity   |
|-----------------------------------|---------------------|------------|
| OpenBeam precut kit               | 100540              | 1          |
| ODrive (v3.5+)                    | ODrive v3.5         | 1          |
| ODrive dual shaft motor 270 kV    | D5065-270KV         | 2          |
| CUI encoders                      | AMT102-V            | 2          |
| Cable for CUI AMT-102             | find on ODrive shop | 2          |
| Yellow Jacket 608 bearings 8 pack | B071DT4V1Q          | 2          |
| M8 all thread                     | xx                  | 1 meter    |
| M8 nuts                           | xx                  | 4          |
| Various M3 screws and nuts        | xx                  | 50 or less |
| T5 73 tooth timing belt           | T5-365-10           | 1          |
| T5 37 tooth timing belt           | T5-185-10           | 2          |

## Setup and Use

### ODrive

The ODrive, motors, and encoders will need to be configured before this project
can be used. Documentation on the [odrive website](docs.odriverobotics.com/)
should be followed for this. Most importantly, the PIDs of the controller will
need to be well tuned for the motors to work effectively.

### Code

All software files and resources are contained in the [src](src) folder. Before
running any code, the python modules listed in the
[requirements](requirementes.txt) should be installed first through pip.

```txt
pip -r requirements.txt
```

If running the visualization on a Windows computer, the DLLs in the
[lib](src/lib) folder will need to be present in the src directory, same as they
are in this repo. If running on linux, SDL2 and SDL2_TTF should be installed
manually or through your package manager. For Arch:

```txt
# pacman -Syu sdl2 sdl2_ttf
```

If already connected to an ODrive, the [main.py](src/main.py) can be run
directly. Some command line arguments are available and can be listed with the
following.

```txt
python3 main.py --help
```

Because the program will start by calibrating the encoders, it is necessary that
the motors are able to rotate freely. For the current hardware design, this
means the timing belts must be disengaged from the motor pulleys during
calibration, hence the purpose of the sliding motor mounts.

If not connected to hardware, or if a simulation is desired,
[visualize.py](src/visualize.py) can also be run as main. The same or similar
command line arguments will be required and can be listed in similar manor.

```txt
python3 visualize.py --help
```

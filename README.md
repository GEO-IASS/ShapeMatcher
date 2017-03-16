# ShapeMatcher
## Overview
Firebird V a robot developed by [Nex Robotics](www.nex-robotics.com/) was programmed to pick shapes made of thermocol and place them in matched cells in a grid by avoiding obstacles in its path. The robot was guided through an overhead camera mounted on the ceiling. A-Star algorithm was used to provide the robot with shortest path to be traversed. The robot code is written in Embedded-C and the image processing code is written in Python using OpenCV library. XBEE modules were used for communication.

## Demo
![DEMO](/Demo/ShapeMatcher.gif?raw=true "Optional Title")
1. [YouTube Demo](https://youtu.be/TyNgytB2YUo)

## Components Used
1. Firebird V Robot
2. XBEE modules
3. USB Camera
4. Grid with obstacles

## Softwares Used
1. Atmel Studio 6.0.1843
2. X-CTU
3. WinAVR 20090313
4. AVR Bootloader

## Usage
The code for the bot is found in Embedded-C folder. The hex file is burned onto the robot using AVR Bootloader software.
The image processing code and communication code is found in the Python folder.

## References
1. [OpenCV](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html)
2. [Path Planning Algorithm](http://www.redblobgames.com/pathfinding/a-star/introduction.html)
3. [Firebird V Robot](http://www.nex-robotics.com/products/fire-bird-v-robots/fire-bird-v-atmega2560-robotic-research-platform.html)

## Software Drive
[Drive Link](http://elsi.e-yantra.org/resources#winodws_softwares)

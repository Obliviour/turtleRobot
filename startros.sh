#!/bin/bash

# make this executable chmod +x script.sh
# make sure the robot is turned on before running it

# general ros node 
roslaunch turtlebot_bringup minimal.launch --screen

# additional ones
# roslaunch turtlebot_teleop keyboard_teleop.launch --screen
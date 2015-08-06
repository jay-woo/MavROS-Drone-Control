#!/bin/sh
cd ~/catkin_ws
pwd
source devel/setup.sh
rosrun drone_control tkinter_map_manager.py

# MavROS-Drone-Control

This repository contains various scripts and launch files for controlling drones using MavROS.

## Installation
1. Ensure you have ROS installed (preferablly Jade). Follow the directions [here] (http://wiki.ros.org/jade/Installation/Ubuntu).
2. Install a package to allow [joystick control](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick).
3. Install [MavROS](https://github.com/mavlink/mavros/tree/master/mavros).
4. Clone this repository in the /src folder of your Catkin workspace

## Usage
There are several launch files that each perform a different mission to accomplish a particular task.
- waypoints.launch - Publishes a series of waypoints to the air vehicle
- fiducial.launch - Controls the drone using visual servoing
- snotbot.launch - Simulates the collection of whale snot

For much more detailed information on each of these launch files, consult the documentation in the Google Drive.

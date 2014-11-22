# Overview

This package is a fork of https://github.com/damanfb/ros_vrpn_client, which is a fork of the official ros_vrpn_client.
Its aim is to support ros_vrpn_client on ROS Indigo.

# Installation

1. Download VRPN from http://www.cs.unc.edu/Research/vrpn/. The instruction where tested with version 07.32.
2. Clone this repository to a ROS workspace
3. Run `export VRPN_ROOT=/path/where/your/vrpn/is`
4. Run `catkin_make`

# Testing your connection

1. Start a VRPN server. For example, you can use `vrpn_server` with a vrpn.cfg containing the line `vrpn_Tracker_NULL	Tracker0	1	60.0`
2. Run `roslaunch ros_vrpn_client track.launch` to listen to Tracker0a@localhost
3. `rosrun tf view_frames && evince frames.pdf` should show Tracker0 with an update rate of 60 Hz

# TF frames

The layout is `vrpn/<name of your ros node>`.

# Notes

* Unlike the original package, this package does neither do any additional transformation nor has special handling for optitrack.
* The original package had some latency (up to one second) between a received vrpn message and publishing the transformation. This limitation has been removed.

# Overview

This document covers a running explanation of how to use the repo, because my memory is terrible

## Gazebo_Push_Plugin

This package was directly derived from a [Gazebo tutorial](http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin) giving an example of how to create a plugin which interacts with a Gazebo model in simulation.

## ackermann.urdf.xacro

Custom xacro file of an ackermann vehicle. xacro is a scripting language for more easily creating URDF files which is used in ROS but not Gazebo. The scripting functionality greatly simplifies creating a model, and there is an `xacro.py` script, which turns xacro files into Gazebo useable URDF files. An example of this is available in `gazebo.launch`.

This file also provides and example of how to load a model with a gazebo plugin, namely `gazebo_push_plugin`.

## Gazebo.Launch

This launch file launches Gazebo and loads `ackermann.urdf.xacro` into it. 

The launch file also launches the robot state publisher, which is important for interfacing between ROS and Gazebo

## ackermann.launch

### RVIZ Note

Without use of some sort of odometry or localization package, robot_state_publisher will only provide the changes links and joints of the model relative to each other. There are ways of getting the ground truth of the robot, for example the `p3d_base_controller` as shown in `ackermann.urdf.xacro` however I haven't made a node to convert the odometry message output to a TF message format, so it will not be visualized in RVIZ yet.
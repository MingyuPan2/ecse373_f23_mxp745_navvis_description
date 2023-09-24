# Lab 2: navvis_description

## Overview

In this lab, a robot with a base, four lasers and three wheels was modeled. This lab demonstrate how to develop a robot description in ROS. BY using URDF and XACRO, description of robots were described, created and automated.

### Lab 2 Link

Lab 2 Link: [Laboratory #2_20230913_cert.pdf](https://canvas.case.edu/courses/38747/assignments/509271/)

## Modelling Processes

### Know the dimensions of the robot

Follow the bulleted list describing the geometry of the visual and collision elements of the Navvis platform. The names of the links are critical. 

The dimensions are listed below:
base (parent): box size 0.45 m x 0.78 m x 1.98 mm <br />
laser_vert_bottom (child): box size 0.062 mm x 0.062 mm x 0.087 mm <br />
laser_vert_top_right (child): box size 0.062 mm x 0.062 mm x 0.087 mm <br />
laser_vert_top_left (child): box size 0.062 mm x 0.062 mm x 0.087 mm <br />
laser_horiz (child): cylinder radius 0.052 mm x length 0.072 mm <br />

For position and orientation parameters, follow the graph on P2.

### Creating a URDF File

#### ROS link to URDF

The authoritative reference for the XML elements and attributes used in URDF : [XML](http://wiki.ros.org/urdf/XML/)

#### Create appropriate ROS package

This ROS Package that depends on rviz, urdf, xacro, sensor_msgs, and geometry_msgs.

	catkin_create_pkg navvis_description rviz urdf xacro sensor_msgs geometry_msgs
	cd path_to_ROS_package
	mkdir urdf
	gedit urdf/lab2robot.urdf

For building the foundation, just copy the following commands:

	<robot name="navvis">
	<link name="base">
	<visual>
	<geometry>
	<box size="0.45 0.78 1.92" />
	</geometry>
	</visual>
	<collision>
	<geometry>
	<box size="0.45 0.78 1.92" />
	</geometry>
	</collision>
	</link>
	</robot>


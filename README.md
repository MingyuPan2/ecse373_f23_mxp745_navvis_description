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

For all position and orientation parameters, follow the graph on P2.

### Creating a URDF File

#### -ROS link to URDF-

The authoritative reference for the XML elements and attributes used in URDF : [XML](http://wiki.ros.org/urdf/XML/)

#### -Create appropriate ROS package-

This ROS Package that depends on rviz, urdf, xacro, sensor_msgs, and geometry_msgs.

	catkin_create_pkg navvis_description rviz urdf xacro sensor_msgs geometry_msgs
	cd path_to_ROS_package
	mkdir urdf
	gedit urdf/lab2robot.urdf

For building the foundation, just copy the following commands:

	<robot name="navvis">
		<link name="base">
			<visual>
				<origin xyz="0.0 0.0 0.96" rpy="0.0 0.0 0.0" />
			<geometry>
				<box size="0.45 0.78 1.92" />
			</geometry>
			</visual>
			<collision>
				<origin xyz="0.0 0.0 0.96" rpy="0.0 0.0 0.0" />
			<geometry>
				<box size="0.45 0.78 1.92" />
			</geometry>
			</collision>
		</link>
	</robot>

Child links are defined with respect to the base link with its origin offset. Other links and joints can be created using the following commands:
	
	<link name="laser_vert_top_left">
		<visual>
			<origin xyz="0.2997 0.1531 1.8443" rpy="1.1780 0.6198 1.9560" />
			<geometry>
				<box size="0.062 0.062 0.087" />
			</geometry>
		</visual>
		<collision>
			<origin xyz="0.2997 0.1531 1.8443" rpy="1.1780 0.6198 1.9560" />
			<geometry>	
				<box size="0.062 0.062 0.087" />
			</geometry>
		</collision>
	</link>
	
	<joint name="laser_vert_bottom_laser_vert_top_left_joint" type="fixed">
		<parent link="base" />
		<child link="laser_vert_top_left" />
		<origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
	</joint>
	
#### -Add wheels-

Model the wheels and all their respective joints using the same method above. For exact parameters, follow P2 and P10 on Lab 2 pdf.

Remember to update the header file:

	<?xml version="1.0"?>
	<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="<insert_robot_name_here>">
	
Indent the link and joint definitions and a XACRO macro block around them:

	<xacro:macro name="xacro_name" params="side reflect">
	...
	</xacro:macro>
	
To generate a wheel, add and MODIFY the following command to match names:

	<xacro:xacro_name side="left" reflect="1" />

#### -URDF checker-

Use this tool to check the XML of a URDF file. Read the responses to fix any issues  with the URDF XML file.

	sudo apt install liburdfdom-dev liburdfdom-tools ros-noetic-urdfdom-py
	cd catkin_ws/src/navvis_description/urdf
	check_urdf lab2robot.urdf
	rosrun urdfdom_py display_urdf lab2robot.urdf
	
Move on if everything is ok.

## Create Appropriate Launch File

In the package directory, create a folder named `launch`, then create a `display.launch` file in the `launch` folder. The .xacro file WILL NOT be used by default. The joint_gui WILL be used by default. Some code in <?ignore ... ?> were included for reference. Follow the code below to include everything in the file:

	<launch>
		<?ignore
		<arg name = "lab2robot" default = "urdf_from_xacro.urdf" />
		<arg name = "lab2robotfile" default = "$(find navvis_description)/urdf/$(arg urdf_from_xacro)" />
		<param name = "robot_description" textfile = "$(arg lab2robotfile)" />
		<node pkg = "robot_state_publisher" type = "robot_state_publisher" name = "robot_state_publisher" />
		<node pkg = "rviz" type = "rviz" name = "rviz" args = "-d $(find navvis_description)/config/config.rviz" required = "true" />
		?>

		<arg name="use_xacro" default="false" />
		<arg name="joint_gui" default="true" />
		<arg name="rviz_config_file" default="$(find navvis_description)/config/config.rviz" />
		<arg if="$(arg use_xacro)" name="filename" default="lab2robot.xacro" />
		<arg unless="$(arg use_xacro)" name="filename" default="urdf_from_xacro.urdf" />
		<arg name="file" default="$(find navvis_description)/urdf/$(arg filename)" />	
		<param if="$(arg use_xacro)" name="robot_description" command="$(find xacro)/xacro $(arg file)" /> 
		<param unless="$(arg use_xacro)" name="robot_description" textfile="$(arg file)" />
		<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" /> 
		<node pkg="rviz" type="rviz" name="rviz" args="-d $(arg rviz_config_file)" required = "true" />
		<node pkg ="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" if="$(arg joint_gui)" />
		<node pkg ="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" unless="$(arg joint_gui)" />
	</launch>

## Generating URDF From XACRO File

A XACRO file must be processed into URDF for it to be useful. This is accomplished by using the xacro program in the xacro package:

	# Generate URDF from a XACRO file
	rosrun xacro xacro lab2robot.xacro
	
	# Generate URDF from a XACRO file and put it in a file.
	rosrun xacro xacro lab2robot.xacro > urdf_from_xacro.urdf

## View robot in RVIZ using the launch file

To view the robot, first:

	cd catkin_ws
	source devel/setup.bash
	roscore &
	
To view robot WITH the joint_gui:
	
	roslaunch navvis_description display.launch use_xacro:=true &
	
To view robot WITHOUT the joint_gui:
	
	roslaunch navvis_description display.launch use_xacro:=true joint_gui:= false &

To NOT use .xacro file, delete:

	... use_xacro:=true ...

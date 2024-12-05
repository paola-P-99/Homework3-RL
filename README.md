# Homework3-RL
# Overview
The goal of this homework is to implement a vision-based controller for a 7-degrees-of-freedom robotic manipulator arm into the Gazebo environmen using ROS2 
The repository contains instructions to download the folders from GitHub, launch the robot, and run the files.
# Setup
Open the terminal, launch the container, and navigate to the directory where you want to download the folder. Then, clone the repository with:
```
git clone 
```
To build the packages, navigate to the ROS2 workspace and build them with:
```
colcon build 
```
Afterward, source the workspace:
```
source install/setup.bash
```
# Launching
The repository provides two word one with a sphere one with an aruco marker the command to launch each world is the following.
To launch the sphere word 
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true world_file:=sphere.world initial_positions_file:=initial_positions_sphere.yaml
```
To launch the aruco word

```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true world_file:=empty.world initial_positions_file:=initial_positions_aruco.yaml
```
The code provides different command interfaces the one implemented is the velocity , but the default is positioning so in the launch there must be specified the velocity interface.


# sphere detection 
To detect the sphere you have to run the following line of code  in an other terminal connected to the same docker container after launching the sphere word to see the image of the sphere use 
```
ros2 run rqt_image_view rqt_image_view

```
then run the command  in another terminal to see the detection 

```
ros2 run ros2_opencv ros2_opencv_node 
```
# Aruco interaction 
 After launching the aruco word to be able to detect the marker of the aruco run the following line of code 
 
 ```
ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.2 -p reference_frame:=camera_link -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```
 The code provides two kind of controller one is a positioning with an off-set respect to the aruco to see it run the command


```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=velocity -p task_mode_:=positioning
```
 if you want to run the command to use the look at point use the following command to detect the aruco marker 

 ```
ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.2 -p reference_frame:=camera_link_optimal -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```
the run in an other terminal the command to execute the look at point task 
 
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=velocity -p task_mode_:=look_at_point
```

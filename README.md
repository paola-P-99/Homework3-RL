# Homework3-RL
# Overview
The goal of this project is to implement a vision-based controller for a 7-degrees-of-freedom robotic manipulator arm using ROS2.
This repository contains instructions to download the folders from GitHub, launch the robot, and run the files.
# Setup
Open the terminal, launch the container, and navigate to the directory where you want to download the folder. Then, clone the repository with:
```
git clone https://github.com/paola-P-99/Homework3-RL.git
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
The repository provides two worlds to launch in Gazebo: one containing a Sphere model, and the other an aruco marker. 
To launch the first, enter the following command:
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true world_file:=sphere.world initial_positions_file:=initial_positions_sphere.yaml
```

As for the latter:

```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true world_file:=empty.world initial_positions_file:=initial_positions_aruco.yaml
```
The code provides different command interfaces the one implemented is the velocity , but the default is positioning so in the launch there must be specified the velocity interface.

# Sphere detection 
It is possible to visualize the sphere using the camera mounted on the manipulator's end-effector.
In another terminal --- connected to the same docker container --- upon launching the sphere.world, enter:
```
ros2 run rqt_image_view rqt_image_view

```
To detect the Sphere in the world, run in another terminal:

```
ros2 run ros2_opencv ros2_opencv_node 
```
# Aruco interaction 
Following the launch of empty.world, i.e., the world in which the Aruco Marker is located, it is possible to detect such marker running the node:
 
 ```
ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.2 -p reference_frame:=camera_link -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```
 The code provides two kind of controller.One is for positioning with an offset relative to the Aruco marker. To run the positioning command, use:



```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=velocity -p task_mode_:=positioning
```
 To perform the "look at point" task using the Aruco marker, run the marker detection command again with the camera frame set to camera_link_optical: 

 ```
ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.2 -p reference_frame:=camera_link_optimal -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```
Then, in a separate terminal, run the following command to start the "look at point" task:
 
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=velocity -p task_mode_:=look_at_point
```

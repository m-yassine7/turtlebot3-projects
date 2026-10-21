# ECE 7425, Assignment 3 — AprilTag Follower

## Overview

This ROS 2 Jazzy package contains a Python controller for the TurtleBot 3 Burger to detect and follow AprilTags.

The program uses the apriltag_ros package for tag detection and TF to obtain the tag's position. The controller uses this infomation to determine how the robot should move and publishes velocity commands to /cmd_vel. LiDAR data is also used to check for obstacles. 

The goal is for the robot to find an AprilTag, align with it, approach it, and then maintain a distance of approx. 50 cm.

In assignment 3, this package would satisfy part 5 of the requirements.

## Files

- tag_follower.py: Main Python controller for robot movement, tag tracking, and safety checks.
- controller.yaml: Controller parameters, including distances, gains, speed, limits, and timeouts.
- apriltag.yaml: AprilTag detection settings, including tag family, ID, and size.
- matador.launch.py: Launches the AprilTag detector and controller nodes.
- setup.py, setup.cfg, package.xml: Files needed to configure and build the ROS 2 Python package.

## Requirements

The package is intended for Ubuntu 24.04 with ROS 2 Jazzy.

For operation, the following are required:

- TurtleBot3 Burger or a compatible simulation.
- Camera with a calibrated image and camera information topic.
- LiDAR data published to /scan.
- Robot accepting velocity commands through /cmd_vel.
- A valid TF connection between the robot's base_link and the camera frame.
- AprilTag from the 36h11 family, with ID 0 (or the relevant specifications, not sure what ours will be).

The example AprilTag size is set to 0.10 m in apriltag.yaml. This should be changed to match the actual printed tag.

## Building the Package

Install the AprilTag ROS 2 package and build the controller in a ROS 2 workspace:

--
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install ros-jazzy-apriltag-ros python3-colcon-common-extensions

cd ~/ros2_ws
colcon build --symlink-install --packages-select matador_controller
source install/setup.bash
--

The matador_controller package must already be placed inside ~/ros2_ws/src.

## Running the Program

The camera, LiDAR, robot drivers, and necessary TF transforms must already be running.

Start the AprilTag detector and controller:

--
ros2 launch matador_controller matador.launch.py
--

The launch file can also accept different camera topic names if needed.

For a simulation using Gazebo's clock:

--
ros2 launch matador_controller matador.launch.py use_sim_time:=true
--

The simulation and other ROS 2 nodes must use compataible time settings.

## Enable or Stop the Controller

The controller starts disabled for safety.

Enable the controller:

--
ros2 service call /matador/enable std_srvs/srv/SetBool "{data: true}"
--

Stop the controller:

--
ros2 service call /matador/enable std_srvs/srv/SetBool "{data: false}"
--

Check the robot's current state:

--
ros2 topic echo /matador/state
--

Check the velocity commands:

--
ros2 topic echo /cmd_vel
--

## Controller Operation

The controller uses different states depending on the position of the tag and the sensor readings.

- SEARCH: The robot rotates slowly until it finds the target AprilTag. Searching stops after 10 seconds if the tag is not detected.
- ALIGN: The robot rotates until it is facing within 5 degrees of the tag.
- APPROACH: The robot moves forward when the tag is farther than 0.55 m. Proportional control is used to calculate the forward and turning speeds.
- HOLD: The robot stops when the tag is between 0.45 m and 0.55 m. It also stops if the tag is closer than 0.45 m.
- LOST_TARGET: The robot stops if the tracked tag disappears or its detection becomes outdated.
- SAFETY_STOP: The robot stops if an obstacle is too close, sensor data is invalid or outdated, or the operator stops the controller. 

The controller must be enabled again after a timeout, loss of the target, or a safety stop.

## Proportional Control

The forward speed is calculated using the distance error:

--
error = distance - self.target_distance
forward = self.distance_gain * error
--

The turning speed is calculated using the angle to the tag:

--
turn = self.angular_gain * bearing
--

Both speeds are limited using the maximum values specified in controller.yaml.

## Notes

The assignment's initial description mentions reversing when the tag is too close. However, the detailed controller requirements in the table specify stopping between 0.45 m. This implementation follows the detailed requirements and doesn't reverse.

The controller has not yet been tested on a physical TurtleBot3 or in Gazebo. Its behavior will need to be verified in a ROS 2 Jazzy environment with the required camera, LiDAR, TF data, etc.

## References:

- ROS 2 Jazzy Documentation
- ROS 2 Python Publisher and Subscriber Tutorial
- ROS 2 Launch File Tutorial
- Christian Rauch - AprilTag ROS 2
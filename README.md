# TidyBot - colour_chaser (OpenCV & Ros2)
This system implements a robot behaviour to "Tidy", using image processing and computer vision to trigger an AgileX Limo robot to target and push coloured objects.


# Project overview

I have created a 'colour-chaser' node in a ROS2 package which:
- subscribes to the camera feed
- Detects red and green objects using HSV colour segmentation and contours
- Alligns itself with the objects center and moves forward for 5 seconds
- Then rotates for 3 seconds and begins its search again

# Requirements

-ROS2
- Code editor with DevContainer image from "lcas.lincoln.ac.uk/devcontainer/ros2-teaching:4"
-ROS2 packages: cv_bridge, sensor_msgs, geometry_msgs and rclpy

# How to build

1. Open the project folder in the DevContainer
2. within the terminal run:
- colcon build
- source install/setup.bash

# Launch the simulation and node
1. In the terminal run:
- ros2 launch uol_tidybot tidybot.launch.py
2. In a new terminal run:
- ros2 run py_pubsub colour_chaser

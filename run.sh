#!/bin/bash

# Source ROS2 and local install
source /opt/ros/iron/setup.bash
source install/setup.bash

# Run your node here (replace with actual node name if needed)
ros2 run waterlinked_3d_sonar_driver sonar_3d_15_node.py

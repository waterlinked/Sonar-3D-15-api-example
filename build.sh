#!/bin/bash
set -e

# Build the ROS2 package
colcon build --symlink-install

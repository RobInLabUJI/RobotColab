#!/bin/bash

export ROS_DOMAIN_ID=$(ip route get 8.8.8.8 | sed -n '/src/{s/.*src *\([^ ]*\).*/\1/p;q}' | cut -d . -f 4)
source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02

ros2 launch turtlebot3_bringup robot.launch.py


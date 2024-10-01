#!/bin/bash

export ROS_DOMAIN_ID=$1
source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard


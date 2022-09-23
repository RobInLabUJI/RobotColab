#!/bin/bash

export ROS_DOMAIN_ID=$(ip route get 8.8.8.8 | sed -n '/src/{s/.*src *\([^ ]*\).*/\1/p;q}' | cut -d . -f 4)
source /opt/ros/foxy/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard


#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=$(ip route get 8.8.8.8 | sed -n '/src/{s/.*src *\([^ ]*\).*/\1/p;q}' | cut -d . -f 4)
ros2 run camera_ros camera_node --ros-args -p width:=160 -p height:=120 -p format:=BGR888


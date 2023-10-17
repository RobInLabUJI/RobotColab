#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

export ROS_HOSTNAME=$(ip route get 8.8.8.8 | sed -n '/src/{s/.*src *\([^ ]*\).*/\1/p;q}')
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
export PATH=$PATH:/home/ubuntu/.local/bin
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02

roslaunch turtlebot3_bringup turtlebot3_robot.launch

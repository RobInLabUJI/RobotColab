#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash

export ROS_HOSTNAME=$(ip route get 8.8.8.8 | sed -n '/src/{s/.*src *\([^ ]*\).*/\1/p;q}')
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
export PATH=$PATH:/home/pi/.local/bin

roslaunch /home/pi/RobotColab/ROS/Pioneer3/usb_cam.launch

#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
export PATH=$PATH:/home/pi/.local/bin

rostopic pub -1 /cmd_motor_state p2os_msgs/MotorState 1 &

cd /home/pi/RobotColab/ROS && \
jupyter notebook --NotebookApp.allow_origin='https://colab.research.google.com' \
                 --port=8888 --NotebookApp.port_retries=0 --no-browser \
                 --NotebookApp.token='' --ip 0.0.0.0


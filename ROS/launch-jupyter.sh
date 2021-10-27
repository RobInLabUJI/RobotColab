#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
export PATH=$PATH:/home/ubuntu/.local/bin

cd /home/ubuntu/RobotColab/ROS && \
jupyter notebook --NotebookApp.allow_origin='https://colab.research.google.com' \
                 --port=8888 --NotebookApp.port_retries=0 --no-browser \
                 --NotebookApp.token='' --ip 0.0.0.0 &


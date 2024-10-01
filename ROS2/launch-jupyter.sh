#!/usr/bin/env bash

export ROS_DOMAIN_ID=$(ip route get 8.8.8.8 | sed -n '/src/{s/.*src *\([^ ]*\).*/\1/p;q}' | cut -d . -f 4)
source /opt/ros/humble/setup.bash
export PATH=$PATH:/home/ubuntu/.local/bin

cd /home/ubuntu/RobotColab/ROS2 && \
jupyter notebook --NotebookApp.allow_origin='https://colab.research.google.com' \
                 --port=8888 --NotebookApp.port_retries=0 --no-browser \
                 --NotebookApp.token='' --ip 0.0.0.0


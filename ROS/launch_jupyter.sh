#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
jupyter notebook --NotebookApp.allow_origin='https://colab.research.google.com' \
                 --port=8888 --NotebookApp.port_retries=0 --no-browser \
                 --NotebookApp.token=''


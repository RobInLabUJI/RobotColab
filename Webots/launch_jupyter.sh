#!/usr/bin/env bash

export WEBOTS_HOME=${HOME}/webots
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${WEBOTS_HOME}/lib/controller
export PYTHONPATH=${PYTHONPATH}:${WEBOTS_HOME}/lib/controller/python38
export PYTHONIOENCODING=UTF-8
jupyter notebook --NotebookApp.allow_origin='https://colab.research.google.com' \
                 --port=8888 --NotebookApp.port_retries=0 --no-browser \
                 --NotebookApp.token=''


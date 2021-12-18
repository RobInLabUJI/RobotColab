#!/usr/bin/env bash

rocker --nvidia --x11 --network host \
  --name tb3sim robinlab/tb3sim /scripts/launch_tb3sim_empty.sh


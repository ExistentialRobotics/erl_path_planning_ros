#! /usr/bin/bash

docker build --rm -t erl/ros-noetic:cpu-path-planning . \
  --build-arg BASE_IMAGE=erl/path_planning:20.04 $@

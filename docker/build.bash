#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd $(dirname $0); pwd)
BUILD_ROS_NOETIC=${BUILD_ROS_NOETIC:-true}
BUILD_ROS_HUMBLE=${BUILD_ROS_HUMBLE:-true}

if [ "${BUILD_ROS_NOETIC}" = "true" ]; then
  echo "Building ROS image..."
  cd $SCRIPT_DIR/ros-noetic
  ./build.bash $@
fi

if [ "${BUILD_ROS_HUMBLE}" = "true" ]; then
  echo "Building ROS Humble image..."
  cd $SCRIPT_DIR/ros-humble
  ./build.bash $@
fi

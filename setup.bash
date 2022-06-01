#!/usr/bin/env bash

THIS_FILE=$BASH_SOURCE
THIS_PROJECT_ROOT=$(realpath $(dirname $(realpath $THIS_FILE)))

# Install ROS2 dependency
unset colcon_ws
rosdep update
rosdep install -r -y -i --from-paths ${THIS_PROJECT_ROOT} --rosdistro $ROS_DISTRO

unset THIS_FILE
unset THIS_PROJECT_ROOT

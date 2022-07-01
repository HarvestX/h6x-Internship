#!/usr/bin/env bash

THIS_FILE=$BASH_SOURCE
THIS_PROJECT_ROOT=$(realpath $(dirname $(realpath $THIS_FILE)))

# Install local repo dependency
colcon_ws=${THIS_PROJECT_ROOT}/../../
cd ${colcon_ws}/src
vcs import --recursive <${THIS_PROJECT_ROOT}/rvizplugin.repos
unset colcon_ws

# Install ROS2 dependency
rosdep update
rosdep install -r -y -i --from-paths ${THIS_PROJECT_ROOT} --rosdistro $ROS_DISTRO

unset THIS_FILE
unset THIS_PROJECT_ROOT

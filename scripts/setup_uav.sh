#!/bin/bash

sudo apt-get install ros-"${ROS_DISTRO}"-mavros ros-"${ROS_DISTRO}"-mavros-extras ros-"${ROS_DISTRO}"-mavros-msgs ros-"${ROS_DISTRO}"-vrpn ros-"${ROS_DISTRO}"-realsense2-camera

wget -O - https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

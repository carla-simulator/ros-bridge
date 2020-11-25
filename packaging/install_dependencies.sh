#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_VERSION=$(rosversion -d)
if [ "$ROS_VERSION" = "noetic" ]; then
    PYTHON_SUFFIX=3
else
    PYTHON_SUFFIX=""
fi

sudo apt update
sudo apt-get install --no-install-recommends -y \
    python$PYTHON_SUFFIX-pip \
    python$PYTHON_SUFFIX-osrf-pycommon \
    python$PYTHON_SUFFIX-catkin-tools \
    python$PYTHON_SUFFIX-catkin-pkg \
    python$PYTHON_SUFFIX-catkin-pkg-modules \
    python$PYTHON_SUFFIX-rosdep \
    python$PYTHON_SUFFIX-wstool \
    ros-$ROS_VERSION-opencv-apps \
    ros-$ROS_VERSION-ackermann-msgs \
    ros-$ROS_VERSION-derived-object-msgs \
    ros-$ROS_VERSION-cv-bridge \
    ros-$ROS_VERSION-vision-opencv \
    ros-$ROS_VERSION-rospy-message-converter \
    ros-$ROS_VERSION-rviz \
    ros-$ROS_VERSION-rqt-image-view \
    ros-$ROS_VERSION-rqt-gui-py \
    ros-$ROS_VERSION-rviz \
    qt5-default \
    ros-$ROS_VERSION-pcl-conversions \
    ros-$ROS_VERSION-pcl-ros

pip$PYTHON_SUFFIX install -r $SCRIPT_DIR/../requirements.txt

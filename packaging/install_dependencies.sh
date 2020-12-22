#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_VERSION=$(rosversion -d)
if [ "$ROS_VERSION" = "noetic" -o "$ROS_VERSION" = "foxy" ]; then
    PYTHON_SUFFIX=3
else
    PYTHON_SUFFIX=""
fi

if [ "$ROS_VERSION" = "foxy" ]; then
    ADDITIONAL_PACKAGES=ros-$ROS_VERSION-rviz2
else
    ADDITIONAL_PACKAGES=ros-$ROS_VERSION-rviz \
        ros-$ROS_VERSION-opencv-apps \
        ros-$ROS_VERSION-rospy-message-converter \
        ros-$ROS_VERSION-pcl-ros
fi
echo ADDITIONAL PACKAGES $ADDITIONAL_PACKAGES

sudo apt update
sudo apt-get install --no-install-recommends -y \
    python$PYTHON_SUFFIX-pip \
    python$PYTHON_SUFFIX-osrf-pycommon \
    python$PYTHON_SUFFIX-catkin-tools \
    python$PYTHON_SUFFIX-catkin-pkg \
    python$PYTHON_SUFFIX-catkin-pkg-modules \
    python$PYTHON_SUFFIX-rosdep \
    python$PYTHON_SUFFIX-wstool \
    ros-$ROS_VERSION-ackermann-msgs \
    ros-$ROS_VERSION-derived-object-msgs \
    ros-$ROS_VERSION-cv-bridge \
    ros-$ROS_VERSION-vision-opencv \
    ros-$ROS_VERSION-rqt-image-view \
    ros-$ROS_VERSION-rqt-gui-py \
    qt5-default \
    ros-$ROS_VERSION-pcl-conversions \
    $ADDITIONAL_PACKAGES

pip$PYTHON_SUFFIX install -r $SCRIPT_DIR/../requirements.txt

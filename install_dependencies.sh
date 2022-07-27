#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SUFFIX=""
if [ "$ROS_PYTHON_VERSION" = "3" ]; then
    PYTHON_SUFFIX=3
fi

if [ "$ROS_VERSION" = "2" ]; then
    ADDITIONAL_PACKAGES="ros-$ROS_DISTRO-rviz2"
else
    ADDITIONAL_PACKAGES="ros-$ROS_DISTRO-rviz
                         ros-$ROS_DISTRO-opencv-apps
                         ros-$ROS_DISTRO-rospy
                         ros-$ROS_DISTRO-rospy-message-converter
                         ros-$ROS_DISTRO-pcl-ros"
fi

if [ "$(lsb_release -sc)" = "focal" ]; then
    ADDITIONAL_PACKAGES="$ADDITIONAL_PACKAGES
                         python-is-python3"
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
    python$PYTHON_SUFFIX-opencv \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-derived-object-msgs \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-rqt-image-view \
    ros-$ROS_DISTRO-rqt-gui-py \
    wget \
    qt5-default \
    ros-$ROS_DISTRO-pcl-conversions \
    $ADDITIONAL_PACKAGES

pip$PYTHON_SUFFIX install --upgrade pip$PYTHON_SUFFIX
pip$PYTHON_SUFFIX install -r $SCRIPT_DIR/requirements.txt

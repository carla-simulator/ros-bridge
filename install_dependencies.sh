#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SUFFIX=""
if [ "$ROS_PYTHON_VERSION" = "3" ]; then
    PYTHON_SUFFIX=3
fi

ADDITIONAL_PACKAGES="python$PYTHON_SUFFIX-catkin-pkg \
    python$PYTHON_SUFFIX-catkin-pkg-modules \
    python$PYTHON_SUFFIX-opencv \
    python$PYTHON_SUFFIX-osrf-pycommon \
    python$PYTHON_SUFFIX-pip \
    python$PYTHON_SUFFIX-rosdep \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-derived-object-msgs \
    ros-$ROS_DISTRO-etsi-its-cam-msgs \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-rqt-gui-py \
    ros-$ROS_DISTRO-rqt-image-view \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-tf2-eigen \
    ros-$ROS_DISTRO-vision-opencv \
    wget"

if [ "$(lsb_release -sc)" = "focal" ]; then
    ADDITIONAL_PACKAGES="$ADDITIONAL_PACKAGES \
                         python-is-python3 \
                         python$PYTHON_SUFFIX-catkin-tools \
                         python$PYTHON_SUFFIX-wstool \
                         qt5-default"
elif [ "$(lsb_release -sc)" = "jammy" ] || [ "$(lsb_release -sc)" = "noble" ]; then
    ADDITIONAL_PACKAGES="$ADDITIONAL_PACKAGES \
                         qtbase5-dev qt5-qmake"
fi


echo "ADDITIONAL PACKAGES to be installed: $ADDITIONAL_PACKAGES"

sudo apt update
sudo apt-get install --no-install-recommends -y $ADDITIONAL_PACKAGES


FOCRE_PYTHON_PACKAGES=""
if [ "$(lsb_release -sc)" = "noble" ]; then
    # On Ubuntu 24.04 (Noble), pip enforces stricter package management which may
    # conflict with system packages. The --break-system-packages flag allows pip
    # to install packages even if they might break system packages.
    # Alternative way of using virtual environment is not working with ROS2 installation.
    # (test it out with 'ros2 launch carla_manual_control carla_manual_control.launch.py')
    # If someone knows better solution, please update this script.
    FOCRE_PYTHON_PACKAGES="--break-system-packages"
fi

python$PYTHON_SUFFIX -m pip install ${FOCRE_PYTHON_PACKAGES} --upgrade pip
python$PYTHON_SUFFIX -m pip install ${FOCRE_PYTHON_PACKAGES} -r $SCRIPT_DIR/requirements.txt
 
#!/bin/sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_DISTRO="foxy"
CARLA_VERSION=$(cat ${SCRIPT_DIR}/../carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION)

while getopts r:c: flag
do
    case "${flag}" in
        r) ROS_DISTRO=${OPTARG};;
        c) CARLA_VERSION=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker build \
    -t carla-ros-bridge:$ROS_DISTRO \
    -f Dockerfile ${SCRIPT_DIR}/.. \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --build-arg CARLA_VERSION=$CARLA_VERSION

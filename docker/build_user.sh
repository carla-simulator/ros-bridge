#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_DISTRO="humble"
CARLA_VERSION=$(cat ${SCRIPT_DIR}/../carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION)
USERNAME=${USER}
UID=$(id -u)
GID=$(id -g)

while getopts r:c: flag
do
    case "${flag}" in
        r) ROS_DISTRO=${OPTARG};;
        c) CARLA_VERSION=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

if [[ "$ROS_DISTRO" == "humble" ]]
then
docker build \
    -t carla-ros-bridge:$ROS_DISTRO \
    -f Dockerfile_humble_user ${SCRIPT_DIR}/.. \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --build-arg CARLA_VERSION=$CARLA_VERSION \
    --build-arg USERNAME=$USERNAME \
    --build-arg UID=$UID \
    --build-arg GID=$GID
else
docker build \
    -t carla-ros-bridge:$ROS_DISTRO \
    -f Dockerfile ${SCRIPT_DIR}/.. \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --build-arg CARLA_VERSION=$CARLA_VERSION
fi
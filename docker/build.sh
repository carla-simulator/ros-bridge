#!/bin/sh

ROS_DISTRO="foxy"
CARLA_VERSION="0.9.10"

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
    -f Dockerfile ./.. \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --build-arg CARLA_VERSION=$CARLA_VERSION

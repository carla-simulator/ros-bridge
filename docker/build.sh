#!/bin/sh

docker build -t carla-ros-bridge -f Dockerfile ./.. "$@"

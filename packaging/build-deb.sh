#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR=${SCRIPT_DIR}/build/$(rosversion -d)
WORKSPACE_DIR=${BUILD_DIR}/catkin_ws

mkdir -p ${BUILD_DIR} ${WORKSPACE_DIR}

# bring the code
rsync -r ${SCRIPT_DIR}/../* ${WORKSPACE_DIR}/src --exclude packaging
# init ros workspace
source /opt/ros/$(rosversion -d)/setup.bash
cd ${WORKSPACE_DIR} && catkin_init_workspace
# add debian files
cp -r ${SCRIPT_DIR}/debian ${WORKSPACE_DIR}
sed -i "s/{ROS_DIST}/$(rosversion -d)/g" ${WORKSPACE_DIR}/debian/control
# build debian package
cd ${WORKSPACE_DIR} && dpkg-buildpackage -uc -us -b

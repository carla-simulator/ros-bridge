#!/usr/bin/env bash

set -e

if [ "${ROS_VERSION}" != "1" ]; then
    echo "Only ROS1 packages are supported"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR=${PWD}/build/$(rosversion -d)
WORKSPACE_DIR=${BUILD_DIR}/ws
PYTHON_SUFFIX=""
if [ "$ROS_PYTHON_VERSION" = "3" ]; then
    PYTHON_SUFFIX=3
fi

sudo apt update
sudo apt-get install --no-install-recommends -y \
    rsync \
    build-essential \
    dh-make

rm -rf ${WORKSPACE_DIR}
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

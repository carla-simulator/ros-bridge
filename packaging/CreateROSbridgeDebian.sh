#!/bin/sh

#This script builds debian package for CARLA ROS Bridge
#Tested with Ubuntu 14.04, 16.04, 18.04 and 19.10

sudo apt-get install build-essential dh-make

#Adding maintainer name
DEBFULLNAME=Carla\ Simulator\ Team
export DEBFULLNAME

#carla-ros-bridge github repository
ROS_BRIDGE_GIT=https://github.com/carla-simulator/ros-bridge.git

mkdir -p carla-ros-debian/carla-ros-bridge/catkin_ws/src
mkdir -p carla-ros-debian/carla-ros-bridge-$(rosversion -d)-0.9.8/carla-ros-bridge/

cd carla-ros-debian/carla-ros-bridge

#cloning carla-ros-bridge git repository
git clone ${ROS_BRIDGE_GIT}
rm -r ros-bridge/carla_msgs
cp -a ros-bridge/* catkin_ws/src/

cd catkin_ws

source /opt/ros/$(rosversion -d)/setup.bash

#installing required dependency packages to build catkin_make
sudo apt install ros-$(rosversion -d)-derived-object-msgs \
ros-$(rosversion -d)-ackermann-msgs ros-$(rosversion -d)-carla-msgs ros-$(rosversion -d)-pcl-conversions \
ros-$(rosversion -d)-rviz ros-$(rosversion -d)-rqt ros-$(rosversion -d)-pcl-ros

catkin_make install

cp -r install ../../carla-ros-bridge-$(rosversion -d)-0.9.8/carla-ros-bridge/

cd ../../carla-ros-bridge-$(rosversion -d)-0.9.8/
mv carla-ros-bridge/install carla-ros-bridge/$(rosversion -d)

rm Makefile

#Making debian package to install Carla-ros-bridge in /opt folder
cat >> Makefile <<EOF
binary:
	# we are not going to build anything
install:
	mkdir -p \$(DESTDIR)/opt/carla-ros-bridge
	cp -r carla-ros-bridge/$(rosversion -d) \$(DESTDIR)/opt/carla-ros-bridge/
EOF

dh_make -e carla.simulator@gmail.com --indep --createorig -y  #to create necessary file structure for debian packaging

cd debian/

#removing unnecessary files
rm ./*.ex
rm ./*.EX

rm control

#Adding package dependencies(Package will install them itself) and description
cat >> control <<EOF
Source: carla-ros-bridge-$(rosversion -d)
Section: misc
Priority: optional
Maintainer: Carla Simulator Team <carla.simulator@gmail.com>
Build-Depends: debhelper (>= 9)
Standards-Version:0.9.7
Homepage: https://github.com/carla-simulator/ros-bridge

Package: carla-ros-bridge-$(rosversion -d)
Architecture: amd64
Depends: ros-$(rosversion -d)-carla-msgs,
	 ros-$(rosversion -d)-roslaunch,
	 ros-$(rosversion -d)-catkin,
	 ros-$(rosversion -d)-rospy,
	 ros-$(rosversion -d)-nav-msgs,
	 ros-$(rosversion -d)-ackermann-msgs,
	 ros-$(rosversion -d)-std-msgs,
	 ros-$(rosversion -d)-dynamic-reconfigure,
	 ros-$(rosversion -d)-topic-tools,
	 ros-$(rosversion -d)-sensor-msgs,
	 ros-$(rosversion -d)-message-runtime,
	 ros-$(rosversion -d)-geometry-msgs,
	 ros-$(rosversion -d)-message-generation,
	 ros-$(rosversion -d)-visualization-msgs,
	 ros-$(rosversion -d)-tf,
	 ros-$(rosversion -d)-tf2,
	 ros-$(rosversion -d)-rviz,
	 ros-$(rosversion -d)-cv-bridge,
	 ros-$(rosversion -d)-rosbag-storage,
	 ros-$(rosversion -d)-derived-object-msgs,
	 ros-$(rosversion -d)-shape-msgs,
	 ros-$(rosversion -d)-tf2-msgs,
	 ros-$(rosversion -d)-rosgraph-msgs,
	 ros-$(rosversion -d)-roscpp,
	 ros-$(rosversion -d)-pcl-conversions,
	 ros-$(rosversion -d)-pcl-ros,
	 ros-$(rosversion -d)-rostopic,
	 ros-$(rosversion -d)-rqt-gui-py
Description: The carla_ros_bridge package aims at providing a simple ROS bridge for CARLA simulator.
EOF

rm copyright
cp ../../../carla-ros-bridge/ros-bridge/LICENSE ./copyright


#Updating debian/Changelog
awk '{sub(/UNRELEASED/,"stable")}1' changelog > tmp && mv tmp changelog
awk '{sub(/unstable/,"stable")}1' changelog > tmp && mv tmp changelog

cd ..

dpkg-buildpackage -uc -us -b #building debian package

#install debian package using "sudo dpkg -i ../carla_ros-bridge-<rosversion>_amd64.deb"


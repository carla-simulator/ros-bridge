#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# This node subscribes to the VehicleControl topic.
# - If data is received, the ROS bridge gets started.
# - If no data is received for a specific time, the ROS bridge gets stopped.
#
import rospy
import roslaunch
import sys
import threading
import datetime

from carla_ros_bridge.msg import CarlaVehicleControl  # pylint: disable=no-name-in-module,import-error

class RosBridgeLifecycle(object):

    def callback(self, data):
        self.lastMsgReceived = datetime.datetime.now()

    def __init__(self):
        self.lastMsgReceived = datetime.datetime(datetime.MINYEAR, 1, 1)
        self.lock = threading.Lock() #to prevent start/stop while previous action was not finished
        self.is_active = False
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.process = None
        roslaunch.configure_logging(self.uuid)
        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaVehicleControl, self.callback)

    def stop_ros_bridge(self):
        with self.lock:
            self.is_active = False
            rospy.loginfo("Stopping ros bridge")
            self.launch.shutdown()

    def run(self):
        while not rospy.core.is_shutdown():
            if self.lock.acquire(False):
                if not self.is_active and (self.lastMsgReceived + datetime.timedelta(0, 5)) > datetime.datetime.now():
                    self.is_active = True
                    rospy.loginfo("Starting ros bridge")
                    self.node = roslaunch.core.Node("carla_ros_bridge", "client.py")
                    self.launch = roslaunch.scriptapi.ROSLaunch()
                    self.launch.start()
                    self.process = self.launch.launch(self.node)
                if self.is_active and (self.lastMsgReceived + datetime.timedelta(0, 5)) < datetime.datetime.now():
                    self.is_active = False
                    rospy.loginfo("Stopping ros bridge")
                    self.process.stop()
                    self.launch.stop()
                    self.process = None
                    self.launch = None
                self.lock.release()

            rospy.rostime.wallsleep(0.5)

    def __del__(self):
        if self.is_active:
            if self.process and self.process.is_alive():
                self.process.stop()
            if self.launch:
                self.launch.stop()

def main():
    rospy.init_node('ros_bridge_lifecycle')
    timeout = rospy.get_param('/carla/ros_bridge/timeout', '5') # default: 5 seconds
    lifecycle = RosBridgeLifecycle()
    try:
        lifecycle.run()
    finally:
        del lifecycle
        rospy.loginfo("Done")

if __name__ == "__main__":
    main()


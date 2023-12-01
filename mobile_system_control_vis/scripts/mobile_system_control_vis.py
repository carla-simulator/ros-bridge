#!/usr/bin/python3

import rospy
from math import pi, cos, sin
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
# import numpy as np
import csv


class TrackVisualizer:
    def __init__(self):
        self.pub_track = rospy.Publisher("~track", Marker, queue_size=10)
        self.sub_pose = rospy.Subscriber("~pose", Float32MultiArray, self.posecb, queue_size=10)
        self.pub_vehicle = rospy.Publisher("~vehicle", Marker, queue_size=10)
        self.csv_dir = rospy.get_param('~csv_dir')

        self.track = Marker()
        self.track.type = self.track.SPHERE_LIST
        self.track.header.frame_id = "map"
        self.track.id = 0
        self.track.color.a = 0.8
        self.track.color.r = 0
        self.track.color.g = 1.0
        self.track.color.b = 0
        self.track.scale.x = 0.8
        self.track.scale.y = 0.8
        self.track.scale.z = 0.8
        self.track.pose.position.x = 0
        self.track.pose.position.y = 0
        self.track.pose.position.z = 0
        self.track.pose.orientation.x = 0
        self.track.pose.orientation.y = 0
        self.track.pose.orientation.z = 0
        self.track.pose.orientation.w = 1

        self.vehicle = Marker()
        self.vehicle.type = self.vehicle.ARROW
        self.vehicle.header.frame_id = "map"
        self.vehicle.color.a = 1.0
        self.vehicle.color.r = 1.0
        self.vehicle.color.g = 0
        self.vehicle.color.b = 0
        self.vehicle.scale.x = 4
        self.vehicle.scale.y = 1.0
        self.vehicle.scale.z = 2

    def map_client(self):
        self.track.points=[]
        f = open(self.csv_dir,'r')
        rdr = csv.reader(f)

        for line in rdr:
            p = Point()
            p.x = float(line[0])
            p.y = float(line[1])
            p.z = 10.0
            self.track.points.append(p)
            # print(p)

    def posecb(self, data):
        # self.vehicle.points=[]
        # p = Point()
        # p.x = 
        # p.y = 
        # p.z = 10.0
        # self.vehicle.points.append(p)

        self.vehicle.pose.position.x = data.data[0]
        self.vehicle.pose.position.y = data.data[1]
        self.vehicle.pose.position.z = 0
        self.vehicle.pose.orientation.x = 0
        self.vehicle.pose.orientation.y = 0
        self.vehicle.pose.orientation.z = sin(data.data[2] / 2)
        self.vehicle.pose.orientation.w = cos(data.data[2] / 2)
        # print(data.data)

    def publish(self):
        self.pub_track.publish(self.track)
        self.pub_vehicle.publish(self.vehicle)

def main():
    rospy.init_node("mobile_system_control_vis", anonymous=True)
    vis = TrackVisualizer()
    vis.map_client()
    rate = rospy.Rate(10)  # 20hz
    while not rospy.is_shutdown():
        vis.publish()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaWorldInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import ad_map_access as ad
from std_msgs.msg import ColorRGBA

import xml.etree.ElementTree as ET

class CarlaMapVisualizer(Node):
    def __init__(self):
        super().__init__('carla_map_visualizer')

        # Colors: [R, G, B, A]
        self.COLOR_NORMAL = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)       # Blue
        self.COLOR_INTERSECTION = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8) # Green
        self.COLOR_SHOULDER = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.4)     # Grey
        self.COLOR_PEDESTRIAN = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)    # Yellow
        self.COLOR_BIKE = ColorRGBA(r=1.0, g=0.65, b=0.0, a=0.8)        # Orange
        self.COLOR_OTHER = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.8)       # Light Grey

        self.world_info_sub = self.create_subscription(
            CarlaWorldInfo,
            '/carla/world_info',
            self.world_info_callback,
            rclpy.qos.QoSProfile(depth=1,
                                 durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

        self.marker_pub = self.create_publisher(
            MarkerArray, '/carla/world_info/lane_markers',
            rclpy.qos.QoSProfile(depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.get_logger().info("Carla Map Visualizer ready.")


    def remove_offset_from_opendrive(self, opendrive_str):
        # Parse the XML string into an ElementTree
        root = ET.fromstring(opendrive_str)

        # Find the header, then find and remove the offset tag
        header = root.find('header')
        if header is not None:
            offset = header.find('offset')
            if offset is not None:
                header.remove(offset)

        # Convert the XML tree back to a string
        cleaned_xml = ET.tostring(root, encoding='unicode', xml_declaration=True)
        return cleaned_xml


    def world_info_callback(self, msg):
        if not msg.opendrive:
            return

        # in case the OpenDrive content contains an offset, we need to remove it since CARLA is using this offset
        # only for the GPS Sensor, but the actual map data is in the ENU frame without the offset applied.
        #  If we don't remove the offset, the displayed map will be misaligned.
        clean_opendrive = self.remove_offset_from_opendrive(msg.opendrive)

        self.get_logger().info("Processing map...")
        if not ad.map.access.initFromOpenDriveContent(
            clean_opendrive, 0.2,
            ad.map.intersection.IntersectionType.PriorityToRight,
            ad.map.landmark.TrafficLightType.UNKNOWN):
            self.get_logger().error("Failed to initialize map from OpenDrive content.")
            return

        self.publish_lanes()

    def create_line_marker(self, marker_id, lane_type, edge_ecef_points, side_name):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"lane_{side_name}"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15 # Line thickness

        # Color logic based on LaneType
        if lane_type == ad.map.lane.LaneType.INTERSECTION:
            marker.color = self.COLOR_INTERSECTION
        elif lane_type == ad.map.lane.LaneType.NORMAL:
            marker.color = self.COLOR_NORMAL
        elif lane_type == ad.map.lane.LaneType.SHOULDER:
            marker.color = self.COLOR_SHOULDER
        elif lane_type == ad.map.lane.LaneType.PEDESTRIAN:
            marker.color = self.COLOR_PEDESTRIAN
        elif lane_type == ad.map.lane.LaneType.BIKE:
            marker.color = self.COLOR_BIKE
        else:
            marker.color = self.COLOR_OTHER

        for pt in edge_ecef_points:
            enu = ad.map.point.toENU(pt)
            ros_pt = Point(x=enu.x.mENUCoordinate, y=enu.y.mENUCoordinate, z=enu.z.mENUCoordinate)
            marker.points.append(ros_pt)

        return marker

    def publish_lanes(self):
        marker_array = MarkerArray()
        all_lanes = ad.map.lane.getLanes()

        for lane_id in all_lanes:
            lane = ad.map.lane.getLane(lane_id)

            # Create markers for both left and right boundaries
            marker_array.markers.append(
                self.create_line_marker(lane_id.mLaneId, lane.type, lane.edge_left.ecef_points, "left")
            )
            marker_array.markers.append(
                self.create_line_marker(lane_id.mLaneId, lane.type, lane.edge_right.ecef_points, "right")
            )

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Visualized {len(all_lanes)} lanes with intersection highlighting.")

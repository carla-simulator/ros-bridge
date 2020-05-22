import os
ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))
if ROS_VERSION == 1:
    from ros_compatible_node import *
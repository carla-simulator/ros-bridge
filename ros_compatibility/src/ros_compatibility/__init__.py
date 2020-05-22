import os
ROS_VERSION = int(os.environ['ROS_VERSION'])
if ROS_VERSION == 1:
    from ros_compatible_node import *
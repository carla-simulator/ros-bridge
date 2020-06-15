ARG CARLA_VERSION='0.9.6'
ARG ROS_VERSION='kinetic'
ARG CARLA_BUILD=''

FROM carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla

FROM ros:$ROS_VERSION-ros-base

ARG ROS_VERSION

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    libpng16-16 \
    ros-$ROS_VERSION-ackermann-msgs \
    ros-$ROS_VERSION-derived-object-msgs \
    ros-$ROS_VERSION-tf \
    ros-$ROS_VERSION-cv-bridge \
    ros-$ROS_VERSION-pcl-conversions \
    ros-$ROS_VERSION-pcl-ros \
    python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

ARG CARLA_VERSION

COPY --from=carla --chown=root /home/carla/PythonAPI /opt/carla/PythonAPI
RUN python -m easy_install /opt/carla/PythonAPI/carla/dist/carla-$CARLA_VERSION-py2.7-linux-x86_64.egg

COPY carla_ackermann_control /opt/carla-ros-bridge/src/carla_ackermann_control
COPY carla_ego_vehicle /opt/carla-ros-bridge/src/carla_ego_vehicle
COPY carla_infrastructure /opt/carla-ros-bridge/src/carla_infrastructure
COPY carla_manual_control /opt/carla-ros-bridge/src/carla_manual_control
COPY carla_msgs /opt/carla-ros-bridge/src/carla_msgs
COPY carla_ros_bridge /opt/carla-ros-bridge/src/carla_ros_bridge
COPY carla_waypoint_publisher /opt/carla-ros-bridge/src/carla_waypoint_publisher
COPY pcl_recorder /opt/carla-ros-bridge/src/pcl_recorder

RUN /bin/bash -c 'source /opt/ros/$ROS_VERSION/setup.bash; cd /opt/carla-ros-bridge; catkin config --install; catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /

CMD ["roslaunch", "carla_ros_bridge", "carla_ros_bridge.launch"]

ARG CARLA_VERSION='0.9.4'
ARG CARLA_BUILD=''

FROM carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla

FROM ros:kinetic-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    libpng16-16 \
    ros-kinetic-ackermann-msgs \
    ros-kinetic-derived-object-msgs \
    ros-kinetic-tf \
    ros-kinetic-cv-bridge \
    ros-kinetic-pcl-conversions \
    ros-kinetic-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

ARG CARLA_VERSION

COPY --from=carla --chown=root /home/carla/PythonAPI /opt/carla/PythonAPI
ENV PYTHONPATH=/opt/carla/PythonAPI/carla-$CARLA_VERSION-py2.7-linux-x86_64.egg:/opt/carla/PythonAPI/carla

COPY carla_ackermann_control /opt/carla-ros-bridge/src/carla_ackermann_control
COPY carla_ego_vehicle /opt/carla-ros-bridge/src/carla_ego_vehicle
COPY carla_infrastructure /opt/carla-ros-bridge/src/carla_infrastructure
COPY carla_manual_control /opt/carla-ros-bridge/src/carla_manual_control
COPY carla_msgs /opt/carla-ros-bridge/src/carla_msgs
COPY carla_ros_bridge /opt/carla-ros-bridge/src/carla_ros_bridge
COPY carla_waypoint_publisher /opt/carla-ros-bridge/src/carla_waypoint_publisher
COPY pcl_recorder /opt/carla-ros-bridge/src/pcl_recorder

RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /opt/carla-ros-bridge; catkin_make -DCMAKE_BUILD_TYPE=Release install'

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /

CMD ["roslaunch", "carla_ros_bridge", "carla_ros_bridge.launch"]

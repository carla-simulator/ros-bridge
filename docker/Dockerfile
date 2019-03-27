ARG CARLA_VERSION='0.9.4'
ARG CARLA_BUILD=''

FROM carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla

FROM ros:kinetic-ros-base

RUN apt-get update \
	&& apt-get install -y libpng16-16  ros-kinetic-ackermann-msgs  ros-kinetic-derived-object-msgs ros-kinetic-tf ros-kinetic-cv-bridge --no-install-recommends \
	&& rm -rf /var/lib/apt/lists/*

ARG CARLA_VERSION

COPY --from=carla --chown=root /home/carla/PythonAPI /opt/carla/PythonAPI
ENV PYTHONPATH=/opt/carla/PythonAPI/carla-$CARLA_VERSION-py2.7-linux-x86_64.egg:/opt/carla/PythonAPI

COPY carla_ackermann_control /opt/carla-ros-bridge/src/carla_ackermann_control
COPY carla_ros_bridge /opt/carla-ros-bridge/src/carla_ros_bridge
COPY carla_ros_bridge_lifecycle /opt/carla-ros-bridge/src/carla_ros_bridge_lifecycle

RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd  /opt/carla-ros-bridge; catkin_make -DCMAKE_BUILD_TYPE=Release install'

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /

CMD ["roslaunch", "carla_ros_bridge", "carla_ros_bridge.launch"]

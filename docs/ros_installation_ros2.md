# ROS bridge installation for ROS 2 Humble

This section is a guide on how to install the ROS bridge on Linux for use with ROS 2. You will find the prerequisites, installation steps, how to run a basic package to make sure everything is working well and commands to run tests.

- [__Before You Begin__](#before-you-begin)
- [__ROS Bridge Installation__](#ros-bridge-installation)
- [__Run the ROS Bridge__](#run-the-ros-bridge)

!!! Important
    ROS is still [experimental](http://wiki.ros.org/noetic/Installation) for Windows. It has only been tested for Linux systems.
---

## Before You Begin

You will need to fulfill the following software requirements before using the ROS bridge:

- CARLA
    - Install CARLA 0.9.15 (or later). Follow the [quick start installation](https://carla.readthedocs.io/en/latest/start_quickstart/) or make the build for [Linux](https://carla.readthedocs.io/en/latest/build_linux/). 
    - Carla Client API for Python 3.10 found [here](https://github.com/gezp/carla_ros/releases/)
        - Either copy both the `.egg` file and the `.whl` file to your `CARLA_ROOT/PythonAPI/carla/dist` directory *or* install the `.whl` file using `pip`.
    - Set `CARLA_ROOT` environmental variable to your installation directory in `{.zshrc|.bashrc}`.
        ```sh
            export CARLA_ROOT=$HOME/Workspace/CARLA_0.9.15
            export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla
        ```
- SCENARIO RUNNER
    - Install the matching Scenario Runner version (e.g. [v0.9.15](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.15)) using the installation instructions found [here](https://carla-scenariorunner.readthedocs.io/en/latest/getting_scenariorunner/)
    - Set `SCENARIO_RUNNER_ROOT` environmental variable to your installation directory in `{.zshrc|.bashrc}`
        ```sh
            export SCENARIO_RUNNER_ROOT=$HOME/Workspace/scenario_runner
        ```
- ROS 2 Humble
    - [__ROS 2 Humble__](https://docs.ros.org/en/humble/Installation.html) — For Ubuntu 22.04 (Jammy)
    - Additional ROS packages may be required depending on your needs. [rviz2](https://github.com/ros2/rviz) is highly recommended to visualize ROS data.
    - Append the following to your `{.zshrc|.bashrc}`
        ```sh
            source /opt/ros/humble/setup.{zsh|bash}
        ```

---

## ROS Bridge Installation

__1.__ Set up the project directory and clone the ROS bridge repository and submodules:

```sh
    mkdir -p ~Workspace/ros-bridge && cd ~/ros-bridge
    git clone --recurse-submodules https://github.com/ttgamage/ros-bridge
    mv ros-bridge src
```

__2.__ Set up the ROS environment and install dependencies:

```sh
    source /opt/ros/humble/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r
```

__3.__ Build the ROS bridge workspace using colcon:

```sh
    colcon build --symlink-install
```

---

## Run the ROS bridge

__1.__ Start a CARLA server according to the installation method used to install CARLA:

```sh
    # Package version in carla root folder
    sh $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen
```

__2.__ Add the source path for the ROS bridge workspace:

```sh
    cd ~/Workspace/ros-bridge
    source ./install/setup.{zsh|bash}
```

__4.__ In another terminal, start the ROS 2 bridge. You can run one of the two options below:

```sh
    # Option 1, start the basic ROS bridge package
    ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

    # Option 2, start the ROS bridge with an example ego vehicle
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```
---

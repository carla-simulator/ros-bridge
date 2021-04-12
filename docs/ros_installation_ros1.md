# ROS bridge installation for ROS 1

This section is a guide on how to install the ROS bridge on Linux for use with ROS 1. You will find the prerequisites, installation steps, how to run a basic package to make sure everything is working well and commands to run tests.

- [__Before you begin__](#before-you-begin)
    - [__ROS bridge installation __](#ros-bridge-installation)
        - [A. Using the Debian repository](#a-using-the-debian-repository)
        - [B. Using the source repository](#b-using-the-source-repository)
- [__Run the ROS bridge__](#run-the-ros-bridge)
- [__Testing__](#testing)

!!! Important
    The ROS bridge has not yet been tested in Windows.

---
## Before you begin

You will need to fulfill the following software requirements before using the ROS bridge:

- Install ROS according to your operating system:
    - [__ROS Melodic__](https://wiki.ros.org/melodic/Installation/Ubuntu) — For Ubuntu 18.04 (Bionic)
    - [__ROS Noetic__](https://wiki.ros.org/noetic#Installation) — For Ubuntu 20.04 (Focal)
- Additional ROS packages may be required depending on your needs. [rviz](https://wiki.ros.org/rviz) is highly recommended to visualize ROS data.
- CARLA 0.9.7 or later — Previous versions are not compatible with the ROS bridge. Follow the [quick start installation](https://carla.readthedocs.io/en/latest/start_quickstart/) or make the build for [Linux](https://carla.readthedocs.io/en/latest/build_linux/). It is recommended to match the ROS bridge version to the CARLA version when possible.

---
## ROS bridge installation

There are two options available to install the ROS bridge:

- Install via the __apt__ tool from the Debian repository. Only available on Ubuntu 18.04.
- Cloning from the source repository on GitHub.

 Both methods are detailed below.

!!! Important
    To install ROS bridge versions prior to 0.9.10, you will find the instructions in the older versions of the CARLA documentation [here](https://carla.readthedocs.io/en/0.9.10/ros_installation/). Change to the appropriate version of the documentation using the panel in the bottom right corner of the window. ![docs_version_panel](images/docs_version_panel.jpg)

### A. Using the Debian repository

!!! Note
    This installation method is ony available on Ubuntu 18.04. For other supported distributions, see [Section B: Using the source repository](#b-using-the-source-repository).

__1.__ Set up the Debian repository in your system:
```sh
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
    sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
```

__2.__ Install the ROS bridge:

> - Latest version:
```sh
        sudo apt-get update # Update the Debian package index
        sudo apt-get install carla-ros-bridge # Install the latest ROS bridge version, or update the current installation
```

> - Install a specific version by adding a version tag to the command:
```sh
        apt-cache madison carla-ros-bridge # List the available versions of the ROS bridge
        sudo apt-get install carla-ros-bridge=0.9.10-1 # In this case, "0.9.10" refers to the ROS bridge version, and "1" to the Debian revision
```

__3.__ Check the ROS bridge has been installed successfully in the `/opt/` folder.

### B. Using the source repository


__1.__ Create a catkin workspace:
```sh
    mkdir -p ~/carla-ros-bridge/catkin_ws/src
```

__2.__ Clone the ROS Bridge repository and submodules:
```sh
    cd ~/carla-ros-bridge
    git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git catkin_ws/src/ros-bridge
```

__5.__ Set up the ROS environment according to the ROS version you have installed:
```sh
    source /opt/ros/<melodic/noetic>/setup.bash
```
__6.__ Install the required ros-dependencies:
```sh
    cd catkin_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r
```

__7.__ Build the ROS bridge:
```sh
    catkin build   # alternatively catkin_make
```

---

## Run the ROS bridge

__1.__ Start a CARLA server according to the installation method used to install CARLA:
```sh
    # Package version in carla root folder
    ./CarlaUE4.sh

    # Debian installation in `opt/carla-simulator/`
    ./CarlaUE4.sh

    # Build from source version in carla root folder
    make launch
```

__2.__ Add the correct CARLA modules to your Python path:

```sh
        export CARLA_ROOT=<path-to-carla>
        export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla
```

__3.__ Add the source path for the ROS bridge workspace according to the installation method of the ROS bridge. This should be done in every terminal each time you want to run the ROS bridge:

```sh
    # For debian installation of ROS bridge. Change the command according to your installed version of ROS.
    source /opt/carla-ros-bridge/<melodic/noetic>/setup.bash

    # For GitHub repository installation of ROS bridge
    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
```

!!! Important
    The source path can be set permanently, but it may cause conflict when working with another workspace.

__4.__ Start the ROS bridge. Use any of the different launch files available to check the installation:

```sh
    # Option 1: start the ros bridge
    roslaunch carla_ros_bridge carla_ros_bridge.launch

    # Option 2: start the ros bridge together with an example ego vehicle
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
```


!!! Note

    If you receive the error: `ImportError: no module named CARLA` then the path to the CARLA Python API is missing. The apt installation sets the path automatically, but it may be missing for other installations.

    You will need to add the appropriate `.egg` file to your Python path. You will find the file in either `/PythonAPI/` or `/PythonAPI/dist/` depending on the CARLA installation. Execute the following command with the complete path to the `.egg` file, using the file that corresponds to your installed version of Python:

    `export PYTHONPATH=$PYTHONPATH:path/to/carla/PythonAPI/<your_egg_file>`

    It is recommended to set this variable permanently by adding the previous line to your `.bashrc` file.

    To check the CARLA library can be imported correctly, run the following command and wait for a success message:

            python3 -c 'import carla;print("Success")' # python3

            or

            python -c 'import carla;print("Success")' # python2

---

## Testing

To execute tests using catkin:

__1.__ Build the package:

```sh
    catkin_make -DCATKIN_ENABLE_TESTING=0
```

__2.__ Run the tests:

```sh
    rostest carla_ros_bridge ros_bridge_client.test
```

---

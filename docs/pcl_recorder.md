# Point Cloud Map Creation

The [PCL recorder package](https://github.com/carla-simulator/ros-bridge/tree/master/pcl_recorder) allows you to create point cloud maps from CARLA maps.

---

## Before you begin

Install the `pcl-tools` library:

```sh
sudo apt install pcl-tools
```

---

## Using the PCL recorder

The PCL recorder package will spawn an ego vehicle that can be controlled with the keyboard or via the autopilot functionality within the Carla PythonAPI.

__1.__ After starting a CARLA server, in a new terminal run the following command to launch the PCL recorder package:

```sh
# ROS 1
roslaunch pcl_recorder pcl_recorder.launch

# ROS 2
ros2 launch pcl_recorder pcl_recorder.launch.py
```
__2.__ When the capture drive is finished, reduce the overall size of the point cloud:

```
# Create one point cloud file
pcl_concatenate_points_pcd /tmp/pcl_capture/*.pcd

# Filter duplicates
pcl_voxel_grid -leaf 0.1,0.1,0.1 output.pcd map.pcd
```

__3.__ Verify the result:

```sh
pcl_viewer map.pcd
```

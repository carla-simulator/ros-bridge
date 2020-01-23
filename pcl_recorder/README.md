# Point Cloud Map Creation

Create pointcloud maps for Carla Levels.

The point clouds are created by driving around with an ego vehicle, using the autopilot functionality within the Carla PythonAPI.

## Setup

See setup of carla-ros-bridge.

## Run

Execute the Carla Simulator and the Pcl-Recorder.

    #Terminal 1

    #execute Carla
    SDL_VIDEODRIVER=offscreen <path-to-carla>/CarlaUE4.sh /Game/Carla/Maps/Town01 -benchmark -fps=20

    #Terminal 2

    #Execute the PCL Capturing
    #The captured point clouds are saved to /tmp/pcl_capture directory.
    export PYTHONPATH=<path-to-carla>/PythonAPI/carla/dist/carla-<version_and_arch>.egg:<path-to-carla>/PythonAPI/carla/
    source <path-to-catkin-workspace>/devel/setup.bash
    roslaunch pcl_recorder pcl_recorder.launch

When the capture drive is done, you can reduce the overall size of the point cloud.

    #create one point cloud file
    pcl_concatenate_points_pcd /tmp/pcl_capture/*.pcd

    #filter duplicates
    pcl_voxel_grid -leaf 0.1,0.1,0.1 output.pcd map.pcd

    #verify the result
    pcl_viewer map.pcd

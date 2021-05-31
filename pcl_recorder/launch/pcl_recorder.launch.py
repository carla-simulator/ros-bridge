import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def launch_enable_autopilot_publisher(context, *args, **kwargs):
    topic_name = "/carla/" + launch.substitutions.LaunchConfiguration('role_name').perform(context) + "/enable_autopilot"
    return [
        launch.actions.ExecuteProcess(
            output="screen",
            cmd=["ros2", "topic", "pub", topic_name,
                 "std_msgs/msg/Bool", "{'data': True}", "--qos-durability", "transient_local"],
            name='topic_pub_enable_autopilot')]


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.OpaqueFunction(function=launch_enable_autopilot_publisher),
        launch_ros.actions.Node(
            package='pcl_recorder',
            executable='pcl_recorder_node',
            name='pcl_recorder_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge_with_example_ego_vehicle.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

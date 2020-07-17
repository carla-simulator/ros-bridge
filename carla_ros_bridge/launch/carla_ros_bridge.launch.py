import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_role_names',
            default_value=["hero", "ego_vehicle", "hero0", "hero1", "hero2",
                           "hero3", "hero4", "hero5", "hero6", "hero7", "hero8", "hero9"]
        ),
        launch_ros.actions.Node(
            package='carla_ros_bridge',
            node_executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'carla/host': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'carla/port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'carla/timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'carla/synchronous_mode': launch.substitutions.LaunchConfiguration('synchronous_mode')
                },
                {
                    'carla/synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command')
                },
                {
                    'carla/fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
                },
                {
                    'carla/town': launch.substitutions.LaunchConfiguration('town')
                },
                {
                    'carla/ego_vehicle/role_name': launch.substitutions.LaunchConfiguration('ego_vehicle_role_names')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

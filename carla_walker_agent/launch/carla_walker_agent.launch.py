import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='target_speed',
            default_value='2.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mode',
            default_value='vehicle'
        ),
        launch_ros.actions.Node(
            package='carla_walker_agent',
            executable='carla_walker_agent',
            name=launch.substitutions.LaunchConfiguration('role_name'),
            output='screen',
            emulate_tty='True',
            parameters=[
                {
                    'target_speed': launch.substitutions.LaunchConfiguration('target_speed')
                },
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
                {
                    'mode': launch.substitutions.LaunchConfiguration('mode')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

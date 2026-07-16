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
            name='control_priority',
            default_value='20'
        ),
        launch_ros.actions.Node(
            package='carla_manual_control',
            executable='carla_manual_control',
            name=['carla_manual_control_', launch.substitutions.LaunchConfiguration('role_name')],
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
                {
                    'control_priority': launch.substitutions.LaunchConfiguration('control_priority')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

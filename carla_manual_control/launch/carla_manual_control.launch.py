import os
import sys

import launch
import launch_ros.actions


def launch_setup(context, *args, **kwargs):
    # workaround to use launch argument 'role_name' as a part of the string used for the node's name
    node_name = 'carla_manual_control_' + \
        launch.substitutions.LaunchConfiguration('role_name').perform(context)

    carla_manual_control = launch_ros.actions.Node(
        package='carla_manual_control',
        executable='carla_manual_control',
        name=node_name,
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }
        ]
    )

    return [carla_manual_control]


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.OpaqueFunction(function=launch_setup)
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

import os
import sys

import launch
import launch_ros.actions


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
            name='timeout',
            default_value='2'
        ),
        launch.actions.DeclareLaunchArgument(
            name='infrastructure_sensor_definition_file'
        ),
        launch_ros.actions.Node(
            package='carla_infrastructure',
            node_executable='carla_infrastructure',
            name='carla_infrastructure',
            output='screen',
            emulate_tty='True',
            parameters=[
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
                    'infrastructure_sensor_definition_file': launch.substitutions.LaunchConfiguration('infrastructure_sensor_definition_file')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

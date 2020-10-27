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
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='resolution_x',
            default_value='800'
        ),
        launch.actions.DeclareLaunchArgument(
            name='resolution_y',
            default_value='600'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fov',
            default_value='50'
        ),
        launch_ros.actions.Node(
            package='carla_spectator_camera',
            node_executable='carla_spectator_camera',
            name='carla_spectator_camera',
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
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
                {
                    'resolution_x': launch.substitutions.LaunchConfiguration('resolution_x')
                },
                {
                    'resolution_y': launch.substitutions.LaunchConfiguration('resolution_y')
                },
                {
                    'fov': launch.substitutions.LaunchConfiguration('fov')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

import os
import sys

import launch
import launch_ros.actions

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package='carla_map_visualizer',
            executable='carla_map_visualizer',
            name='carla_map_visualizer',
            output='screen',
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

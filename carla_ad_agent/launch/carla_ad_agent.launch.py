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
        launch.actions.SetLaunchConfiguration(
            name='test',
            value='jb'
        ),
        launch.actions.DeclareLaunchArgument(
            name='target_speed',
            default_value='20'
        ),
        launch.actions.DeclareLaunchArgument(
            name='avoid_risk',
            default_value='True'
        ),
        launch_ros.actions.Node(
            package='carla_ad_agent',
            executable='carla_ad_agent',
            name=['carla_ad_agent_', launch.substitutions.LaunchConfiguration('role_name')],
            output='screen',
            parameters=[
                {
                    'target_speed': launch.substitutions.LaunchConfiguration('target_speed')
                },
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
                {
                    'avoid_risk': launch.substitutions.LaunchConfiguration('avoid_risk')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

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
            name='vehicle_filter',
            default_value='vehicle.*'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sensor_definition_file',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='remap_rviz_initialpose_goal',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_ego_vehicle',
            default_value='True'
        ),

        # TODO: topic_tools not yet available in ROS2
        # launch_ros.actions.Node(
        #     package='topic_tools',
        #     node_executable='relay',
        #     name=launch.substitutions.LaunchConfiguration('role_name'),
        #     parameters=[
        #         {
        #             '/carla/host': launch.substitutions.LaunchConfiguration('host')
        #         },
        #         {
        #             '/carla/port': launch.substitutions.LaunchConfiguration('port')
        #         },
        #         {
        #             '/carla/timeout': launch.substitutions.LaunchConfiguration('timeout')
        #         }
        #     ],
        #     condition=launch.conditions.IfCondition(
        #         launch.substitutions.LaunchConfiguration('remap_rviz_initialpose_goal'))
        # ),
        # launch_ros.actions.Node(
        #     package='topic_tools',
        #     node_executable='relay',
        #     name=launch.substitutions.LaunchConfiguration('role_name'),
        #     parameters=[
        #         {
        #             '/carla/host': launch.substitutions.LaunchConfiguration('host')
        #         },
        #         {
        #             '/carla/port': launch.substitutions.LaunchConfiguration('port')
        #         },
        #         {
        #             '/carla/timeout': launch.substitutions.LaunchConfiguration('timeout')
        #         }
        #     ],
        #     condition=launch.conditions.IfCondition(
        #         launch.substitutions.LaunchConfiguration('remap_rviz_initialpose_goal'))
        # ),
        launch_ros.actions.Node(
            package='carla_ego_vehicle',
            node_executable='carla_ego_vehicle',
            name=launch.substitutions.LaunchConfiguration('role_name'),
            output='screen',
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
                    'sensor_definition_file': launch.substitutions.LaunchConfiguration('sensor_definition_file')
                },
                {
                    'vehicle_filter': launch.substitutions.LaunchConfiguration('vehicle_filter')
                },
                {
                    'spawn_point': launch.substitutions.LaunchConfiguration('spawn_point')
                },
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
                {
                    'spawn_ego_vehicle': launch.substitutions.LaunchConfiguration('spawn_ego_vehicle')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

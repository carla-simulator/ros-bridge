import os
import sys

import launch
import launch_ros.actions

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='objects_topic',
            default_value='/carla/objects'
        ),
        launch.actions.DeclareLaunchArgument(
            name='lifetime_seconds',
            default_value='1.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_vehicle_info_topic',
            default_value='',
            description = 'Set a topic for the ego_vehicle to enable filtering of the ego-object'
        ),
        launch.actions.DeclareLaunchArgument(
            name='actor_list_topic',
            default_value='',
            description = 'Set a topic for the actor list to enable filtering of environment objects' \
                          ' based on the detailed CityObjectLabel information.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='environment_objects_labels_filter_positive',
            default_value='Pedestrians,Rider,Car,Truck,Bus,Train,Motorcycle,Bicycle',
            description = 'Comma-separated list of CityObjectLabel values to be allowed from environment objects based on their label received via actor_list_topic. ' \
                          'If empty, all environment objects will be allowed based on their labels. But Rviz won t be able to render thousands of boxes' \
                          'Labels are e.g. (see CarlaActorInfo.msg): ' \
                          'Roads, Sidewalks, Buildings, Walls, Fences, Poles, Vegetation, Terrain, Water, RoadLines, Ground, Bridge, RailTrack, GuardRail, TrafficSigns, TrafficLights'
        ),
        launch.actions.DeclareLaunchArgument(
            name='environment_objects_labels_filter_negative',
            default_value='',
            description = 'Comma-separated list of CityObjectLabel values to be dropped from environment objects based on their label received via actor_list_topic. ' \
                          'This list is only considered if environment_objects_labels_filter_positive is empty, otherwise environment_objects_labels_filter_positive has higher priority. ' \
                          'If empty, no environment objects will be dropped based on their labels. But Rviz won t be able to render thousands of boxes' \
                          'Labels are e.g. (see CarlaActorInfo.msg): ' \
                          'Roads, Sidewalks, Buildings, Walls, Fences, Poles, Vegetation, Terrain, Water, RoadLines, Ground, Bridge, RailTrack, GuardRail, TrafficSigns, TrafficLights'
        ),
        launch_ros.actions.Node(
            package='derived_objects_visualizer',
            executable='derived_objects_visualizer',
            name='derived_objects_visualizer',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'objects_topic': launch.substitutions.LaunchConfiguration('objects_topic')
                },
                {
                    'lifetime_seconds': launch.substitutions.LaunchConfiguration('lifetime_seconds')
                },
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                },
                {
                    'ego_vehicle_vehicle_info_topic': launch.substitutions.LaunchConfiguration('ego_vehicle_vehicle_info_topic')
                },
                {
                    'actor_list_topic': launch.substitutions.LaunchConfiguration('actor_list_topic')
                },
                {
                    'environment_objects_labels_filter_positive': launch.substitutions.LaunchConfiguration('environment_objects_labels_filter_positive')
                },
                {
                    'environment_objects_labels_filter_negative': launch.substitutions.LaunchConfiguration('environment_objects_labels_filter_negative')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

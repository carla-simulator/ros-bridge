import os

import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=get_package_share_directory(
                'carla_spawn_objects') + '/config/objects.json'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='hero'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point_hero',
            default_value='5.09,-240.45,0,0,0,0.67,0.74'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_sensors_only',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_id',
            default_value='control'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'carla_spawn_objects.launch.py')
            ),
            launch_arguments={
                'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file'),
                'spawn_point_hero': launch.substitutions.LaunchConfiguration('spawn_point_hero'),
                'spawn_sensors_only': launch.substitutions.LaunchConfiguration('spawn_sensors_only')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'set_initial_pose.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'control_id': launch.substitutions.LaunchConfiguration('control_id')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='avoid_risk',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kp_lateral',
            default_value='0.9'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Ki_lateral',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kd_lateral',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kp_longitudinal',
            default_value='0.206'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Ki_longitudinal',
            default_value='0.0206'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kd_longitudinal',
            default_value='0.515'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_time_step',
            default_value='0.05'
        ),
        launch_ros.actions.Node(
            package='carla_ad_agent',
            executable='ad_agent',
            name=['carla_ad_agent_', launch.substitutions.LaunchConfiguration('role_name')],
            output='screen',
            parameters=[
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
                {
                    'avoid_risk': launch.substitutions.LaunchConfiguration('avoid_risk')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='carla_ad_agent',
            executable='local_planner',
            name=['local_planner_', launch.substitutions.LaunchConfiguration('role_name')],
            output='screen',
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
                {
                    'Kp_lateral': launch.substitutions.LaunchConfiguration('Kp_lateral')
                },
                {
                    'Ki_lateral': launch.substitutions.LaunchConfiguration('Ki_lateral')
                },
                {
                    'Kd_lateral': launch.substitutions.LaunchConfiguration('Kd_lateral')
                },
                {
                    'Kp_longitudinal': launch.substitutions.LaunchConfiguration('Kp_longitudinal')
                },
                {
                    'Ki_longitudinal': launch.substitutions.LaunchConfiguration('Ki_longitudinal')
                },
                {
                    'Kd_longitudinal': launch.substitutions.LaunchConfiguration('Kd_longitudinal')
                },
                {
                    'control_time_step': launch.substitutions.LaunchConfiguration('control_time_step')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

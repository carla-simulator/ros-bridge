import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

# Get the value of the SCENARIO_RUNNER_ROOT environment variable
scenario_runner_root = os.getenv('SCENARIO_RUNNER_ROOT')

# Default Goal Pose for the Ego Vehicle (Town04). Stored as Dictionary
default_goal_pose = {
    'px': float('6.0'),
    'py': float('-80.0'),
    'pz': float('0'),
    'ox': float('0'),
    'oy': float('0'),
    'oz': float('0.71'),
    'ow': float('0.7'),
}

# string with message to publish on topic /carla/available/scenarios
# This topic expects dictionary-like messages
abf_scenario_file = os.path.join(get_package_share_directory('carla_abf_demo'), 'config/ABFScenario.xosc')

ros_topic_msg_string = "{{ 'scenarios': \
    [\
         {{ 'name': 'ABF Scenario', 'scenario_file': '{}'}}\
    ] }}".format(abf_scenario_file)


def generate_launch_description():
    ld = launch.LaunchDescription([
        # Socket Address of the CARLA Server
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),

        # Default Town Town04
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town04'
        ),

        # Timeout to Load Town Map
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='15'
        ),

        # Sync Mode Wait for Vehicle Command
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),

        # Delta Seconds
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05'
        ),

        # Scenario Runner Location
        launch.actions.DeclareLaunchArgument(
            name='scenario_runner_path',
            default_value=scenario_runner_root
        ),

        # Set Ego Vehicle's Role Name
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='hero'
        ),

        # Set Ego Vehicle's Goal Pose
        launch.actions.DeclareLaunchArgument(
            name='goal_pose',
            default_value=default_goal_pose
        ),

        # Carla Twist to Control
        launch_ros.actions.Node(
            package='carla_twist_to_control',
            executable='carla_twist_to_control',
            name='carla_twist_to_control',
            remappings=[
                (
                    ["/carla/",
                        launch.substitutions.LaunchConfiguration('role_name'), "/vehicle_control_cmd"],
                    ["/carla/",
                        launch.substitutions.LaunchConfiguration('role_name'), "/vehicle_control_cmd_manual"]
                )
            ],
            parameters=[
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                }
            ]
        ),

        # Publish ABFCaseStudy to topic /carla/available_scenarios
        launch.actions.ExecuteProcess(
            cmd=["ros2", "topic", "pub", "/carla/available_scenarios",
                 "carla_ros_scenario_runner_types/CarlaScenarioList", ros_topic_msg_string],
            name='topic_pub_available_scenarios',
        ),

        # Launch CARLA Bridge
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
            }.items()
        ),

        # Spawn Ego Vehicle (done outside of OpenScenario)
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
            ),
            launch_arguments={
                'object_definition_file': get_package_share_directory('carla_spawn_objects') + '/config/objects.json',
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),

        # Launch Waypoint Publisher
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_waypoint_publisher'), 'carla_waypoint_publisher.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),

        # Launch Scenario Runner
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_scenario_runner'), 'carla_ros_scenario_runner.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'scenario_runner_path': launch.substitutions.LaunchConfiguration('scenario_runner_path'),
                'wait_for_ego': 'True'
            }.items()
        ),

        # Launch ns-3 Bridge
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ns3_ros_bridge'), 'ns3_ros_bridge.launch.py')
            ),
            launch_arguments={
                'delay_ms': '0'
            }.items()
        ),


        # Service Call to Load the Scenario
        launch.actions.ExecuteProcess(
            cmd=["ros2", "service", "call", "/scenario_runner/execute_scenario",
                 "carla_ros_scenario_runner_types/srv/ExecuteScenario",
                 "{{scenario: {{scenario_file: '{}'}}}}".format(abf_scenario_file)],
            name='service_call_execute_scenario',
            output='screen',  # This line prints the output to the terminal
        ),

        # Launch RVIZ 2 Interface
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            remappings=[
                (
                    "/carla/hero/spectator_pose",
                    "/carla/hero/rgb_view/control/set_transform"
                )
            ],
            arguments=[
                '-d', os.path.join(get_package_share_directory('carla_abf_demo'), 'config/abf_demo_config.rviz')],
            on_exit=launch.actions.Shutdown()
        ),

        # Set Ego Vehicle's Goal Pose
        launch.actions.ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub", "--once", "/carla/hero/goal_pose", "geometry_msgs/msg/PoseStamped", 
                '{{ "pose": {{ \
                    "position": {{"x": {px}, "y": {py}, "z": {pz}}}, \
                    "orientation": {{"x": {ox}, "y": {oy}, "z": {oz}, "w": {ow}\
                }}}}}}'.format(
                    px=default_goal_pose['px'],
                    py=default_goal_pose['py'],
                    pz=default_goal_pose['pz'],
                    ox=default_goal_pose['ox'],
                    oy=default_goal_pose['oy'],
                    oz=default_goal_pose['oz'],
                    ow=default_goal_pose['ow'])
            ],
            name='execute_topic_pub_goal_pose',
        ),

        # launch carla_vehicle_data
        launch_ros.actions.Node(
            package='carla_vehicle_data',
            executable='carla_vehicle_data',
            name='carla_vehicle_data',
            #output='screen'
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

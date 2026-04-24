import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    package_dir = get_package_share_directory('multi_robot')
    urdf_file = os.path.join(package_dir, 'urdf', 'puzzlebot.urdf')
    params_file = os.path.join(package_dir, 'config', 'robot_params.yaml')

    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_description = infp.read()

    namespace = LaunchConfiguration('namespace')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_theta = LaunchConfiguration('initial_theta')

    trajectory_type = LaunchConfiguration('trajectory_type')
    side_length = LaunchConfiguration('side_length')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_theta', default_value='0.0'),
        DeclareLaunchArgument('trajectory_type', default_value='square'),
        DeclareLaunchArgument('side_length', default_value='0.8'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),

        GroupAction([
            PushRosNamespace(namespace),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_odom_broadcaster',
                arguments=[
                    '--x', initial_x,
                    '--y', initial_y,
                    '--z', '0.0',
                    '--yaw', initial_theta,
                    '--pitch', '0.0',
                    '--roll', '0.0',
                    '--frame-id', 'world',
                    '--child-frame-id', [namespace, '/odom'],
                ],
                output='screen',
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    params_file,
                    {'robot_description': robot_description},
                    {'frame_prefix': [namespace, '/']},
                ],
            ),

            Node(
                package='multi_robot',
                executable='kinematic_simulator',
                name='puzzlebot_sim',
                output='screen',
                parameters=[
                    params_file,
                    {'publish_tf': False},
                    {'publish_joint_states': False},
                    {'x0': 0.0},
                    {'y0': 0.0},
                    {'theta0': 0.0},
                    {'odom_frame': [namespace, '/odom']},
                    {'base_frame': [namespace, '/base_footprint']},
                    {'pose_frame': [namespace, '/odom']},
                ],
            ),

            Node(
                package='multi_robot',
                executable='dead_reckoning_localization',
                name='dead_reckoning_localization',
                output='screen',
                parameters=[
                    params_file,
                    {'x0': 0.0},
                    {'y0': 0.0},
                    {'theta0': 0.0},
                    {'odom_frame': [namespace, '/odom']},
                    {'base_frame': [namespace, '/base_footprint']},
                ],
            ),

            Node(
                package='multi_robot',
                executable='point_stabilizer',
                name='point_stabilizer',
                output='screen',
                parameters=[params_file],
            ),

            Node(
                package='multi_robot',
                executable='setpoint_generator',
                name='setpoint_generator',
                output='screen',
                parameters=[
                    params_file,
                    {'trajectory_type': trajectory_type},
                    {'side_length': side_length},
                    {'start_x': start_x},
                    {'start_y': start_y},
                ],
            ),

            Node(
                package='multi_robot',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[params_file],
            ),
        ])
    ])
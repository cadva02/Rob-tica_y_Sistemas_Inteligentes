import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('kinematic_model')
    urdf_file = os.path.join(package_dir, 'urdf', 'puzzlebot.urdf')
    params_file = os.path.join(package_dir, 'config', 'robot_params.yaml')
    rviz_config = os.path.join(package_dir, 'rviz', 'puzzlebot_rviz.rviz')

    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_description = infp.read()

    static_transform_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'world', '--child-frame-id', 'map',
        ],
    )

    static_transform_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.5', '--y', '0.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
    )

    static_transform_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'odom', '--child-frame-id', 'base_footprint',
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            params_file,
            {'robot_description': robot_description},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    rqt_tf_tree = Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        name='rqt_tf_tree',
        output='screen',
    )

    return LaunchDescription([
        static_transform_world_map,
        static_transform_map_odom,
        static_transform_odom_base,
        robot_state_publisher,
        rviz,
        rqt_tf_tree,
    ])

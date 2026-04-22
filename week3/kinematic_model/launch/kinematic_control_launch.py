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

    puzzlebot_sim = Node(
        package='kinematic_model',
        executable='kinematic_simulator',
        name='puzzlebot_sim',
        output='screen',
        parameters=[
            params_file,
            {'publish_tf': False},
        ],
    )

    dead_reckoning_localization = Node(
        package='kinematic_model',
        executable='dead_reckoning_localization',
        name='dead_reckoning_localization',
        output='screen',
        parameters=[params_file],
    )

    point_stabilizer = Node(
        package='kinematic_model',
        executable='point_stabilizer',
        name='point_stabilizer',
        output='screen',
        parameters=[params_file],
    )

    setpoint_generator = Node(
        package='kinematic_model',
        executable='setpoint_generator',
        name='setpoint_generator',
        output='screen',
        parameters=[params_file],
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

    rqt_graph = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen',
    )

    rqt_plot_pose = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot_pose',
        output='screen',
        arguments=[
            '/next_point/x',
            '/next_point/y',
            '/pose_sim/pose/position/x',
            '/pose_sim/pose/position/y',
            '/odom/pose/pose/position/x',
            '/odom/pose/pose/position/y',
        ],
    )

    rqt_plot_vel = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot_vel',
        output='screen',
        arguments=[
            '/cmd_vel/linear/x',
            '/cmd_vel/angular/z',
            '/odom/twist/twist/linear/x',
            '/odom/twist/twist/angular/z',
        ],
    )

    return LaunchDescription([
        static_transform_world_map,
        static_transform_map_odom,
        robot_state_publisher,
        puzzlebot_sim,
        dead_reckoning_localization,
        point_stabilizer,
        setpoint_generator,
        rviz,
        rqt_tf_tree,
        rqt_graph,
        rqt_plot_pose,
        rqt_plot_vel,
    ])

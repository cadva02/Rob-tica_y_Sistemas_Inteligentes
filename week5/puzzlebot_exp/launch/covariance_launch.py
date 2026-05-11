import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    package_dir = get_package_share_directory('puzzlebot_exp')
    urdf_file = os.path.join(package_dir, 'urdf', 'puzzlebot.urdf')
    params_file = os.path.join(package_dir, 'config', 'robot_params.yaml')
    rviz_config = os.path.join(package_dir, 'rviz', 'puzzlebot_rviz.rviz')

    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_description = infp.read()

    with open(params_file, 'r', encoding='utf-8') as pf:
        params_data = yaml.safe_load(pf) or {}

    # Ensure setpoint params are applied even under namespace (e.g. /robot1/setpoint_generator)
    setpoint_params = params_data.get('setpoint_generator', {}).get('ros__parameters', {})

    namespace = LaunchConfiguration('namespace')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_theta = LaunchConfiguration('initial_theta')
    real_namespace = LaunchConfiguration('real_namespace')
    initial_x_real = LaunchConfiguration('initial_x_real')
    initial_y_real = LaunchConfiguration('initial_y_real')
    initial_theta_real = LaunchConfiguration('initial_theta_real')

    robot1_group = GroupAction([
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
            package='puzzlebot_exp',
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
            package='puzzlebot_exp',
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
            package='puzzlebot_exp',
            executable='point_stabilizer',
            name='point_stabilizer',
            output='screen',
            parameters=[params_file],
        ),

        # Important: setpoint params come only from robot_params.yaml.
        # No launch-time overrides here.
        Node(
            package='puzzlebot_exp',
            executable='setpoint_generator',
            name='setpoint_generator',
            output='screen',
            parameters=[params_file, setpoint_params],
        ),

        Node(
            package='puzzlebot_exp',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[params_file],
        ),
    ])

    real_robot_group = GroupAction([
        PushRosNamespace(real_namespace),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_broadcaster_real',
            arguments=[
                '--x', initial_x_real,
                '--y', initial_y_real,
                '--z', '0.0',
                '--yaw', initial_theta_real,
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'world',
                '--child-frame-id', [real_namespace, '/odom'],
            ],
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_real',
            output='screen',
            parameters=[
                params_file,
                {'robot_description': robot_description},
                {'frame_prefix': [real_namespace, '/']},
            ],
        ),

        Node(
            package='puzzlebot_exp',
            executable='kinematic_simulator',
            name='puzzlebot_sim_real',
            output='screen',
            parameters=[
                params_file,
                {'publish_tf': False},
                {'publish_joint_states': False},
                {'x0': 0.0},
                {'y0': 0.0},
                {'theta0': 0.0},
                {'odom_frame': [real_namespace, '/odom']},
                {'base_frame': [real_namespace, '/base_footprint']},
                {'pose_frame': [real_namespace, '/odom']},
            ],
        ),

        Node(
            package='puzzlebot_exp',
            executable='dead_reckoning_localization',
            name='dead_reckoning_localization_real',
            output='screen',
            parameters=[
                params_file,
                {'x0': 0.0},
                {'y0': 0.0},
                {'theta0': 0.0},
                {'sigma_v': 0.01},
                {'sigma_w': 0.1},
                {'odom_frame': [real_namespace, '/odom']},
                {'base_frame': [real_namespace, '/base_footprint']},
            ],
        ),

        Node(
            package='puzzlebot_exp',
            executable='point_stabilizer',
            name='point_stabilizer_real',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='puzzlebot_exp',
            executable='setpoint_generator',
            name='setpoint_generator_real',
            output='screen',
            parameters=[params_file, setpoint_params],
        ),

        Node(
            package='puzzlebot_exp',
            executable='joint_state_publisher',
            name='joint_state_publisher_real',
            output='screen',
            parameters=[params_file],
        ),
    ])

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
            '/robot1/next_point/x',
            '/robot1/next_point/y',
            '/robot1/pose_sim/pose/position/x',
            '/robot1/pose_sim/pose/position/y',
            '/robot1/odom/pose/pose/position/x',
            '/robot1/odom/pose/pose/position/y',
        ],
    )

    rqt_plot_vel = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot_vel',
        output='screen',
        arguments=[
            '/robot1/cmd_vel/linear/x',
            '/robot1/cmd_vel/angular/z',
            '/robot1/odom/twist/twist/linear/x',
            '/robot1/odom/twist/twist/angular/z',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('real_namespace', default_value='robot1_real'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_theta', default_value='0.0'),
        DeclareLaunchArgument('initial_x_real', default_value='0.0'),
        DeclareLaunchArgument('initial_y_real', default_value='0.0'),
        DeclareLaunchArgument('initial_theta_real', default_value='0.0'),
        robot1_group,
        real_robot_group,
        rviz,
        rqt_tf_tree,
        rqt_graph,
        rqt_plot_pose,
        rqt_plot_vel,
    ])

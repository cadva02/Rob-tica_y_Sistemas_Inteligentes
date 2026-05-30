import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('puzzlebot_description_final')
    pkg_gazebo = get_package_share_directory('puzzlebot_gazebo_final')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='puzzlebot_office.world',
        description='Gazebo world with route goals and guide markers'
    )
    declare_robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='puzzlebot_jetson_lidar_ed',
        description='Puzzlebot robot model'
    )
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='Puzzlebot1',
        description='Instance name used by Gazebo'
    )
    declare_x_arg = DeclareLaunchArgument('x', default_value='-2.5')
    declare_y_arg = DeclareLaunchArgument('y', default_value='-3.0')
    declare_yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_marker_map_arg = DeclareLaunchArgument(
        'marker_map_json',
        default_value=json.dumps([
            {'id': 0, 'x': -0.95, 'y': -3.8, 'theta': 1.5708},
            {'id': 1, 'x': 3.5, 'y': 0.0, 'theta': 3.141592654},
            {'id': 2, 'x': -1.2, 'y': 1.2, 'theta': -0.7854},
            {'id': 3, 'x': -1.0, 'y': 3.8, 'theta': 0.7854},
        ]),
        description='ArUco marker map as JSON list with id, x, y, theta'
    )

    world = LaunchConfiguration('world')
    robot = LaunchConfiguration('robot')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    marker_map_json = LaunchConfiguration('marker_map_json')

    world_launch = os.path.join(pkg_gazebo, 'launch', 'gazebo_world_launch.py')
    robot_launch = os.path.join(pkg_gazebo, 'launch', 'gazebo_puzzlebot_launch.py')

    include_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={
            'world': world,
            'pause': 'false',
            'verbosity': '4',
        }.items(),
    )

    include_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            'robot_name': robot_name,
            'robot': robot,
            'x': x,
            'y': y,
            'yaw': yaw,
            'use_sim_time': 'true',
            'prefix': '',
            'camera_frame': '',
            'tof_frame': '',
            'lidar_frame': '',
        }.items(),
    )

    localization_node = Node(
        package='puzzlebot_description_final',
        executable='localization',
        name='localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wheel_radius': 0.05,
            'wheel_base': 0.19,
            'sample_time': 0.02,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'wr_topic': '/VelocityEncR',
            'wl_topic': '/VelocityEncL',
            'aruco_topic': '/aruco_markers',
            'marker_map_json': ParameterValue(marker_map_json, value_type=str),
            'camera_base_x': 0.1241,
            'camera_base_y': 0.0,
            'camera_base_theta': 0.0,
            'x0': -2.5,
            'y0': -3.0,
            'theta0': 0.0,
            'sigma_v': 0.003,
            'sigma_w': 0.03,
            'sigma_obs_x': 0.20,
            'sigma_obs_y': 0.20,
            'sigma_obs_theta': 0.30,
            'aruco_distance_gain': 1.5,
            'aruco_theta_distance_gain': 1.5,
            'aruco_update_min_dist': 2.5,
            'mahal_threshold': 24.0,
        }],
    )

    detector_node = Node(
        package='puzzlebot_description_final',
        executable='gazebo_aruco_detector',
        name='gazebo_aruco_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera',
            'camera_info_topic': '/camera_info',
            'aruco_topic': '/aruco_markers',
            'marker_size_m': 0.14,
            'dictionary': 'DICT_4X4_50',
        }],
    )

    setpoint_generator_node = Node(
        package='puzzlebot_description_final',
        executable='setpoint_generator',
        name='setpoint_generator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'trajectory_type': 'custom',
            'side_length': 2.0,
            'start_x': -2.5,
            'start_y': -3.0,
            'publish_rate': 2.0,
            'min_waypoint_time_sec': 2.0,
            'custom_waypoints_json': json.dumps([
                {'x': -0.95, 'y': -3.8, 'theta': 1.5708},
                {'x': 0.5, 'y': 0.0, 'theta': 3.141592654},
                {'x': -1.0, 'y': 1.0, 'theta': -1.5708},
                {'x': -1.0, 'y': 3.8, 'theta': 0.0},
            ]),
        }],
    )

    obstacle_avoidance_node = Node(
        package='puzzlebot_description_final',
        executable='obstacle_avoidance',
        name='obstacle_avoidance_bug0',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_rate': 20.0,
            'linear_speed': 0.15,
            'angular_speed': 0.45,
            'goal_x': 0.0,
            'goal_y': 0.0,
            'goal_theta': 0.0,
            'goal_tolerance': 0.15,
            'yaw_tolerance': 0.25,
            'obstacle_distance': 0.40,
            'wall_dist_target': 0.35,
            'front_angle': 40.0,
            'kp_wall': 1.0,
            'wall_ang_limit': 0.8,
            'obstacle_detection_count': 3,
            'wall_exit_suppression': 1.0,
        }],
    )

    # RViz node to visualize odometry, TF, waypoints and markers
    rviz_config = os.path.join(pkg_description, 'rviz', 'puzzlebot_desc.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        declare_world_arg,
        declare_robot_arg,
        declare_robot_name_arg,
        declare_x_arg,
        declare_y_arg,
        declare_yaw_arg,
        declare_marker_map_arg,
        include_world,
        include_robot,
        detector_node,
        localization_node,
        setpoint_generator_node,
        obstacle_avoidance_node,
        rviz_node,
    ])

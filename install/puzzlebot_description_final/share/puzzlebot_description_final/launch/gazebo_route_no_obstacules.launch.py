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
        default_value='puzzlebot_route_markers.world',
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
    declare_x_arg = DeclareLaunchArgument('x', default_value='0.0')
    declare_y_arg = DeclareLaunchArgument('y', default_value='2.0')
    declare_yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_marker_map_arg = DeclareLaunchArgument(
        'marker_map_json',
        default_value=json.dumps([
            {'id': 0, 'x': 2.5, 'y': -2.0, 'theta': 1.5708},
            {'id': 1, 'x': 1.0, 'y': 1.8, 'theta': 3.141592654},
            {'id': 2, 'x': -0.5, 'y': 2.5, 'theta': -3.141592654},
            {'id': 3, 'x': -0.5, 'y': -0.5, 'theta': 0.0},
            {'id': 4, 'x': 3.0, 'y': -0.3, 'theta': 0.0},
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
            'x0': 0.0,
            'y0': 2.0,
            'theta0': 0.0,
            'sigma_v': 0.003,
            'sigma_w': 0.03,
            'sigma_obs_x': 0.12,
            'sigma_obs_y': 0.12,
            'sigma_obs_theta': 0.20,
            'aruco_distance_gain': 1.5,
            'aruco_theta_distance_gain': 1.0,
            'aruco_update_min_dist': 2.5,
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
            'start_x': 0.0,
            'start_y': 2.0,
            'publish_rate': 2.0,
            'min_waypoint_time_sec': 2.0,
            'custom_waypoints_json': json.dumps([
                {'x': 0.0, 'y': 2.0, 'theta': -0.4636476090008061},
                {'x': 1.0, 'y': 1.5, 'theta': -0.7853981633974483},
                {'x': 2.5, 'y': 0.0, 'theta': -2.0344439357957027},
                {'x': 2.0, 'y': -1.0, 'theta': 3.141592653589793},
                {'x': -1.0, 'y': -1.0, 'theta': 1.2490457723982544},
                {'x': 0.0, 'y': 2.0, 'theta': 0.0},
            ]),
        }],
    )

    point_stabilizer_node = Node(
        package='puzzlebot_description_final',
        executable='point_stabilizer',
        name='point_stabilizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'control_rate': 20.0,
            'position_tolerance': 0.05,
            'angle_tolerance': 0.05,
            'goal_x': 0.0,
            'goal_y': 0.0,
            'goal_theta': 0.0,
            'goal_reached_confirm_cycles': 8,
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
        point_stabilizer_node,
        rviz_node,
    ])

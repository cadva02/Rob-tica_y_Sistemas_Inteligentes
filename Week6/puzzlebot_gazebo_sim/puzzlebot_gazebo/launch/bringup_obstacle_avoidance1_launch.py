import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    declare_world_arg = DeclareLaunchArgument(
        'world', default_value='empty.world',
        description='Gazebo world file to load'
    )

    declare_robot_name_arg = DeclareLaunchArgument('robot_name', default_value='Puzzlebot1')
    declare_robot_arg = DeclareLaunchArgument('robot', default_value='puzzlebot_jetson_lidar_ed')
    declare_x_arg = DeclareLaunchArgument('x', default_value='0.0')
    declare_y_arg = DeclareLaunchArgument('y', default_value='0.0')
    declare_yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    robot = LaunchConfiguration('robot')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    pkg = get_package_share_directory('puzzlebot_gazebo')
    world_launch = os.path.join(pkg, 'launch', 'gazebo_world_launch.py')
    puzzlebot_launch = os.path.join(pkg, 'launch', 'gazebo_puzzlebot_launch.py')

    include_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={
            'world': world,
            'pause': 'false',
            'verbosity': '3',
        }.items(),
    )

    include_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(puzzlebot_launch),
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
        package='puzzlebot_description',
        executable='localization',
        name='localization',
        output='screen',
        parameters=[
            {'wheel_radius': 0.05},
            {'wheel_base': 0.19},
            {'sample_time': 0.02},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_footprint'},
        ],
    )

    setpoint_generator_node = Node(
        package='puzzlebot_description',
        executable='setpoint_generator',
        name='setpoint_generator',
        output='screen',
        parameters=[
            {'publish_rate': 5.0},
            {'trajectory_type': 'square'},
            {'side_length': 0.5},
            {'start_x': 0.0},
            {'start_y': 0.0},
        ],
    )

    obstacle_avoidance_node = Node(
        package='puzzlebot_description',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[
            {'linear_speed': 0.18},
            {'angular_speed': 0.6},
            {'goal_tolerance': 0.1},
            {'yaw_tolerance': 0.25},
            {'obstacle_distance': 0.6},
            {'front_angle': 40.0},
            {'scan_topic': 'scan'},
            {'cmd_vel_topic': 'cmd_vel'},
            {'goal_topic': 'next_point'},
            {'odom_topic': 'odom'},
        ],
    )

    return LaunchDescription([
        declare_world_arg,
        declare_robot_name_arg,
        declare_robot_arg,
        declare_x_arg,
        declare_y_arg,
        declare_yaw_arg,
        include_world,
        include_robot,
        localization_node,
        setpoint_generator_node,
        obstacle_avoidance_node,
    ])

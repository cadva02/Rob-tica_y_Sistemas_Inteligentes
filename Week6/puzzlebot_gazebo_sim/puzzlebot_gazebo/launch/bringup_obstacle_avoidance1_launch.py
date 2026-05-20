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
        'world', default_value='obstacle_avoidance_1.world',
        description='Gazebo world file to load'
    )

    declare_robot_name_arg = DeclareLaunchArgument('robot_name', default_value='Puzzlebot1')
    declare_robot_arg = DeclareLaunchArgument('robot', default_value='puzzlebot_jetson_lidar_ed')
    declare_x_arg = DeclareLaunchArgument('x', default_value='0.0')
    declare_y_arg = DeclareLaunchArgument('y', default_value='0.0')
    declare_yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_robot_lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='laser_frame')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    robot = LaunchConfiguration('robot')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    lidar_frame = LaunchConfiguration('lidar_frame')

    pkg = get_package_share_directory('puzzlebot_gazebo')
    world_launch = os.path.join(pkg, 'launch', 'gazebo_world_launch.py')
    puzzlebot_launch = os.path.join(pkg, 'launch', 'gazebo_puzzlebot_launch.py')

    include_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={
            'world': world,
            'pause': 'false',
            'verbosity': '4',
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
            'lidar_frame': lidar_frame,
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

    kinematic_simulator_node = Node(
        package='puzzlebot_description',
        executable='kinematic_simulator',
        name='kinematic_simulator',
        output='screen',
        parameters=[
            {'update_rate': 50.0},
            {'wheel_radius': 0.05},
            {'wheel_base': 0.19},
            {'x0': 0.0},
            {'y0': 0.0},
            {'theta0': 0.0},
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
            {'side_length': 0.0},
            {'start_x': 1.45},
            {'start_y': 1.2},
        ],
    )

    # obstacle_avoidance_node = Node(
    #         package='puzzlebot_description',
    #         # 1. Cambiar si renombraste el ejecutable en tu setup.py (ej. 'obstacle_avoidance_bug0')
    #         executable='obstacle_avoidance', 
    #         name='obstacle_avoidance',
    #         output='screen',
    #         parameters=[
    #             {'linear_speed': 0.19},            # Velocidad lineal recomendada para Bug 0
    #             {'angular_speed': 0.55},           # Velocidad angular recomendada para Bug 0
    #             {'goal_tolerance': 0.15},          # Tolerancia de llegada a la meta (15 cm)
    #             {'yaw_tolerance': 0.25},
    #             {'obstacle_distance': 0.10},       # Umbral frontal para detectar el muro (40 cm)
    #             {'front_angle': 30.0},             # Cono de visión frontal (40 grados)
    #             {'scan_topic': 'scan'},
    #             {'cmd_vel_topic': 'cmd_vel'},
    #             {'goal_topic': 'next_point'},
    #             {'odom_topic': 'odom'},
                
    #             # --- NUEVO PARÁMETRO ESPECÍFICO DE BUG 0 ---
    #             {'wall_dist_target': 0.25},        # Distancia ideal para costear la pared derecha (35 cm)
    #         ],
    #     )
    
    obstacle_avoidancebug2_node = Node(
            package='puzzlebot_description',
            executable='obstacle_avoidancebug2',  # Asegúrate de que apunte al nuevo script en tu setup.py
            name='obstacle_avoidancebug2',
            output='screen',
            parameters=[
                {'linear_speed': 0.19},            
                {'angular_speed': 0.55},           
                {'goal_tolerance': 0.01},          # Tolerancia de llegada a la meta (15 cm)
                {'yaw_tolerance': 0.25},
                {'obstacle_distance': 0.10},       # Umbral frontal para detectar el muro
                {'front_angle': 30.0},             # Cono de visión frontal (30 grados)
                {'scan_topic': 'scan'},
                {'cmd_vel_topic': 'cmd_vel'},
                {'goal_topic': 'next_point'},
                {'odom_topic': 'odom'},
                {'wall_dist_target': 0.25},        # Distancia ideal para costear la pared derecha
                
                # --- NUEVO PARÁMETRO ESPECÍFICO DE BUG 2 ---
                {'m_line_tolerance': 0.10},        # Tolerancia para re-interceptar la M-Line (10 cm)
            ],
        )

    return LaunchDescription([
        declare_world_arg,
        declare_robot_name_arg,
        declare_robot_arg,
        declare_x_arg,
        declare_y_arg,
        declare_yaw_arg,
        declare_robot_lidar_frame_arg,
        include_world,
        include_robot,
        localization_node,
        kinematic_simulator_node,
        setpoint_generator_node,
        #obstacle_avoidance_node,
        obstacle_avoidancebug2_node,
    ])

"""Launch Puzzlebot visualization and the square trajectory demo in one place.

This launch starts:
- `robot_state_publisher` and `rviz2` with the package RViz config.
- `setpoint_generator` configured for a square trajectory.
- `point_stabilizer` for closed-loop motion control.
- `kinematic_simulator` to generate wheel speeds and joint states.
- `simulated_landmarks` to publish landmark detections and RViz markers that replace the real ArUco tags in simulation.
- `localization` using EKF fused with those landmark detections to publish `/odom` and TF.

The optional `joint_state_publisher_gui` remains available for manual testing,
but it is disabled by default because the simulator publishes joint states.
"""
import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('puzzlebot_description_final')

    default_urdf = os.path.join(
        pkg_share, 'urdf', 'mcr2_robots', 'puzzlebot_jetson_lidar_ed.xacro')

    declare_urdf_path = DeclareLaunchArgument(
        'urdf_path', default_value=default_urdf,
        description='Full path to xacro URDF to load')

    declare_use_joint_gui = DeclareLaunchArgument(
        'use_joint_gui', default_value='false', description='Launch joint_state_publisher_gui')

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('urdf_path')]), value_type=str
            )
        }],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_gui'))
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'puzzlebot_desc.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    setpoint_node = Node(
        package='puzzlebot_description_final',
        executable='setpoint_generator',
        name='setpoint_generator',
        output='screen',
        parameters=[{
            'trajectory_type': 'square',
            'side_length': 4.0,
            'start_x': 0.0,
            'start_y': 0.0,
            'publish_rate': 1.0,
            'loop_route': True,
        }],
    )

    point_stabilizer_node = Node(
        package='puzzlebot_description_final',
        executable='point_stabilizer',
        name='point_stabilizer',
        output='screen',
        parameters=[{
            'control_rate': 20.0,
            'position_tolerance': 0.05,
            'angle_tolerance': 0.05,
        }],
    )

    kinematic_node = Node(
        package='puzzlebot_description_final',
        executable='kinematic_simulator',
        name='kinematic_simulator',
        output='screen',
        parameters=[{
            'publish_tf': False,
            'publish_joint_states': True,
            'update_rate': 50.0,
        }],
    )

    simulated_landmarks_node = Node(
        package='puzzlebot_description_final',
        executable='simulated_landmarks',
        name='simulated_landmarks',
        output='screen',
        parameters=[{
            'pose_topic': 'pose_sim',
            'aruco_topic': '/aruco_markers',
            'visualization_topic': '/sim_landmarks',
            'max_detection_range': 1.0,
            'marker_map_json': json.dumps([
                {'id': 1, 'x': 4.9, 'y': 0.0, 'theta': 3.141592653589793},
                {'id': 3, 'x': -0.9, 'y': 4.0, 'theta': 0.0},
            ]),
            'publish_rate': 10.0,
        }],
    )

    localization_node = Node(
        package='puzzlebot_description_final',
        executable='localization',
        name='localization',
        output='screen',
        parameters=[{
            'wheel_radius': 0.05,
            'wheel_base': 0.19,
            'sample_time': 0.02,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'wr_topic': 'wr',
            'wl_topic': 'wl',
            'aruco_topic': '/aruco_markers',
            'camera_base_x': 0.1241,
            'camera_base_y': 0.0,
            'camera_base_theta': 0.0,
            'sigma_v': 0.25,
            'sigma_w': 0.25,
            # Much higher confidence in camera observations for RViz/demo
            'sigma_obs_x': 0.02,
            'sigma_obs_y': 0.02,
            'sigma_obs_theta': 0.05,
            # Do not inflate observation uncertainty with distance in this simulated demo
            'aruco_distance_gain': 0.0,
            'aruco_theta_distance_gain': 0.0,
            # Keep a finite gate while trusting the perfect simulated landmark
            'mahal_threshold': 50.0,
            'marker_map_json': json.dumps([
                {'id': 1, 'x': 4.9, 'y': 0.0, 'theta': 3.141592653589793},
                {'id': 3, 'x': -0.9, 'y': 4.0, 'theta': 0.0},
            ]),
        }],
    )

    ld = LaunchDescription([
        declare_urdf_path,
        declare_use_joint_gui,
        robot_state_pub_node,
        joint_state_publisher_node,
        rviz_node,
        setpoint_node,
        point_stabilizer_node,
        kinematic_node,
        simulated_landmarks_node,
        localization_node,
    ])

    return ld

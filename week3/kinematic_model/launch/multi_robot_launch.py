import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('kinematic_model')
    robot_group_launch = os.path.join(package_dir, 'launch', 'robot_group_launch.py')
    rviz_config = os.path.join(package_dir, 'rviz', 'puzzlebot_rviz.rviz')

    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_group_launch),
        launch_arguments={
            'namespace': 'robot1',
            'initial_x': '0.0',
            'initial_y': '0.0',
            'initial_theta': '0.0',
            'trajectory_type': 'square',
            'side_length': '0.8',
            'start_x': '0.0',
            'start_y': '0.0',
        }.items(),
    )

    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_group_launch),
        launch_arguments={
            'namespace': 'robot2',
            'initial_x': '1.5',
            'initial_y': '0.5',
            'initial_theta': '0.0',
            'trajectory_type': 'triangle',
            'side_length': '0.6',
            'start_x': '0.2',
            'start_y': '0.2',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot1,
        robot2,
        rviz,
    ])
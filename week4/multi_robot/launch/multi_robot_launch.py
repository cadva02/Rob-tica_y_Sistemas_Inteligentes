import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('multi_robot')
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
            '/robot2/next_point/x',
            '/robot2/next_point/y',
            '/robot2/pose_sim/pose/position/x',
            '/robot2/pose_sim/pose/position/y',
            '/robot2/odom/pose/pose/position/x',
            '/robot2/odom/pose/pose/position/y',
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
            '/robot2/cmd_vel/linear/x',
            '/robot2/cmd_vel/angular/z',
            '/robot2/odom/twist/twist/linear/x',
            '/robot2/odom/twist/twist/angular/z',
        ],
    )

    return LaunchDescription([
        robot1,
        robot2,
        rviz,
        rqt_tf_tree,
        rqt_graph,
        rqt_plot_pose,
        rqt_plot_vel,
    ])
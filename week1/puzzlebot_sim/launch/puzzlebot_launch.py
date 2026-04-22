from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    puzzlebot_dir = get_package_share_directory('puzzlebot_sim')
    urdf_file = os.path.join(puzzlebot_dir, 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(puzzlebot_dir, 'rviz', 'puzzlebot_rviz.rviz')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Static Transform Publisher (map -> odom)
    static_transform_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'odom']
    )
    
    # Robot State Publisher (publishes URDF and TF)
    robot_state_publisher = Node(
        package='   ',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc
        }]
    )
    
    # Joint State Publisher (publishes TF transformations)
    joint_state_publisher = Node(
        package='puzzlebot_sim',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    # RQT TF Tree
    rqt_tf_tree = Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        name='rqt_tf_tree',
        output='screen'
    )
        
    return LaunchDescription([
        static_transform_map_odom,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
        rqt_tf_tree,
    ])
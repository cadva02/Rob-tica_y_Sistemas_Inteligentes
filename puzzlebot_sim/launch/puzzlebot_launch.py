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
    
    # Robot State Publisher (publishes URDF and TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
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
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
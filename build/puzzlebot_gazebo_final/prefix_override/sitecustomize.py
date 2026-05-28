import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/adrian/ros2_ws/src/Rob-tica_y_Sistemas_Inteligentes/install/puzzlebot_gazebo_final'

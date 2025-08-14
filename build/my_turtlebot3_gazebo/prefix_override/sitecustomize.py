import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cloud/ros2_ws/install/my_turtlebot3_gazebo'

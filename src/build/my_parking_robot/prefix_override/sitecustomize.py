import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/andrisgerber/ros2_workspace/src/install/my_parking_robot'

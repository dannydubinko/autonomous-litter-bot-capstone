import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pi5/capstone-group8/ros2_ws/src/install/autonomous_litter_bot_package'

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/roman/ros2_ws/src/service_full_name/install/service_full_name'

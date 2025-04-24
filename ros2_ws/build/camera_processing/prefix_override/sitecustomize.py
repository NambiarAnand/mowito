import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/citc-26/Desktop/anand/ros2_ws/install/camera_processing'

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tugberk/Desktop/Projects/ROS-test/install/drone_avoidance'
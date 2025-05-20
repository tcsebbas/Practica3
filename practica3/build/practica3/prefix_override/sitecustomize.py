import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tcsebbas/ros2_ws/src/practica3/install/practica3'

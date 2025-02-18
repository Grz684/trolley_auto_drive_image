import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/grz/trolley_auto_drive_image/code/install/linear_displacement_sensor'

import sys
if sys.prefix == '/home/trungbao/venvs/dreamer_ros':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/trungbao/cyberrunner_ws/install/cyberrunner_hiwonder'

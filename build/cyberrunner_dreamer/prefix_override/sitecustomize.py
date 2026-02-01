import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/trungbao/cyberrunner_ws/install/cyberrunner_dreamer'

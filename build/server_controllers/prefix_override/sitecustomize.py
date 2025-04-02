import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/golf1234_pc/manipurator_ws/install/server_controllers'

#!/usr/bin/python3
import signal
import sys

import rclpy
from ck_utilities_ros2_py_node.generated.parameters import ParameterizedNode

from std_msgs.msg import String


class LocalNode(ParameterizedNode):
    def __init__(self):
        super().__init__('ck_utilities_ros2_py_node')

def signal_handler(sig, frame):
    sys.exit(0)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    node = LocalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
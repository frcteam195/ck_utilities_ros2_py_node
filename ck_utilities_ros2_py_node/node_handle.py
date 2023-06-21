from ck_utilities_ros2_py_node.singleton import Singleton
import rclpy
import rclpy.node

class NodeHandle(Singleton):
    node_handle : rclpy.node.Node = None
    def __new__(cls, node_handle = None):
        if not hasattr(cls, 'instance'):
            super().__new__(cls)
            cls.node_handle = node_handle
        return cls.instance

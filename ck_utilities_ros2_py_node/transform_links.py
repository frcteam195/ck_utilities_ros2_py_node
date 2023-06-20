import rclpy
from threading import Lock
from ck_utilities_ros2_py_node.geometry import *
import tf
import tf2_ros
import geometry_msgs.msg

class TransformManager:
    __static_transform_publisher = None
    __dynamic_transform_publisher = None

    __static_transform_list = None

    __mutex = Lock()
    __manager = None

    @classmethod
    def init_manager(cls):
        with cls.__mutex:
            if cls.__manager is None:
                cls.__manager = TransformManager()

    @classmethod
    def manager(cls):
        with cls.__mutex:
            return cls.__manager

    def __init__(self):
        self.__class__.__static_transform_publisher = tf2_ros.StaticTransformBroadcaster()
        self.__class__.__dynamic_transform_publisher = tf2_ros.TransformBroadcaster()
        self.__class__.__static_transform_list = []

    def publish_static_transform(self, transform : geometry_msgs.msg.TransformStamped):
        self.__class__.__static_transform_list.append(transform)
        self.__class__.__static_transform_publisher.sendTransform(self.__static_transform_list)

    def publish_dynamic_transform(self, transform : geometry_msgs.msg.TransformStamped):
        self.__class__.__dynamic_transform_publisher.sendTransform(transform)

class TransformBase:
    def __init__(self, name : str, base_frame : str):
        self.__transform : Transform()
        self.__base_frame : str = base_frame
        self.__name : str = name
        self.spawn_transform_manager()

    @classmethod
    def spawn_transform_manager(cls):
        TransformManager.init_manager()

    def set_transform(self, transform : Transform):
        self.__transform = transform

    @classmethod
    def manager(cls) -> TransformManager:
        return TransformManager.manager()

    def convert_to_tf2_msg(self):
        tf2_transform = geometry_msgs.msg.TransformStamped()
        tf2_transform.header.stamp = rospy.Time.now()
        tf2_transform.header.frame_id = self.__base_frame
        tf2_transform.child_frame_id = self.__name
        tf2_transform.transform.translation.x = self.__transform.linear.x
        tf2_transform.transform.translation.y = self.__transform.linear.y
        tf2_transform.transform.translation.z = self.__transform.linear.z
        quat = tf.transformations.quaternion_from_euler(
            float(self.__transform.angular.roll),
            float(self.__transform.angular.pitch),
            float(self.__transform.angular.yaw))
        tf2_transform.transform.rotation.x = quat[0]
        tf2_transform.transform.rotation.y = quat[1]
        tf2_transform.transform.rotation.z = quat[2]
        tf2_transform.transform.rotation.w = quat[3]
        return tf2_transform

class TransformLink(TransformBase):
    def __init__(self, name : str, base_frame : str):
        super().__init__(name, base_frame)

    def publish(self):
        self.manager().publish_dynamic_transform(self.convert_to_tf2_msg())

class StaticTransformLink(TransformBase):
    def __init__(self, name : str, base_frame : str):
        super().__init__(name, base_frame)

    def publish(self):
        transformed_link = self.convert_to_tf2_msg()
        transformed_link.header.stamp = rospy.Time.from_sec(0)
        self.manager().publish_static_transform(self.convert_to_tf2_msg())
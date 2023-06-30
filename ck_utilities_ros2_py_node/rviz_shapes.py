import rclpy
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from threading import Lock
from ck_utilities_ros2_py_node.geometry import *
from ck_utilities_ros2_py_node.node_handle import NodeHandle
from visualization_msgs.msg import *
import tf

#TODO: Convert to ROS2

class Color:
    def __init__(self, r : float, g : float, b, a : float):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

class ShapeManager:
    __static_shape_publisher = None

    __static_shape_map = None

    __mutex = Lock()
    __manager = None

    @classmethod
    def init_manager(cls):
        with cls.__mutex:
            if cls.__manager is None:
                cls.__manager = ShapeManager()

    @classmethod
    def manager(cls):
        with cls.__mutex:
            return cls.__manager

    def __init__(self):
        self.__qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.__class__.__static_shape_publisher = NodeHandle.node_handle.create_publisher(
            topic = "/static_shapes", msg_type=MarkerArray, qos_profile=self.__qos)
        self.__class__.__static_shape_map = {}

    def publish_static_shape(self, marker : Marker):
        if (str(marker.ns) + str(marker.id)) in self.__class__.__static_shape_map:
            raise Exception("Cannot register the same ns + id combo twice: " + str(marker.ns) + ":" + str(marker.id))
        self.__class__.__static_shape_map[str(marker.ns) + str(marker.id)] = marker

        transmit_array = MarkerArray()
        for transmit_marker_key, transmit_marker in self.__class__.__static_shape_map.items():
            transmit_array.markers.append(transmit_marker)

        self.__class__.__static_shape_publisher.publish(transmit_array)

class ShapeBase:
    __cls_id : int = 0

    def __init__(self, base_frame : str, type : int):
        self.__transform = Transform()
        self.__base_frame = base_frame
        self.__namespace = NodeHandle.node_handle.get_name()
        ShapeBase.__cls_id += 1
        self.__id = ShapeBase.__cls_id
        self.__type = type
        self.__scale = Scale(1.0, 1.0, 1.0)
        self.__color = Color(1.0, 1.0, 1.0, 1.0)
        self.spawn_shape_manager()

    @classmethod
    def spawn_shape_manager(cls):
        ShapeManager.init_manager()

    @classmethod
    def manager(cls) -> ShapeManager:
        return ShapeManager.manager()

    def set_transform(self, transform : Transform):
        self.__transform = transform

    def set_scale(self, scale : Scale):
        self.__scale = scale

    def set_color(self, color : Color):
        self.__color = color

    def convert_to_marker(self):
        marker = Marker()
        marker.header.stamp = Time(seconds=0)
        marker.header.frame_id = self.__base_frame
        marker.action = Marker.ADD
        marker.ns = self.__namespace
        marker.id = self.__id
        marker.type = self.__type
        marker.pose.position.x = self.__transform.linear.x
        marker.pose.position.y = self.__transform.linear.y
        marker.pose.position.z = self.__transform.linear.z
        quat = tf.transformations.quaternion_from_euler(
            float(self.__transform.angular.roll),
            float(self.__transform.angular.pitch),
            float(self.__transform.angular.yaw))
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = self.__scale.x
        marker.scale.y = self.__scale.y
        marker.scale.z = self.__scale.z
        marker.color.r = self.__color.r
        marker.color.g = self.__color.g
        marker.color.b = self.__color.b
        marker.color.a = self.__color.a
        marker.frame_locked = True
        return marker

class Cube(ShapeBase):
    def __init__(self, base_frame : str):
        super().__init__(base_frame, 1)

    def publish(self):
        self.manager().publish_static_shape(self.convert_to_marker())

class Sphere(ShapeBase):
    def __init__(self, base_frame : str):
        super().__init__(base_frame, 2)

    def publish(self):
        self.manager().publish_static_shape(self.convert_to_marker())

class Cylinder(ShapeBase):
    def __init__(self, base_frame : str):
        super().__init__(base_frame, 3)

    def publish(self):
        self.manager().publish_static_shape(self.convert_to_marker())

class Arrow(ShapeBase):
    def __init__(self, base_frame : str):
        super().__init__(base_frame, 0)

    def publish(self):
        self.manager().publish_static_shape(self.convert_to_marker())
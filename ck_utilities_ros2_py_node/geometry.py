import numpy
import geometry_msgs
from nav_msgs.msg import *
from std_msgs.msg import *
import tf2_ros
from dataclasses import dataclass

#TODO: Convert to ROS2

class Translation:

    def __init__(self, input_type = None):
        if input_type is None:
            self.__translation = numpy.zeros(3)
            return
        elif isinstance(input_type, geometry_msgs.msg._vector3.Vector3):
            self.__init_from_translation(input_type)
            return
        elif isinstance(input_type, geometry_msgs.msg._point.Point):
            self.__init_from_point(input_type)
            return
        elif isinstance(input_type, Translation):
            self.__translation = numpy.zeros(3)
            self.x = input_type.x
            self.y = input_type.y
            self.z = input_type.z
        raise ValueError("Type " + str(type(input_type)) + "is not supported by Translation constructor")

    def __init_from_translation(self, input_translation : geometry_msgs.msg._vector3.Vector3):
        self.__translation = numpy.zeros(3)
        self.x = input_translation.x
        self.y = input_translation.y
        self.z = input_translation.z

    def __init_from_point(self, input_point : geometry_msgs.msg._point.Point):
        self.__translation = numpy.zeros(3)
        self.x = input_point.x
        self.y = input_point.y
        self.z = input_point.z

    @property
    def x(self):
        return self.__translation[0]
    @x.setter
    def x(self, value):
        self.__translation[0] = value

    @property
    def y(self):
        return self.__translation[1]
    @y.setter
    def y(self, value):
        self.__translation[1] = value

    @property
    def z(self):
        return self.__translation[2]
    @z.setter
    def z(self, value):
        self.__translation[2] = value

    def rotate(self, rotation):
        quat = rotation.to_quaternion()
        vec = numpy.zeros(4)
        vec[0] = self.__translation[0]
        vec[1] = self.__translation[1]
        vec[2] = self.__translation[2]
        vec[3] = 0
        vec = quat * vec * quat.conjugate()
        vec = vec / numpy.linalg.norm(vec)
        vec = vec * numpy.linalg.norm(self.__translation)
        translation = Translation()
        translation.x = vec[0,0]
        translation.y = vec[1,0]
        translation.z = vec[2,0]

        return translation

    def to_msg(self) -> geometry_msgs.msg._vector3.Vector3:
        output = geometry_msgs.msg._vector3.Vector3()
        output.x = self.x
        output.y = self.y
        output.z = self.z
        return output

    def to_msg_point(self) -> geometry_msgs.msg._point.Point:
        output = geometry_msgs.msg._point.Point()
        output.x = self.x
        output.y = self.y
        output.z = self.z
        return output

class Rotation:

    def __init__(self, input_type = None):
        if input_type is None:
            self.__rotation = numpy.zeros(3)
            return
        elif isinstance(input_type, geometry_msgs.msg._vector3.Vector3):
            self.__init_from_angular(input_type)
            return
        elif isinstance(input_type, geometry_msgs.msg._quaternion.Quaternion):
            self.__init_from_quaternion(input_type)
            return
        elif isinstance(input_type, Rotation):
            self.__rotation = numpy.zeros(3)
            self.roll = input_type.roll
            self.pitch = input_type.pitch
            self.yaw = input_type.yaw
        raise ValueError("Type " + str(type(input_type)) + "is not supported by Rotation constructor")

    def __init_from_angular(self, input_angular : geometry_msgs.msg._vector3.Vector3):
        self.__rotation = numpy.zeros(3)
        self.roll = input_angular.x
        self.pitch = input_angular.y
        self.yaw = input_angular.z

    def __init_from_quaternion(self, input_quaternion : geometry_msgs.msg._quaternion.Quaternion):
        self.__rotation = numpy.zeros(3)
        quat = ([input_quaternion.x, input_quaternion.y, input_quaternion.z, input_quaternion.w])
        eulers = euler_from_quaternion(quat)
        self.roll = eulers[0]
        self.pitch = eulers[1]
        self.yaw = eulers[2]

    @property
    def roll(self) -> float:
        return self.__rotation[0]
    @roll.setter
    def roll(self, value : float):
        self.__rotation[0] = value

    @property
    def pitch(self) -> float:
        return self.__rotation[1]
    @pitch.setter
    def pitch(self, value : float):
        self.__rotation[1] = value

    @property
    def yaw(self) -> float:
        return self.__rotation[2]
    @yaw.setter
    def yaw(self, value : float):
        self.__rotation[2] = value

    def to_quaternion(self):
        quat = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        return quaternion_matrix(quat)

    def to_msg(self) -> geometry_msgs.msg._vector3.Vector3:
        output = geometry_msgs.msg._vector3.Vector3()
        output.x = self.roll
        output.y = self.pitch
        output.z = self.yaw
        return output

    def to_msg_quat(self) -> geometry_msgs.msg._quaternion.Quaternion:
        output = geometry_msgs.msg._quaternion.Quaternion()
        quats = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        output.x = quats[0]
        output.y = quats[1]
        output.z = quats[2]
        output.w = quats[3]
        return output

class RotationQuaternion:

    def __init__(self, input_type = None):
        if input_type is None:
            self.__rotation = numpy.zeros((4,4))
            return
        elif isinstance(input_type, geometry_msgs.msg._quaternion.Quaternion):
            self.__init_from_quaternion(input_type)
            return
        raise ValueError("Type " + str(type(input_type)) + "is not supported by RotationQuaternion constructor")

    def __init_from_quaternion(self, input_quaternion : geometry_msgs.msg._quaternion.Quaternion):
        self.__rotation = numpy.zeros(4,4)
        self.x = input_quaternion.x
        self.y = input_quaternion.y
        self.z = input_quaternion.z
        self.w = input_quaternion.w

    @property
    def x(self) -> float:
        return self.__rotation[0,0]
    @x.setter
    def x(self, value : float):
        self.__rotation[0,0] = value

    @property
    def y(self) -> float:
        return self.__rotation[1,1]
    @y.setter
    def y(self, value : float):
        self.__rotation[1,1] = value

    @property
    def z(self) -> float:
        return self.__rotation[2,2]
    @z.setter
    def z(self, value : float):
        self.__rotation[2,2] = value

    @property
    def w(self) -> float:
        return self.__rotation[3,3]
    @w.setter
    def w(self, value : float):
        self.__rotation[3,3] = value

class Pose:
    def __init__(self, input_type = None):
        if input_type is None:
            self.__position : Translation = Translation()
            self.__orientation : Rotation = Rotation()
            return
        elif isinstance(input_type, geometry_msgs.msg._pose.Pose):
            self.__init_from_pose(input_type)
            return
        elif isinstance(input_type, Pose):
            self.__position : Translation = Translation(input_type.position)
            self.__orientation : Rotation = Rotation(input_type.orientation)
        raise ValueError("Type " + str(type(input_type)) + "is not supported by RotationQuaternion constructor")

    def __init_from_pose(self, input_pose : geometry_msgs.msg._pose.Pose):
        self.__position = Translation(input_pose.position)
        self.__orientation = Rotation(input_pose.orientation)

    @property
    def position(self) -> Translation:
        return self.__position
    @position.setter
    def position(self, value : Translation):
        self.__position = value

    @property
    def orientation(self) -> Rotation:
        return self.__orientation
    @orientation.setter
    def orientation(self, value : Rotation):
        self.__orientation = value

    def to_msg(self) -> geometry_msgs.msg._pose.Pose:
        output = geometry_msgs.msg._pose.Pose()
        output.position = self.__position.to_msg_point()
        output.orientation = self.__orientation.to_msg_quat()
        return output

    def __repr__(self) -> str:
        return f"X: {self.__position.x}, Y: {self.__position.y}, Z: {self.__position.z}"

class Transform:
    def __init__(self, input_type = None):
        if input_type is None:
            self.__linear : Translation = Translation()
            self.__angular : Rotation = Rotation()
            return
        elif isinstance(input_type, geometry_msgs.msg._transform.Transform):
            self.__init_from_pose(input_type)
            return
        raise ValueError("Type " + str(type(input_type)) + "is not supported by RotationQuaternion constructor")

    def __init_from_pose(self, input_transform : geometry_msgs.msg._transform.Transform):
        self.__linear = Translation(input_transform.translation)
        self.__angular = Rotation(input_transform.rotation)

    @property
    def linear(self) -> Translation:
        return self.__linear
    @linear.setter
    def linear(self, value : Translation):
        self.__linear = value

    @property
    def angular(self) -> Rotation:
        return self.__angular
    @angular.setter
    def angular(self, value : Rotation):
        self.__angular = value

    def to_msg(self) -> geometry_msgs.msg._transform.Transform:
        output = geometry_msgs.msg._transform.Transform()
        output.translation = self.__linear.to_msg()
        output.rotation = self.__angular.to_msg_quat()

class Twist:
    def __init__(self, input_type = None):
        if input_type is None:
            self.__linear : Translation = Translation()
            self.__angular : Rotation = Rotation()
            return
        elif isinstance(input_type, geometry_msgs.msg._twist.Twist):
            self.__init_from_twist(input_type)
            return
        elif isinstance(input_type, geometry_msgs.msg._transform.Transform):
            self.__init_from_pose(input_type)
            return
        raise ValueError("Type " + str(type(input_type)) + "is not supported by RotationQuaternion constructor")

    def __init_from_pose(self, input_transform : geometry_msgs.msg._transform.Transform):
        self.__linear = Translation(input_transform.translation)
        self.__angular = Rotation(input_transform.rotation)

    def __init_from_twist(self, input_transform : geometry_msgs.msg._twist.Twist):
        self.__linear = Translation(input_transform.linear)
        self.__angular = Rotation(input_transform.angular)

    @property
    def linear(self) -> Translation:
        return self.__linear
    @linear.setter
    def linear(self, value : Translation):
        self.__linear = value

    @property
    def angular(self) -> Rotation:
        return self.__angular
    @angular.setter
    def angular(self, value : Rotation):
        self.__angular = value

    def to_msg(self) -> geometry_msgs.msg._twist.Twist:
        output = geometry_msgs.msg._twist.Twist()
        output.linear = self.__linear.to_msg()
        output.angular = self.__angular.to_msg()
        return output

class Scale:
    def __init__(self, x : float, y : float, z : float):
        self.x : float = x
        self.y : float = y
        self.z : float = z

class Covariance:
    def __init__(self):
        self.__covariance = numpy.zeros((6,6))

    @property
    def x_var(self) -> float:
        return self.__covariance[0,0]
    @x_var.setter
    def x_var(self, value : float):
        self.__covariance[0,0] = value

    @property
    def y_var(self) -> float:
        return self.__covariance[1,1]
    @y_var.setter
    def y_var(self, value : float):
        self.__covariance[1,1] = value

    @property
    def z_var(self) -> float:
        return self.__covariance[2,2]
    @z_var.setter
    def z_var(self, value : float):
        self.__covariance[2,2] = value

    @property
    def roll_var(self) -> float:
        return self.__covariance[3,3]
    @roll_var.setter
    def roll_var(self, value : float):
        self.__covariance[3,3] = value

    @property
    def pitch_var(self) -> float:
        return self.__covariance[4,4]
    @pitch_var.setter
    def pitch_var(self, value : float):
        self.__covariance[4,4] = value

    @property
    def yaw_var(self) -> float:
        return self.__covariance[5,5]
    @yaw_var.setter
    def yaw_var(self, value : float):
        self.__covariance[5,5] = value

    def to_msg(self):
        output = []
        for i in self.__covariance:
            for j in i:
                output.append(j)
        return output

@dataclass
class Rectangle:
    upper_left_x : float = 0
    upper_left_y : float = 0
    lower_right_x : float = 0
    lower_right_y : float = 0
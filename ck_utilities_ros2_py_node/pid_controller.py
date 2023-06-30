import rclpy
from ck_utilities_ros2_py_node.node_handle import NodeHandle

#TODO: Convert to ROS2

class PIDController:
    def __init__(self, kP : float = 0, kI : float = 0, kD : float = 0, kF : float = 0, filter_r : float = 0) -> None:
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.__filter_r = filter_r
        self.__error = 0
        self.__error_sum = 0
        self.__error_last = 0
        self.__error_d = 0
        self.__actual = 0
        self.__setpoint = 0
        self.__last_time = 0
        self.__prev_setpoint = 0

    def set_gains(self,  kP : float, kI : float, kD : float, kF : float):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF

    def set_filter(self, filter_r : float):
        self.__filter_r = filter_r

    def update(self, setpoint : float, actual : float) -> float:
        self.__setpoint = setpoint
        self.__actual = actual

        self.__error = self.__setpoint - self.__actual

        if self.__prev_setpoint != self.__setpoint:
            self.__error_sum = 0

        self.__error_sum += self.__error
        self.__error_d = self.__error_d + (1 - self.__filter_r) * (self.__error - self.__error_last)
        self.__error_last = self.__error

        time = NodeHandle.node_handle.get_clock().now()
        dt = time - self.__last_time
        self.__last_time = time
        self.__prev_setpoint = self.__setpoint

        return self.__error * self.kP + self.__error_sum * self.kI + self.__error_d * self.kD + self.kF * self.__setpoint

    def update_by_error(self, error : float) -> float:
        self.__error = error
        self.__error_sum += self.__error
        if self.kI == 0:
            self.__error_sum = 0
        self.__error_d = (1 - self.__filter_r) * (self.__error - self.__error_last)
        self.__error_last = self.__error

        time = NodeHandle.node_handle.get_clock().now()
        dt = time - self.__last_time
        self.__last_time = time

        return self.__error * self.kP + self.__error_sum * self.kI + self.__error_d * self.kD
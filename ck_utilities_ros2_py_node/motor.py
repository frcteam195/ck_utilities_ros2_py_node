#!/usr/bin/env python3

import rclpy
from dataclasses import dataclass
from threading import Thread, RLock
from ck_ros2_base_msgs_node.msg import MotorStatus
from ck_ros2_base_msgs_node.msg import MotorStatusArray
from ck_ros2_base_msgs_node.msg import MotorControl
from ck_ros2_base_msgs_node.msg import MotorControlArray
from ck_ros2_base_msgs_node.msg import MotorConfiguration
import ck_ros2_base_msgs_node.msg
from enum import Enum
from ck_utilities_ros2_py_node.ckmath import within

from ck_utilities_ros2_py_node.node_handle import NodeHandle

from typing import List

class ControlMode(Enum):
    """
    Enumeration of the various control modes.
    """
    DUTY_CYCLE = 0
    TORQUE_CURRENT = 1
    VOLTAGE = 2
    POSITION = 3
    VELOCITY = 4
    MOTION_MAGIC = 5
    NEUTRAL_OUT = 6
    STATIC_BRAKE = 7
    COAST_OUT = 8
    FOLLOWER = 9

class FeedForwardType(Enum):
    """
    Enumeration of the available feed forward types.
    """
    NONE = 0
    DUTY_CYCLE = 1
    TORQUE_CURRENT = 2
    VOLTAGE = 3

@dataclass
class OutputControl:
    """
    Data class matching the motor control message.
    """
    motor_id : int = 0
    control_mode : ControlMode = ControlMode.DUTY_CYCLE
    setpoint : float = 0.0
    feed_forward : float = 0.0
    feed_forward_type : FeedForwardType = FeedForwardType.NONE
    active_slot : int = 0

@dataclass
class MotorConfiguration:
    """
    Data class matching the motor configuration message.
    """
    motor_id : int = 0
    master_id : int = -1
    invert : bool = False
    brake_neutral : bool = False

    duty_cycle_neutral_deadband : float = 0.0
    peak_forward_duty_cycle : float = 0.0
    peak_reverse_duty_cycle : float = 0.0

    kP : List[float] = [0 for i in range(3)]
    kI : List[float] = [0 for i in range(3)]
    kD : List[float] = [0 for i in range(3)]
    kV : List[float] = [0 for i in range(3)]
    kS : List[float] =  [0 for i in range(3)]

    enable_stator_current_limit : bool = False
    stator_current_limit : float = 0.0

    enable_supply_current_limit : bool = False
    supply_current_limit : float = 0.0
    supply_current_threshold : float = 0.0
    supply_time_threshold : float = 0.0

    supply_voltage_time_constant : float = 0.0
    peak_forward_voltage : float = 0.0
    peak_reverse_voltage : float = 0.0

    torque_neutral_deadband : float = 0.0
    peak_forward_torque_current : float = 0.0
    peak_reverse_torque_current : float = 0.0

    duty_cycle_closed_loop_ramp_period : float = 0.0
    torque_current_closed_loop_ramp_period : float = 0.0
    voltage_closed_loop_ramp_period : float = 0.0

    duty_cycle_open_loop_ramp_period : float = 0.0
    torque_current_open_loop_ramp_period : float = 0.0
    voltage_open_loop_ramp_period : float = 0.0

    enable_forward_soft_limit : bool = False
    forward_soft_limit_threshold : float = 0.0

    enable_reverse_soft_limit : bool = False
    reverse_soft_limit_threshold : float = 0.0

    motion_magic_acceleration : float = 0.0
    motion_magic_cruise_velocity : float = 0.0
    motion_magic_jerk : float = 0.0


class MotorManager:
    """
    Class that manages all of the robot's motors.
    """
    def __init__(self):
        self.__motor_configurations = {}
        self.__motor_controls = {}
        self.__motor_statuses = {}

        self.__control_publisher = NodeHandle.node_handle.create_publisher("/MotorControl", Motor_Control, qos_profile=10)
        self.__configuration_publisher = NodeHandle.node_handle.create_publisher("/MotorConfiguration", Motor_Configuration, qos_profile=10)

        self.__mutex = RLock()
        x = Thread(target=self.__loop)

        qos_profile = rclpy.qos.QoSProfile(
            reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history = rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth = 1,
            durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        NodeHandle.node_handle.create_subscription("/MotorStatus", Motor_Status, callback=self.__receive_motor_status, qos_profile=qos_profile)

        x.start()


    def __receive_motor_status(self, data):
        """
        Receives the status of all the robot motors.
        """
        with self.__mutex:
            for motor in data.motors:
                self.__motor_statuses[motor.id] = motor

    def apply_motor_config(self, motor_id : int, motor_configuration : MotorConfiguration):
        """
        Applies a configuration to the local motor.
        """
        with self.__mutex:
            self.__motor_configurations[motor_id] = motor_configuration

    def update_motor_control(self, motor_id : int, output_control : OutputControl):
        """
        Updates the motor control for the local motor.
        """
        with self.__mutex:
            old_output_control = None
            if motor_id in self.__motor_controls:
                old_output_control = self.__motor_controls[motor_id]
            self.__motor_controls[motor_id] = output_control
            if old_output_control != self.__motor_controls[motor_id]:
                self.__set_motor_now(motor_id)

    def get_status(self, motor_id : int):
        """
        Gets the status for the specified motor.
        """
        with self.__mutex:
            if id in self.__motor_statuses:
                return self.__motor_statuses[motor_id]
            return None

    def get_control(self, motor_id : int):
        """
        Gets the current control for the specified motor.
        """
        with self.__mutex:
            if motor_id in self.__motor_controls:
                return self.__motor_controls[motor_id]
            return None

    @staticmethod
    def create_motor_control_message(motor_id : int, motor_control : OutputControl):
        """
        Create a motor control message.
        """
        control_message = ck_ros_base_msgs_node.msg.Motor()

        control_message.id = motor_id
        control_message.control_mode = motor_control.control_mode.value
        control_message.setpoint = motor_control.setpoint
        control_message.feed_forward = motor_control.feed_forward
        control_message.feed_forward_type = motor_control.feed_forward_type.value
        control_message.active_slot = motor_control.active_slot

        return control_message

    @staticmethod
    def create_motor_config_message(motor_id : int, motor_configuration : MotorConfiguration):
        """
        Create a motor configruation message from the internal structure.
        """
        configuration_message = Motor_Configuration()

        configuration_message.id = motor_id
        configuration_message.master_id = motor_configuration.master_id

        configuration_message.invert = motor_id
        configuration_message.brake_neutral = motor_configuration.brake_neutral

        for i in range(3):
            configuration_message.kP[i] = motor_configuration.kP[i]
            configuration_message.kI[i] = motor_configuration.kI[i]
            configuration_message.kD[i] = motor_configuration.kD[i]
            configuration_message.kV[i] = motor_configuration.kV[i]
            configuration_message.kS[i] = motor_configuration.kS[i]

        configuration_message.enable_stator_current_limit = motor_configuration.enable_stator_current_limit
        configuration_message.stator_current_limit = motor_configuration.stator_current_limit

        configuration_message.enable_supply_current_limit = motor_configuration.enable_supply_current_limit
        configuration_message.supply_current_limit = motor_configuration.supply_current_limit
        configuration_message.supply_current_threshold = motor_configuration.supply_current_threshold
        configuration_message.supply_time_threshold = motor_configuration.supply_time_threshold

        configuration_message.duty_cycle_closed_loop_ramp_period = motor_configuration.duty_cycle_closed_loop_ramp_period
        configuration_message.torque_current_closed_loop_ramp_period = motor_configuration.torque_current_closed_loop_ramp_period
        configuration_message.voltage_closed_loop_ramp_period = motor_configuration.voltage_closed_loop_ramp_period

        configuration_message.duty_cycle_open_loop_ramp_period = motor_configuration.duty_cycle_open_loop_ramp_period
        configuration_message.torque_current_open_loop_ramp_period = motor_configuration.torque_current_open_loop_ramp_period
        configuration_message.voltage_open_loop_ramp_period = motor_configuration.voltage_open_loop_ramp_period

        configuration_message.enable_forward_soft_limit = motor_configuration.enable_forward_soft_limit
        configuration_message.forward_soft_limit_threshold = motor_configuration.forward_soft_limit_threshold

        configuration_message.enable_reverse_soft_limit = motor_configuration.enable_reverse_soft_limit
        configuration_message.reverse_soft_limit_threshold = motor_configuration.reverse_soft_limit_threshold

        configuration_message.motion_magic_acceleration = motor_configuration.motion_magic_acceleration
        configuration_message.motion_magic_cruise_velocity = motor_configuration.motion_magic_cruise_velocity
        configuration_message.motion_magic_jerk = motor_configuration.motion_magic_jerk

        return configuration_message

    def __transmit_motor_configurations(self):
        """
        Publishes the motor configurations.
        """
        configuration_message = Motor_Configuration()
        configuration_message.motors = []

        for motor_id, motor_configuration in self.__motor_configurations.items():
            if motor_id in self.__motor_controls:
                configuration_message.motors.append(self.create_motor_config_message(motor_id, motor_configuration))
        self.__configuration_publisher.publish(configuration_message)

    def __transmit_motor_controls(self):
        """
        Publishs the motor controls.
        """
        control_message = Motor_Control()
        control_message.motors = []

        for motor_id, motor_control in self.__motor_controls.items():
            if motor_id in self.__motor_configurations:
                if self.__motor_configurations[motor_id]:
                    control_message.motors.append(self.create_motor_control_message(motor_id, motor_control))
        self.__control_publisher.publish(control_message)

    def __set_motor_now(self, motor_id : int):
        """
        Appends the motor's control message and immediately publishes it.
        """
        control_message = Motor_Control()
        control_message.motors = []
        control_message.motors.append(self.create_motor_control_message(motor_id, self.__motor_controls[motor_id]))
        self.__control_publisher.publish(control_message)

    def __loop(self):
        """
        Overall loop for motor configuration and control publishing.
        """
        r = rclpy.Rate(10) #10Hz
        while not rclpy.is_shutdown():
            with self.__mutex:
                self.__transmit_motor_controls()
                self.__transmit_motor_configurations()
            r.sleep()

class Motor:
    """
    Class representing the individual motors of a system.
    """
    manager = None
    mutex = RLock()

    def __init__(self, motor_id):
        """
        Initialize the motor.
        """
        self.configuration = MotorConfiguration()
        if isinstance(motor_id, int):
            self.id = motor_id
            self.spawn_motor_manager()

        elif isinstance(motor_id, str):
            if not rclpy.has_param(f"{rclpy.get_name()}/{motor_id}_motor_id"):
                raise Exception (f"Motor: {motor_id}_id is not set!")
            self.id = rclpy.get_param(f"{rclpy.get_name()}/{motor_id}_motor_id")
            self.spawn_motor_manager()
            self.__load_motor_config(motor_id)
            self.apply()

    @classmethod
    def spawn_motor_manager(cls):
        """
        Spawns the motor manager.
        """
        with cls.mutex:
            if cls.manager is None:
                cls.manager = MotorManager()

    def apply(self):
        """
        Apply a motor configuration to a motor.
        """
        with self.__class__.mutex:
            __class__.manager.apply_motor_config(self.id, self.configuration)
            # TODO: Need to properly define a master/follower motor.

    def set_kP(self, slot : int, value : float):
        """
        Sets the kP value in the specified slot.
        """
        if slot in range(3):
            self.configuration.kP[slot] = value

    def set_kI(self, slot : int, value : float):
        """
        Sets the kI value in the specified slot.
        """
        if slot in range(3):
            self.configuration.kI[slot] = value

    def set_kD(self, slot : int, value : float):
        """
        Sets the kD value in the specified slot.
        """
        if slot in range(3):
            self.configuration.kD[slot] = value

    def set_kV(self, slot : int, value : float):
        """
        Sets the kV value in the specified slot.
        """
        if slot in range(3):
            self.configuration.kV[slot] = value

    def set_kS(self, slot : int, value : float):
        """
        Sets the kS value in the specified slot.
        """
        if slot in range(3):
            self.configuration.kS[slot] = value

    def set_motion_magic_acceleration(self, value : float):
        """
        Sets the motion magic acceleration constant.
        """
        self.configuration.motion_magic_acceleration = value

    def set_motion_magic_cruise_velocity(self, value : float):
        """
        Sets the motion magic cruise velocity constant.
        """
        self.configuration.motion_magic_cruise_velocity = value

    def set_motion_magic_jerk(self, value : float):
        """
        Sets the motion magic jerk constant.
        """
        self.configuration.motion_magic_jerk = value

    def set_forward_soft_limit(self, value : float):
        """
        Enables the forward soft limit with the provided value.
        """
        self.configuration.enable_forward_soft_limit = True
        self.configuration.forward_soft_limit_threshold = value

    def enable_forward_soft_limit(self, enabled : bool):
        """
        Enables the existing forward soft limit.
        """
        self.configuration.enable_forward_soft_limit = enabled

    def set_reverse_soft_limit(self, value : float):
        """
        Enables the reverse soft limit with the provided value.
        """
        self.configuration.enable_reverse_soft_limit = True
        self.configuration.reverse_soft_limit_threshold = value

    def enable_reverse_soft_limit(self, enabled : bool):
        """
        Enables the existing reverse soft limit.
        """
        self.configuration.enable_reverse_soft_limit = enabled

    def assign_master_motor(self, motor_id : int):
        """
        Assigns a master motor for this motor to follow via ID.
        """
        self.configuration.master_id = motor_id

    def set_brake_mode(self):
        """
        Sets the motor neutral mode to brake.
        """
        self.configuration.brake_neutral = True

    def set_coast_mode(self):
        """
        Sets the motor neutral mode to coast.
        """
        self.configuration.brake_neutral = False

    def invert_motor(self, invert : bool):
        """
        Inverts the motor if input is true.
        """
        self.configuration.invert = invert

    def set_supply_current_limit(self, enabled : bool, current_limit : float, trigger_current : float, trigger_time : float):
        """
        Enables/disables the supply current limit and its parameters.
        """
        self.configuration.enable_supply_current_limit = enabled
        self.configuration.supply_current_limit = current_limit
        self.configuration.supply_current_threshold = trigger_current
        self.configuration.supply_time_threshold = trigger_time

    def set_stator_current_limit(self, enabled : bool, current_limit : float):
        """
        Enables/disables the stator current limit.
        """
        self.configuration.enable_stator_current_limit = enabled
        self.configuration.stator_current_limit = current_limit

    def set_peak_forward_duty_cycle(self, peak_forward_duty_cycle : float):
        """
        Sets the peak forward duty cycle.
        """
        self.configuration.peak_forward_duty_cycle = peak_forward_duty_cycle

    def set_peak_reverse_duty_cycle(self, peak_reverse_duty_cycle : float):
        """
        Sets the peak reverse duty cycle.
        """
        self.configuration.peak_reverse_duty_cycle = peak_reverse_duty_cycle

    def set_duty_cycle_neutral_deadband(self, duty_cycle_neutral_deadband : float):
        """
        Sets the duty cycle neutral deadband.
        """
        self.configuration.duty_cycle_neutral_deadband = duty_cycle_neutral_deadband

    def set_supply_voltage_time_constant(self, supply_voltage_time_constant : float):
        """
        Sets the supply voltage time constant.
        """
        self.configuration.supply_voltage_time_constant = supply_voltage_time_constant

    def set_peak_forward_voltage(self, peak_forward_voltage : float):
        """
        Sets the peak forward voltage.
        """
        self.configuration.peak_forward_voltage = peak_forward_voltage

    def set_peak_reverse_voltage(self, peak_reverse_voltage : float):
        """
        Sets the peak reverse voltage.
        """
        self.configuration.peak_reverse_voltage = peak_reverse_voltage

    def set_torque_neutral_deadband(self, torque_neutral_deadband : float):
        """
        Sets the torque neutral deadband.
        """
        self.configuration.torque_neutral_deadband = torque_neutral_deadband

    def set_peak_forward_torque_current(self, peak_forward_torque_current : float):
        """
        Sets the peak forward torque current.
        """
        self.configuration.peak_forward_torque_current = peak_forward_torque_current

    def set_peak_reverse_torque_current(self, peak_reverse_torque_current : float):
        """
        Sets the peak reverse torque current.
        """
        self.configuration.peak_reverse_torque_current = peak_reverse_torque_current

    def set_duty_cycle_closed_loop_ramp_period(self, duty_cycle_closed_loop_ramp_period : float):
        """
        Sets the duty cycle closed loop ramp period.
        """
        self.configuration.duty_cycle_closed_loop_ramp_period = duty_cycle_closed_loop_ramp_period

    def set_torque_current_closed_loop_ramp_period(self, torque_current_closed_loop_ramp_period : float):
        """
        Sets the torque current closed loop ramp period.
        """
        self.configuration.torque_current_closed_loop_ramp_period = torque_current_closed_loop_ramp_period

    def set_voltage_closed_loop_ramp_period(self, voltage_closed_loop_ramp_period : float):
        """
        Sets the voltage closed loop ramp period.
        """
        self.configuration.voltage_closed_loop_ramp_period = voltage_closed_loop_ramp_period

    def set_duty_cycle_open_loop_ramp_period(self, duty_cycle_open_loop_ramp_period : float):
        """
        Sets the duty cycle open loop ramp period.
        """
        self.configuration.duty_cycle_open_loop_ramp_period = duty_cycle_open_loop_ramp_period

    def set_torque_current_open_loop_ramp_period(self, torque_current_open_loop_ramp_period : float):
        """
        Sets the torque current open loop ramp period.
        """
        self.configuration.torque_current_open_loop_ramp_period = torque_current_open_loop_ramp_period

    def set_voltage_open_loop_ramp_period(self, voltage_open_loop_ramp_period : float):
        """
        Sets the voltage open loop ramp period.
        """
        self.configuration.voltage_open_loop_ramp_period = voltage_open_loop_ramp_period

    def set_defaults(self):
        """
        Sets the configuration to the result of the default constructor.
        """
        self.configuration = MotorConfiguration()

    def set(self, mode : ControlMode, setpoint : float, feed_forward : float = 0, feed_forward_type : FeedForwardType = FeedForwardType.NONE):
        """
        Set the motor control.
        """
        output_control = OutputControl()
        output_control.control_mode = mode
        output_control.setpoint = setpoint
        output_control.feed_forward = feed_forward
        output_control.feed_forward_type = feed_forward_type
        with self.__class__.mutex:
            self.__class__.manager.update_motor_control(self.id, output_control)

    def __get_status(self) -> Motor_Status:
        with self.__class__.mutex:
            return __class__.manager.get_status(self.id)

    def __get_control(self) -> OutputControl:
        """
        Returns the output control for the motor.
        """
        with self.__class__.mutex:
            return __class__.manager.get_control(self.id)

    def get_sensor_position(self) -> float:
        """
        Gets the sensor position of the motor.
        """
        status = self.__get_status()
        if status is not None:
            return status.sensor_position
        return 0.0

    def get_sensor_velocity(self) -> float:
        """
        Gets the sensor velocity of the motor.
        """
        status = self.__get_status()
        if status is not None:
            return status.sensor_velocity
        return 0.0

    def get_bus_voltage(self) -> float:
        """
        Gets the bus voltage.
        """
        status = self.__get_status()
        if status is not None:
            return status.bus_voltage
        return 0.0

    def get_bus_current(self) -> float:
        """
        Gets the bus current.
        """
        status = self.__get_status()
        if status is not None:
            return status.bus_current
        return 0.0

    def get_stator_current(self) -> float:
        """
        Gets the stator current.
        """
        status = self.__get_status()
        if status is not None:
            return status.stator_current
        return 0.0

    def get_forward_limit_closed(self) -> bool:
        """
        Returns true if the forward limit is closed.
        """
        status = self.__get_status()
        if status is not None:
            return status.forward_limit_closed
        return False

    def get_reverse_limit_closed(self) -> bool:
        """
        Returns true if the reverse limit is closed.
        """
        status = self.__get_status()
        if status is not None:
            return status.reverse_limit_closed
        return False

    def get_control_mode(self) -> int:
        """
        Gets the control mode.
        """
        status = self.__get_status()
        if status is not None:
            return status.control_mode
        return 0

    def get_commanded_output(self) -> float:
        """
        Gets the commanded output.
        """
        status = self.__get_status()
        if status is not None:
            return status.commanded_output
        return 0.0

    def get_active_trajectory_arbff(self) -> float:
        """
        Gets the active trajectory's arbitrary feed forward.
        """
        status = self.__get_status()
        if status is not None:
            return status.active_trajectory_arbff
        return 0.0

    def get_active_trajectory_position(self) -> float:
        """
        Gets the position of the active trajectory.
        """
        status = self.__get_status()
        if status is not None:
            return status.active_trajectory_position
        return 0.0

    def get_active_trajectory_velocity(self) -> float:
        """
        Gets the active trajectory velocity.
        """
        status = self.__get_status()
        if status is not None:
            return status.active_trajectory_velocity
        return 0.0

    def get_raw_closed_loop_error(self) -> float:
        """
        Gets the raw closed loop error.
        """
        status = self.__get_status()
        if status is not None:
            return status.raw_closed_loop_error
        return 0.0

    def get_raw_integral_accum(self) -> float:
        """
        Gets the raw integral accumulation.
        """
        status = self.__get_status()
        if status is not None:
            return status.raw_integral_accum
        return 0.0

    def get_raw_error_derivative(self) -> float:
        """
        Gets the raw error derivative.
        """
        status = self.__get_status()
        if status is not None:
            return status.raw_error_derivative
        return 0.0

    def get_raw_output_percent(self) -> float:
        """
        Gets the raw output percentage.
        """
        status = self.__get_status()
        if status is not None:
            return status.raw_output_percent
        return 0.0

    def get_setpoint(self) -> float:
        """
        Gets the current setpoint.
        """
        return self.__get_control().output

    def is_at_setpoint(self, setpoint_delta_threshold : float) -> bool:
        """
        Returns true if the setpoint has been achieved.
        """
        control : OutputControl = self.__get_control()
        status : Motor_Status = self.__get_status()
        if control is None or status is None:
            return False

        control_mode = control.controlMode
        setpoint = control.output

        if control_mode == ControlMode.MOTION_MAGIC or control_mode == ControlMode.POSITION:
            return within(setpoint, status.sensor_position, setpoint_delta_threshold)
        elif control_mode == ControlMode.VELOCITY:
            return within(setpoint, status.sensor_velocity, setpoint_delta_threshold)
        elif control_mode == ControlMode.TORQUE_CURRENT:
            return within(setpoint, status.bus_current, setpoint_delta_threshold)
        # TODO: Verify the bus voltage or duty cycle out.
        else:
            return True
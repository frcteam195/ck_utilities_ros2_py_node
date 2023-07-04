#!/usr/bin/env python3

import rclpy
from rclpy.parameter import Parameter
from dataclasses import dataclass
from threading import Thread, RLock
from ck_ros2_base_msgs_node.msg import MotorStatus
from ck_ros2_base_msgs_node.msg import MotorStatusArray
from ck_ros2_base_msgs_node.msg import MotorControl
from ck_ros2_base_msgs_node.msg import MotorControlArray
from ck_ros2_base_msgs_node.msg import MotorConfiguration as MotorConfigurationMsg
from ck_ros2_base_msgs_node.msg import MotorConfigurationArray
from enum import Enum
from ck_utilities_ros2_py_node.ckmath import within

from ck_utilities_ros2_py_node.node_handle import NodeHandle

from typing import List

class NeutralMode(Enum):
    Coast = 1
    Brake = 2

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
    gain_slot : int = 0

@dataclass
class MotorConfiguration:
    """
    Data class matching the motor configuration message.
    """
    motor_id : int = 0
    master_id : int = -1
    invert : bool = False
    brake_neutral : bool = False

    # duty_cycle_neutral_deadband : float = 0.0
    # peak_forward_duty_cycle : float = 0.0
    # peak_reverse_duty_cycle : float = 0.0

    k_p : List[float] = [0 for i in range(3)]
    k_i : List[float] = [0 for i in range(3)]
    k_d : List[float] = [0 for i in range(3)]
    k_v : List[float] = [0 for i in range(3)]
    k_s : List[float] =  [0 for i in range(3)]

    enable_stator_current_limit : bool = False
    stator_current_limit : float = 0.0

    enable_supply_current_limit : bool = False
    supply_current_limit : float = 0.0
    supply_current_threshold : float = 0.0
    supply_time_threshold : float = 0.0

    # supply_voltage_time_constant : float = 0.0
    # peak_forward_voltage : float = 0.0
    # peak_reverse_voltage : float = 0.0

    # torque_neutral_deadband : float = 0.0
    # peak_forward_torque_current : float = 0.0
    # peak_reverse_torque_current : float = 0.0

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

        self.__control_publisher = NodeHandle.node_handle.create_publisher("/MotorControl", MotorControlArray, qos_profile=10)
        self.__configuration_publisher = NodeHandle.node_handle.create_publisher("/MotorConfiguration", MotorConfigurationArray, qos_profile=10)

        self.__mutex = RLock()
        x = Thread(target=self.__loop)

        qos_profile = rclpy.qos.QoSProfile(
            reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history = rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth = 1,
            durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        NodeHandle.node_handle.create_subscription("/MotorStatus", MotorStatusArray, callback=self.__receive_motor_status, qos_profile=qos_profile)

        x.start()


    def __receive_motor_status(self, data : MotorStatusArray):
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
        control_message = MotorControl()

        control_message.id = motor_id
        control_message.control_mode = motor_control.control_mode.value
        control_message.setpoint = motor_control.setpoint
        control_message.feed_forward = motor_control.feed_forward
        control_message.feed_forward_type = motor_control.feed_forward_type.value
        control_message.gain_slot = motor_control.gain_slot

        return control_message

    @staticmethod
    def create_motor_config_message(motor_id : int, motor_configuration : MotorConfiguration):
        """
        Create a motor configruation message from the internal structure.
        """
        configuration_message = MotorConfigurationMsg()

        configuration_message.id = motor_id
        configuration_message.master_id = motor_configuration.master_id

        configuration_message.invert = motor_id
        configuration_message.brake_neutral = motor_configuration.brake_neutral

        for i in range(3):
            configuration_message.k_p[i] = motor_configuration.k_p[i]
            configuration_message.k_i[i] = motor_configuration.k_i[i]
            configuration_message.k_d[i] = motor_configuration.k_d[i]
            configuration_message.k_v[i] = motor_configuration.k_v[i]
            configuration_message.k_s[i] = motor_configuration.k_s[i]

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
        configuration_message = MotorConfigurationArray()
        configuration_message.motors = []

        for motor_id, motor_configuration in self.__motor_configurations.items():
            if motor_id in self.__motor_controls:
                configuration_message.motors.append(self.create_motor_config_message(motor_id, motor_configuration))
        self.__configuration_publisher.publish(configuration_message)

    def __transmit_motor_controls(self):
        """
        Publishs the motor controls.
        """
        control_message = MotorControlArray()
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
        control_message = MotorControlArray()
        control_message.motors = []
        control_message.motors.append(self.create_motor_control_message(motor_id, self.__motor_controls[motor_id]))
        self.__control_publisher.publish(control_message)

    def __loop(self):
        """
        Overall loop for motor configuration and control publishing.
        """
        rate = NodeHandle.node_handle.create_rate(10)
        while rclpy.ok():
            with self.__mutex:
                self.__transmit_motor_controls()
                self.__transmit_motor_configurations()
            rate.sleep()

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
            node_name = NodeHandle.node_handle.get_name()
            if not NodeHandle.node_handle.has_parameter(f"{node_name}/{motor_id}_motor_id"):
                raise Exception (f"Motor: {motor_id}_id is not set!")
            self.id = NodeHandle.node_handle.get_parameter(f"{node_name}/{motor_id}_motor_id").value()
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

    def follow(self, motor_id : int):
        """
        Assigns a master motor for this motor to follow via ID.
        """
        self.configuration.master_id = motor_id

    def set_kP(self, slot : int, value : float):
        """
        Sets the kP value in the specified slot.
        """
        if slot in range(3):
            self.configuration.k_p[slot] = value

    def set_kI(self, slot : int, value : float):
        """
        Sets the kI value in the specified slot.
        """
        if slot in range(3):
            self.configuration.k_i[slot] = value

    def set_kD(self, slot : int, value : float):
        """
        Sets the kD value in the specified slot.
        """
        if slot in range(3):
            self.configuration.k_d[slot] = value

    def set_kV(self, slot : int, value : float):
        """
        Sets the kV value in the specified slot.
        """
        if slot in range(3):
            self.configuration.k_v[slot] = value

    def set_kS(self, slot : int, value : float):
        """
        Sets the kS value in the specified slot.
        """
        if slot in range(3):
            self.configuration.k_s[slot] = value

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

    def set_forward_soft_limit_enable(self, enabled : bool):
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

    def set_reverse_soft_limit_enable(self, enabled : bool):
        """
        Enables the existing reverse soft limit.
        """
        self.configuration.enable_reverse_soft_limit = enabled

    def set_neutral_mode(self, neutral_mode : NeutralMode):
        """
        Sets the motor neutral mode to brake.
        """
        self.configuration.brake_neutral = (neutral_mode == NeutralMode.Brake)

    def set_inverted(self, invert : bool):
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

    # def set_peak_forward_duty_cycle(self, peak_forward_duty_cycle : float):
    #     """
    #     Sets the peak forward duty cycle.
    #     """
    #     self.configuration.peak_forward_duty_cycle = peak_forward_duty_cycle

    # def set_peak_reverse_duty_cycle(self, peak_reverse_duty_cycle : float):
    #     """
    #     Sets the peak reverse duty cycle.
    #     """
    #     self.configuration.peak_reverse_duty_cycle = peak_reverse_duty_cycle

    # def set_duty_cycle_neutral_deadband(self, duty_cycle_neutral_deadband : float):
    #     """
    #     Sets the duty cycle neutral deadband.
    #     """
    #     self.configuration.duty_cycle_neutral_deadband = duty_cycle_neutral_deadband

    # def set_supply_voltage_time_constant(self, supply_voltage_time_constant : float):
    #     """
    #     Sets the supply voltage time constant.
    #     """
    #     self.configuration.supply_voltage_time_constant = supply_voltage_time_constant

    # def set_peak_forward_voltage(self, peak_forward_voltage : float):
    #     """
    #     Sets the peak forward voltage.
    #     """
    #     self.configuration.peak_forward_voltage = peak_forward_voltage

    # def set_peak_reverse_voltage(self, peak_reverse_voltage : float):
    #     """
    #     Sets the peak reverse voltage.
    #     """
    #     self.configuration.peak_reverse_voltage = peak_reverse_voltage

    # def set_torque_neutral_deadband(self, torque_neutral_deadband : float):
    #     """
    #     Sets the torque neutral deadband.
    #     """
    #     self.configuration.torque_neutral_deadband = torque_neutral_deadband

    # def set_peak_forward_torque_current(self, peak_forward_torque_current : float):
    #     """
    #     Sets the peak forward torque current.
    #     """
    #     self.configuration.peak_forward_torque_current = peak_forward_torque_current

    # def set_peak_reverse_torque_current(self, peak_reverse_torque_current : float):
    #     """
    #     Sets the peak reverse torque current.
    #     """
    #     self.configuration.peak_reverse_torque_current = peak_reverse_torque_current

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

    def set(self, mode : ControlMode, setpoint : float, feed_forward_type : FeedForwardType, feed_forward : float = 0, gain_slot : int = 0):
        """
        Set the motor control.
        """
        output_control = OutputControl()
        output_control.motor_id = self.id
        output_control.control_mode = mode
        output_control.setpoint = setpoint
        output_control.feed_forward = feed_forward
        output_control.feed_forward_type = feed_forward_type
        output_control.gain_slot = gain_slot
        with self.__class__.mutex:
            self.__class__.manager.update_motor_control(self.id, output_control)

    def __get_status(self) -> MotorStatus:
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
        return self.__get_control().setpoint

    def is_at_setpoint(self, setpoint_delta_threshold : float) -> bool:
        """
        Returns true if the setpoint has been achieved.
        """
        control : OutputControl = self.__get_control()
        status : MotorStatus = self.__get_status()
        if control is None or status is None:
            return False

        control_mode = control.control_mode
        setpoint = control.setpoint

        if control_mode == ControlMode.MOTION_MAGIC or control_mode == ControlMode.POSITION:
            return within(setpoint, status.sensor_position, setpoint_delta_threshold)
        elif control_mode == ControlMode.VELOCITY:
            return within(setpoint, status.sensor_velocity, setpoint_delta_threshold)
        elif control_mode == ControlMode.TORQUE_CURRENT:
            return within(setpoint, status.bus_current, setpoint_delta_threshold)
        # TODO: Verify the bus voltage or duty cycle out.
        else:
            return True


    def __ros_motor_config_validation(self, motor_string):
        config_strings = {
            "_motor_id",
            "_master_id",
            "_invert",
            "_brakeNeutral",
            "_kP",
            "_kI",
            "_kD",
            "_kV",
            "_kS",
            "_enableStatorCurrentLimit",
            "_statorCurrentLimit",
            "_enableSupplyCurrentLimit",
            "_supplyCurrentLimit",
            "_supplyCurrentTreshhold",
            "_supplyTimeThreshold",
            "_dutyCycleClosedLoopRampPeriod",
            "_torqueCurrentClosedLoopRampPeriod",
            "_voltageClosedLoopRampPeriod",
            "_dutyCycleOpenLoopRampPeriod",
            "_torqueCurrentOpenLoopRampPeriod",
            "_voltageCurrentOpenLoopRampPeriod",
            "_enableForwardSoftLimit",
            "_forwardSoftLimitThreshold",
            "_enableReverseSoftLimit",
            "_reverseSoftLimitThreshold",
            "_motionMagicAcceleration",
            "_motionMagicCruiseVelocity",
            "_motionMagicJerk"
        }

        for config_string in config_strings:
            if not NodeHandle.node_handle.has_parameter("/" + NodeHandle.node_handle.get_name() + "/" + motor_string+config_string):
                raise Exception("Motor: " + motor_string + " is missing config definition for: " + config_string)


    def __load_motor_config(self, motor_string):
        self.__ros_motor_config_validation(motor_string)

        node = NodeHandle.node_handle

        self.configuration.master_id = node.get_parameter(node.get_name() + "/" + motor_string + "_master_id").value()
        self.configuration.invert = node.get_parameter(node.get_name() + "/" + motor_string + "_invert").value()
        self.configuration.brake_neutral = node.get_parameter(node.get_name() + "/" + motor_string + "_brakeNeutral").value()
        self.configuration.k_p = node.get_parameter(node.get_name() + "/" + motor_string + "_kP").value()
        self.configuration.k_i = node.get_parameter(node.get_name() + "/" + motor_string + "_kI").value()
        self.configuration.k_d = node.get_parameter(node.get_name() + "/" + motor_string + "_kD").value()
        self.configuration.k_v = node.get_parameter(node.get_name() + "/" + motor_string + "_kF").value()
        self.configuration.k_s = node.get_parameter(node.get_name() + '/' + motor_string + '_kS').value()
        self.configuration.enable_stator_current_limit = node.get_parameter(node.get_name() + '/' + motor_string + '_enableStatorCurrentLimit').value()
        self.configuration.stator_current_limit = node.get_parameter(node.get_name() + '/' + motor_string + '_statorCurrentLimit').value()
        self.configuration.enable_supply_current_limit = node.get_parameter(node.get_name() + '/' + motor_string + '_enableSupplyCurrentLimit').value()
        self.configuration.supply_current_limit = node.get_parameter(node.get_name() + '/' + motor_string + '_supplyCurrentLimit').value()
        self.configuration.supply_current_threshold = node.get_parameter(node.get_name() + '/' + motor_string + '_supplyCurrentLimitThreshold').value()
        self.configuration.supply_time_threshold = node.get_parameter(node.get_name() + '/' + motor_string + '_supplyTimeThreshold').value()
        self.configuration.duty_cycle_closed_loop_ramp_period = node.get_parameter(node.get_name() + '/' + motor_string + '_dutyCycleClosedLoopRampPeriod').value()
        self.configuration.torque_current_closed_loop_ramp_period = node.get_parameter(node.get_name() + '/' + motor_string + '_torqueCurrentClosedLoopRampPeriod').value()
        self.configuration.voltage_closed_loop_ramp_period = node.get_parameter(node.get_name() + '/' + motor_string + '_voltageClosedLoopRampPeriod').value()
        self.configuration.duty_cycle_open_loop_ramp_period = node.get_parameter(node.get_name() + '/' + motor_string + '_dutyCycleOpenLoopRampPeriod').value()
        self.configuration.torque_current_open_loop_ramp_period = node.get_parameter(node.get_name() + '/' + motor_string + '_torqueCurrentOpenLoopRampPeriod').value()
        self.configuration.voltage_open_loop_ramp_period = node.get_parameter(node.get_name() + '/' + motor_string + '_voltageOpenLoopRampPeriod').value()
        self.configuration.enable_forward_soft_limit = node.get_parameter(node.get_name() + '/' + motor_string + '_enableForwardSoftLimit').value()
        self.configuration.forward_soft_limit_threshold = node.get_parameter(node.get_name() + '/' + motor_string + '_forwardSoftLimitThreshold').value()
        self.configuration.enable_reverse_soft_limit = node.get_parameter(node.get_name() + '/' + motor_string + '_enableReverseSoftLimit').value()
        self.configuration.reverse_soft_limit_threshold = node.get_parameter(node.get_name() + '/' + motor_string + '_reverseSoftLimitThreshold').value()
        self.configuration.motion_magic_acceleration = node.get_parameter(node.get_name() + '/' + motor_string + '_motionMagicAcceleration').value()
        self.configuration.motion_magic_cruise_velocity = node.get_parameter(node.get_name() + '/' + motor_string + '_motionMagicCruiseVelocity').value()
        self.configuration.motion_magic_jerk = node.get_parameter(node.get_name() + '/' + motor_string + '_motionMagicJerk').value()
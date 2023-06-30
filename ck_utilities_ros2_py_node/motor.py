#!/usr/bin/env python3

import rclpy
from dataclasses import dataclass
from threading import Thread, RLock
from ck_ros2_base_msgs_node.msg import Motor_Status
from ck_ros2_base_msgs_node.msg import Motor_Control
from ck_ros2_base_msgs_node.msg import Motor_Configuration
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

class MotorType(Enum):
    """
    Enumeration of CTRE motor types.
    """
    TALON_FX = 0
    TALON_SRX = 1

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
    master_id : int = 0
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
        self.motor_configurations = {}
        self.motor_controls = {}
        self.motor_statuses = {}

        self.control_publisher = NodeHandle.node_handle.create_publisher("/MotorControl", Motor_Control, qos_profile=10)
        self.configuration_publisher = NodeHandle.node_handle.create_publisher("/MotorConfiguration", Motor_Configuration, qos_profile=10)

        self.__mutex = RLock()
        x = Thread(target=self.loop)

        qos_profile = rclpy.qos.QoSProfile(
            reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history = rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth = 1,
            durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        NodeHandle.node_handle.create_subscription("/MotorStatus", Motor_Status, callback=self.receive_motor_status, qos_profile=qos_profile)

        x.start()


    def receive_motor_status(self, data):
        """
        Receives the status of all the robot motors.
        """
        with self.__mutex:
            for motor in data.motors:
                self.motor_statuses[motor.id] = motor

    def apply_motor_config(self, motor_id : int, motor_configuration : MotorConfiguration):
        """
        Applies a configuration to the local motor.
        """
        with self.__mutex:
            self.motor_configurations[motor_id] = motor_configuration

    def update_motor_control(self, motor_id : int, output_control : OutputControl):
        """
        Updates the motor control for the local motor.
        """
        with self.__mutex:
            old_output_control = None
            if motor_id in self.motor_controls:
                old_output_control = self.motor_controls[motor_id]
            self.motor_controls[motor_id] = output_control
            if old_output_control != self.motor_controls[motor_id]:
                self.__set_motor_now(motor_id, output_control)

    def get_status(self, motor_id : int):
        """
        Gets the status for the specified motor.
        """
        with self.__mutex:
            if id in self.motor_statuses:
                return self.motor_statuses[motor_id]
            return None

    def get_control(self, motor_id : int):
        """
        Gets the current control for the specified motor.
        """
        with self.__mutex:
            if motor_id in self.motor_controls:
                return self.motor_controls[motor_id]
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

    def transmit_motor_configurations(self):
        """
        Publishes the motor configurations.
        """
        configuration_message = Motor_Configuration()
        configuration_message.motors = []

        for motor_id, motor_configuration in self.motor_configurations.items():
            if motor_id in self.motor_controls:
                configuration_message.motors.append(self.create_motor_config_message(motor_id, motor_configuration))
        self.configuration_publisher.publish(configuration_message)

    def transmit_motor_controls(self):
        """
        Publishs the motor controls.
        """
        control_message = Motor_Control()
        control_message.motors = []

        for motor_id, motor_control in self.motor_controls.items():
            if motor_id in self.motor_configurations:
                if self.motor_configurations[motor_id]:
                    control_message.motors.append(self.create_motor_control_message(motor_id, motor_control))
        self.control_publisher.publish(control_message)

    def set_motor_now(self, motor_id : int):
        """
        Appends the motor's control message and immediately publishes it.
        """
        control_message = Motor_Control()
        control_message.motors = []
        control_message.motors.append(self.create_motor_control_message(motor_id, self.motor_controls[motor_id]))
        self.control_publisher.publish(control_message)

    def loop(self):
        """
        Overall loop for motor configuration and control publishing.
        """
        r = rclpy.Rate(10) #10Hz
        while not rclpy.is_shutdown():
            with self.__mutex:
                self.transmit_motor_controls()
                self.transmit_motor_configurations()
            r.sleep()

class Motor:
    manager = None
    mutex = RLock()

    def __init__(self, *args):
        self.config = MotorConfig()
        if isinstance(args[0], int) and isinstance(args[1], MotorType):
            self.id = args[0]
            self.type = args[1]
            self.config.type = args[1]
            self.spawn_motor_manager()
        elif isinstance(args[0], str) and isinstance(args[1], MotorType):
            if not rospy.has_param(rospy.get_name() + "/" + args[0] + "_motor_id"):
                raise Exception ("Motor: " + args[0] + "_id is not set!")
            self.id = rospy.get_param(rospy.get_name() + "/" + args[0] + "_motor_id")
            self.type = args[1]
            self.config.type = args[1]
            self.spawn_motor_manager()
            self.__load_motor_config(args[0])
            self.apply()

    @classmethod
    def spawn_motor_manager(cls):
        with cls.mutex:
            if cls.manager is None:
                cls.manager = MotorManager()

    def apply(self):
        with self.__class__.mutex:
            __class__.manager.apply_motor_config(self.id, self.config)
            if self.config.followingEnabled:
                self.set(ControlMode.FOLLOWER, self.config.followerId, 0.)

    def set_fast_master(self, enable : bool):
        self.config.fast_master = enable

    def set_kP(self, value : float):
        self.config.kP = value

    def set_kI(self, value : float):
        self.config.kI = value

    def set_kD(self, value : float):
        self.config.kD = value

    def set_kF(self, value : float):
        self.config.kF = value

    def set_kP_Slot1(self, value : float):
        self.config.kP_1 = value

    def set_kI_Slot1(self, value : float):
        self.config.kI_1 = value

    def set_kD_Slot1(self, value : float):
        self.config.kD_1 = value

    def set_kF_Slot1(self, value : float):
        self.config.kF_1 = value

    def set_active_gain_slot(self, slotIdx : int):
        self.config.active_gain_slot = slotIdx

    def set_i_zone(self, value : float):
        self.config.iZone = value

    def set_max_i_accum(self, value : float):
        self.config.maxIAccum = value

    def set_allowed_closed_loop_error(self, value : float):
        self.config.allowedClosedLoopError = value

    def set_max_closed_loop_peak_output(self, value : float):
        self.config.closedLoopRamp = value

    def set_motion_cruise_velocity(self, value : float):
        self.config.motionCruiseVelocity = value

    def set_motion_acceleration(self, value : float):
        self.config.motionCruiseAcceleration = value

    def set_motion_s_curve_strength(self, value : float):
        self.config.motionSCurveStrength = value

    def set_forward_soft_limit(self, value : float):
        self.config.forwardSoftLimitEnable = True
        self.config.forwardSoftLimit = value

    def set_forward_soft_limit_enable(self, enabled : bool):
        self.config.forwardSoftLimitEnable = enabled

    def set_reverse_soft_limit(self, value : float):
        self.config.reverseSoftLimitEnable = True
        self.config.reverseSoftLimit = value

    def set_reverse_soft_limit_enable(self, enabled : bool):
        self.config.reverseSoftLimitEnable = enabled

    def set_feedback_sensor_coefficient(self, value : float):
        self.config.feedbackSensorCoefficient = value

    def set_voltage_compensation_saturation(self, value : float):
        self.config.voltageCompensationSaturation = value

    def set_voltage_compensation_enabled(self, enabled : bool):
        self.config.voltageCompensationEnabled = enabled

    def set_inverted(self, enabled : bool):
        self.config.inverted = enabled

    def set_sensor_phase_inverted(self, enabled : bool):
        self.config.sensorPhaseInverted = enabled

    def set_neutral_mode(self, mode : NeutralMode):
        self.config.neutralMode = mode

    def set_open_loop_ramp(self, value : float):
        self.config.openLoopRamp = value

    def set_closed_loop_ramp(self, value : float):
        self.config.closedLoopRamp = value

    def set_supply_current_limit(self, enabled : bool, current_limit : float, trigger_current : float, trigger_time : float):
        self.config.supplyCurrentLimitEnable = enabled
        self.config.supplyCurrentLimit = current_limit
        self.config.supplyCurrentLimitThresholdCurrent = trigger_current
        self.config.supplyCurrentLimitThresholdTime = trigger_time

    def set_stator_current_limit(self, enabled : bool, current_limit : float, trigger_current : float, trigger_time : float):
        self.config.statorCurrentLimitEnable = enabled
        self.config.statorCurrentLimit = current_limit
        self.config.statorCurrentLimitThresholdCurrent = trigger_current
        self.config.statorCurrentLimitThresholdTime = trigger_time

    def set_follower(self, enabled : bool, master_id : float):
        self.config.followingEnabled = enabled
        self.config.followerId = master_id

    def set_forward_limit_switch(self, forward_limit_switch_source : LimitSwitchSource, forward_limit_switch_normal : LimitSwitchNormal):
        self.config.forwardLimitSwitchSource = forward_limit_switch_source
        self.config.forwardLimitSwitchNormal = forward_limit_switch_normal

    def set_reverse_limit_switch(self, reverse_limit_switch_source : LimitSwitchSource, reverse_limit_switch_normal : LimitSwitchNormal):
        self.config.reverseLimitSwitchSource = reverse_limit_switch_source
        self.config.reverseLimitSwitchNormal = reverse_limit_switch_normal

    def set_peak_output_forward(self, value : float):
        self.config.peakOutputForward = value

    def set_peak_output_reverse(self, value : float):
        self.config.peakOutputReverse = value

    def set_defaults(self):
        self.config = MotorConfig()
        self.config.type = self.type

    def set(self, controlMode : ControlMode, output : float, arbitraryFeedForward : float=0):
        outputControl = OutputControl()
        outputControl.controlMode = controlMode
        outputControl.output = output
        outputControl.arbFF = arbitraryFeedForward
        outputControl.type = self.type
        with self.__class__.mutex:
            self.__class__.manager.update_motor_control(self.id, outputControl)

    def __get_status(self) -> Motor_Info:
        with self.__class__.mutex:
            return __class__.manager.get_status(self.id)

    def __get_control(self) -> OutputControl:
        with self.__class__.mutex:
            return __class__.manager.get_control(self.id)

    def get_sensor_position(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.sensor_position
        return 0.0

    def get_sensor_velocity(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.sensor_velocity
        return 0.0

    def get_bus_voltage(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.bus_voltage
        return 0.0

    def get_bus_current(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.bus_current
        return 0.0

    def get_stator_current(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.stator_current
        return 0.0

    def get_forward_limit_closed(self) -> bool:
        status = self.__get_status()
        if status is not None:
            return status.forward_limit_closed
        return False

    def get_reverse_limit_closed(self) -> bool:
        status = self.__get_status()
        if status is not None:
            return status.reverse_limit_closed
        return False

    def get_control_mode(self) -> int:
        status = self.__get_status()
        if status is not None:
            return status.control_mode
        return 0

    def get_commanded_output(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.commanded_output
        return 0.0

    def get_active_trajectory_arbff(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.active_trajectory_arbff
        return 0.0

    def get_active_trajectory_position(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.active_trajectory_position
        return 0.0

    def get_active_trajectory_velocity(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.active_trajectory_velocity
        return 0.0

    def get_raw_closed_loop_error(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.raw_closed_loop_error
        return 0.0

    def get_raw_integral_accum(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.raw_integral_accum
        return 0.0

    def get_raw_error_derivative(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.raw_error_derivative
        return 0.0

    def get_raw_output_percent(self) -> float:
        status = self.__get_status()
        if status is not None:
            return status.raw_output_percent
        return 0.0

    def get_faults(self) -> Faults:
        status : Motor_Info = self.__get_status()
        if status is not None:
            return Faults(status.faults)
        return Faults()

    def get_sticky_faults(self) -> StickyFaults:
        status : Motor_Info = self.__get_status()
        if status is not None:
            return StickyFaults(status.sticky_faults)
        return StickyFaults()

    def get_setpoint(self) -> float:
        return self.__get_control().output

    def is_at_setpoint(self, setpoint_delta_threshold : float) -> bool:
        control : OutputControl = self.__get_control()
        status : Motor_Info = self.__get_status()
        if control == None or status == None:
            return False

        ctrl_mode = control.controlMode
        setpoint = control.output

        if ctrl_mode == ControlMode.MOTION_MAGIC or ctrl_mode == ControlMode.POSITION:
            return within(setpoint, status.sensor_position, setpoint_delta_threshold)
        elif ctrl_mode == ControlMode.VELOCITY:
            return within(setpoint, status.sensor_velocity, setpoint_delta_threshold)
        elif ctrl_mode == ControlMode.CURRENT:
            return within(setpoint, status.bus_current, setpoint_delta_threshold)
        else:
            return True


    def __ros_motor_config_validation(self, motor_string):
        config_strings = {
            "_motor_id",
            "_fast_master",
            "_kP",
            "_kI",
            "_kD",
            "_kF",
            "_kP_1",
            "_kI_1",
            "_kD_1",
            "_kF_1",
            "_activeGainSlot",
            "_iZone",
            "_maxIAccum",
            "_allowedClosedLoopError",
            "_maxClosedLoopPeakOutput",
            "_motionCruiseVelocity",
            "_motionCruiseAcceleration",
            "_motionSCurveStrength",
            "_forwardSoftLimit",
            "_forwardSoftLimitEnable",
            "_reverseSoftLimit",
            "_reverseSoftLimitEnable",
            "_feedbackSensorCoefficient",
            "_voltageCompensationSaturation",
            "_voltageCompensationEnabled",
            "_inverted",
            "_sensorPhaseInverted",
            "_neutralModeBrake",
            "_openLoopRamp",
            "_closedLoopRamp",
            "_supplyCurrentLimitEnable",
            "_supplyCurrentLimit",
            "_supplyCurrentLimitThresholdCurrent",
            "_supplyCurrentLimitThresholdTime",
            "_statorCurrentLimitEnable",
            "_statorCurrentLimit",
            "_statorCurrentLimitThresholdCurrent",
            "_statorCurrentLimitThresholdTime",
            "_followingEnabled",
            "_followerId",
            "_forwardLimitSwitchEnabled",
            "_reverseLimitSwitchEnabled",
            "_forwardLimitSwitchSourceType",
            "_reverseLimitSwitchSourceType",
            "_forwardLimitSwitchNormallyClosed",
            "_reverseLimitSwitchNormallyClosed",
            "_peakOutputForward",
            "_peakOutputReverse",
        }

        for config_string in config_strings:
            if not rospy.has_param("/" + rospy.get_name() + "/" + motor_string+config_string):
                raise Exception("Motor: " + motor_string + " is missing config definition for: " + config_string)


    def __load_motor_config(self, motor_string):
        self.__ros_motor_config_validation(motor_string)

        self.config.fast_master = rospy.get_param(rospy.get_name() + "/" + motor_string + "_fast_master")
        self.config.kP = rospy.get_param(rospy.get_name() + "/" + motor_string + "_kP")
        self.config.kI = rospy.get_param(rospy.get_name() + "/" + motor_string + "_kI")
        self.config.kD = rospy.get_param(rospy.get_name() + "/" + motor_string + "_kD")
        self.config.kF = rospy.get_param(rospy.get_name() + "/" + motor_string + "_kF")
        self.config.kP_1 = rospy.get_param(rospy.get_name() + "/" + motor_string + "_kP_1")
        self.config.kI_1 = rospy.get_param(rospy.get_name() + "/" + motor_string + "_kI_1")
        self.config.kD_1 = rospy.get_param(rospy.get_name() + "/" + motor_string + "_kD_1")
        self.config.kF_1 = rospy.get_param(rospy.get_name() + "/" + motor_string + "_kF_1")
        self.config.active_gain_slot = rospy.get_param(rospy.get_name() + "/" + motor_string + "_activeGainSlot")
        self.config.iZone = rospy.get_param(rospy.get_name() + "/" + motor_string + "_iZone")
        self.config.maxIAccum = rospy.get_param(rospy.get_name() + "/" + motor_string + "_maxIAccum")
        self.config.allowedClosedLoopError = rospy.get_param(rospy.get_name() + "/" + motor_string + "_allowedClosedLoopError")
        self.config.maxClosedLoopPeakOutput = rospy.get_param(rospy.get_name() + "/" + motor_string + "_maxClosedLoopPeakOutput")
        self.config.motionCruiseVelocity = rospy.get_param(rospy.get_name() + "/" + motor_string + "_motionCruiseVelocity")
        self.config.motionCruiseAcceleration = rospy.get_param(rospy.get_name() + "/" + motor_string + "_motionCruiseAcceleration")
        self.config.motionSCurveStrength = rospy.get_param(rospy.get_name() + "/" + motor_string + "_motionSCurveStrength")
        self.config.forwardSoftLimit = rospy.get_param(rospy.get_name() + "/" + motor_string + "_forwardSoftLimit")
        self.config.forwardSoftLimitEnable = rospy.get_param(rospy.get_name() + "/" + motor_string + "_forwardSoftLimitEnable")
        self.config.reverseSoftLimit = rospy.get_param(rospy.get_name() + "/" + motor_string + "_reverseSoftLimit")
        self.config.reverseSoftLimitEnable = rospy.get_param(rospy.get_name() + "/" + motor_string + "_reverseSoftLimitEnable")
        self.config.feedbackSensorCoefficient = rospy.get_param(rospy.get_name() + "/" + motor_string + "_feedbackSensorCoefficient")
        self.config.voltageCompensationSaturation = rospy.get_param(rospy.get_name() + "/" + motor_string + "_voltageCompensationSaturation")
        self.config.voltageCompensationEnabled = rospy.get_param(rospy.get_name() + "/" + motor_string + "_voltageCompensationEnabled")
        self.config.inverted = rospy.get_param(rospy.get_name() + "/" + motor_string + "_inverted")
        self.config.sensorPhaseInverted = rospy.get_param(rospy.get_name() + "/" + motor_string + "_sensorPhaseInverted")
        self.config.neutralMode = NeutralMode.Coast
        if rospy.get_param(rospy.get_name() + "/" + motor_string + "_neutralModeBrake") is True:
            self.config.neutralMode = NeutralMode.Brake
        self.config.openLoopRamp = rospy.get_param(rospy.get_name() + "/" + motor_string + "_openLoopRamp")
        self.config.closedLoopRamp = rospy.get_param(rospy.get_name() + "/" + motor_string + "_closedLoopRamp")
        self.config.supplyCurrentLimitEnable = rospy.get_param(rospy.get_name() + "/" + motor_string + "_supplyCurrentLimitEnable")
        self.config.supplyCurrentLimit = rospy.get_param(rospy.get_name() + "/" + motor_string + "_supplyCurrentLimit")
        self.config.supplyCurrentLimitThresholdCurrent = rospy.get_param(rospy.get_name() + "/" + motor_string + "_supplyCurrentLimitThresholdCurrent")
        self.config.supplyCurrentLimitThresholdTime = rospy.get_param(rospy.get_name() + "/" + motor_string + "_supplyCurrentLimitThresholdTime")
        self.config.statorCurrentLimitEnable = rospy.get_param(rospy.get_name() + "/" + motor_string + "_statorCurrentLimitEnable")
        self.config.statorCurrentLimit = rospy.get_param(rospy.get_name() + "/" + motor_string + "_statorCurrentLimit")
        self.config.statorCurrentLimitThresholdCurrent = rospy.get_param(rospy.get_name() + "/" + motor_string + "_statorCurrentLimitThresholdCurrent")
        self.config.statorCurrentLimitThresholdTime = rospy.get_param(rospy.get_name() + "/" + motor_string + "_statorCurrentLimitThresholdTime")
        self.config.followingEnabled = rospy.get_param(rospy.get_name() + "/" + motor_string + "_followingEnabled")
        self.config.followerId = rospy.get_param(rospy.get_name() + "/" + motor_string + "_followerId")
        if rospy.get_param(rospy.get_name() + "/" + motor_string + "_forwardLimitSwitchEnabled") is True:
            self.config.forwardLimitSwitchSource = rospy.get_param(rospy.get_name() + "/" + motor_string + "_forwardLimitSwitchSourceType")
            self.config.forwardLimitSwitchNormal = rospy.get_param(rospy.get_name() + "/" + motor_string + "_forwardLimitSwitchNormallyClosed")
        else:
            self.config.forwardLimitSwitchSource = LimitSwitchSource.Deactivated
            self.config.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled
        if rospy.get_param(rospy.get_name() + "/" + motor_string + "_reverseLimitSwitchEnabled") is True:
            self.config.reverseLimitSwitchSource = rospy.get_param(rospy.get_name() + "/" + motor_string + "_reverseLimitSwitchSourceType")
            self.config.reverseLimitSwitchNormal = rospy.get_param(rospy.get_name() + "/" + motor_string + "_reverseLimitSwitchNormallyOpen")
        else:
            self.config.reverseLimitSwitchSource = LimitSwitchSource.Deactivated
            self.config.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled
        self.config.peakOutputForward = rospy.get_param(rospy.get_name() + "/" + motor_string + "_peakOutputForward")
        self.config.peakOutputReverse = rospy.get_param(rospy.get_name() + "/" + motor_string + "_peakOutputReverse")

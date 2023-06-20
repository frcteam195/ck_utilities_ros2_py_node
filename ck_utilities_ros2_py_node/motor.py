#!/usr/bin/env python3

import rclpy
from dataclasses import dataclass
from threading import Thread, RLock
from ck_ros2_base_msgs_node.msg import Motor_Status
from ck_ros2_base_msgs_node.msg import Motor_Info
from ck_ros2_base_msgs_node.msg import Motor_Control
from ck_ros2_base_msgs_node.msg import Motor_Configuration
from ck_ros2_base_msgs_node.msg import Motor_Config
import ck_ros2_base_msgs_node.msg
from ck_ros2_base_msgs_node.msg import Current_Limit_Configuration
from enum import Enum
from ck_utilities_ros2_py_node.ckmath import within


#TODO: Convert to ROS2

class NeutralMode(Enum):
    Coast = 1
    Brake = 2

class ConfigMode(Enum):
    Master = 0
    FastMaster = 1
    Follower = 2

class InvertType(Enum):
    Nope = 0
    InvertMotorOutput = 1
    FollowMaster = 2
    OpposeMaster = 3

class LimitSwitchSource(Enum):
    FeedbackConnector = 0
    RemoteTalon = 1
    RemoteTalonSRX = 1
    RemoteCANifier = 2
    Deactivated = 3

class LimitSwitchNormal(Enum):
    NormallyOpen = 0
    NormallyClosed = 1
    Disabled = 2

class MotorType(Enum):
    TalonFX = 0
    TalonSRX = 1

class ControlMode(Enum):
    PERCENT_OUTPUT = 0
    POSITION = 1
    VELOCITY = 2
    CURRENT = 3
    FOLLOWER = 5
    MOTION_PROFILE = 6
    MOTION_MAGIC = 7
    MOTION_PROFILE_ARC = 10
    MUSIC_TONE = 13
    DISABLED = 15

@dataclass
class Faults:
    BitField : int = 0
    ###########################
    UnderVoltage : bool = False
    ForwardLimitSwitch : bool = False
    ReverseLimitSwitch : bool = False
    ForwardSoftLimit : bool = False
    ReverseSoftLimit : bool = False
    HardwareFailure : bool = False
    ResetDuringEn : bool = False
    SensorOverflow : bool = False
    SensorOutOfPhase : bool = False
    HardwareESDReset : bool = False
    RemoteLossOfSignal : bool = False
    APIError : bool = False
    SupplyOverV : bool = False
    SupplyUnstable : bool = False
    ############################

    def __post_init__(self, BitField : int = 0):
        self.BitField = BitField
        self.UnderVoltage =       BitField & 0x00000000000001
        self.ForwardLimitSwitch = BitField & 0x00000000000010
        self.ReverseLimitSwitch = BitField & 0x00000000000100
        self.ForwardSoftLimit =   BitField & 0x00000000001000
        self.ReverseSoftLimit =   BitField & 0x00000000010000
        self.HardwareFailure =    BitField & 0x00000000100000
        self.ResetDuringEn =      BitField & 0x00000001000000
        self.SensorOverflow =     BitField & 0x00000010000000
        self.SensorOutOfPhase =   BitField & 0x00000100000000
        self.HardwareESDReset =   BitField & 0x00001000000000
        self.RemoteLossOfSignal = BitField & 0x00010000000000
        self.APIError =           BitField & 0x00100000000000
        self.SupplyOverV =        BitField & 0x01000000000000
        self.SupplyUnstable =     BitField & 0x10000000000000

@dataclass
class StickyFaults:
    BitField : int = 0
    ###########################
    UnderVoltage : bool = False
    ForwardLimitSwitch : bool = False
    ReverseLimitSwitch : bool = False
    ForwardSoftLimit : bool = False
    ReverseSoftLimit : bool = False
    ResetDuringEn : bool = False
    SensorOverflow : bool = False
    SensorOutOfPhase : bool = False
    HardwareESDReset : bool = False
    RemoteLossOfSignal : bool = False
    APIError : bool = False
    SupplyOverV : bool = False
    SupplyUnstable : bool = False
    ############################

    def __post_init__(self, BitField : int = 0):
        self.BitField = BitField
        self.UnderVoltage =       BitField & 0x0000000000001
        self.ForwardLimitSwitch = BitField & 0x0000000000010
        self.ReverseLimitSwitch = BitField & 0x0000000000100
        self.ForwardSoftLimit =   BitField & 0x0000000001000
        self.ReverseSoftLimit =   BitField & 0x0000000010000
        self.ResetDuringEn =      BitField & 0x0000000100000
        self.SensorOverflow =     BitField & 0x0000001000000
        self.SensorOutOfPhase =   BitField & 0x0000010000000
        self.HardwareESDReset =   BitField & 0x0000100000000
        self.RemoteLossOfSignal = BitField & 0x0001000000000
        self.APIError =           BitField & 0x0010000000000
        self.SupplyOverV =        BitField & 0x0100000000000
        self.SupplyUnstable =     BitField & 0x1000000000000

@dataclass
class OutputControl:
    type : MotorType = MotorType.TalonFX
    output : float = 0
    arbFF : float = 0
    controlMode : ControlMode = ControlMode.PERCENT_OUTPUT
@dataclass
class MotorConfig:
    type : MotorType = MotorType.TalonFX
    fast_master : bool = False
    kP : float = 0
    kI : float = 0
    kD : float = 0
    kF : float = 0
    kP_1 : float = 0
    kI_1 : float = 0
    kD_1 : float = 0
    kF_1 : float = 0
    active_gain_slot : int = 0
    iZone : float = 0
    maxIAccum : float = 0
    allowedClosedLoopError : float = 0
    maxClosedLoopPeakOutput : float = 0
    motionCruiseVelocity : float = 0
    motionCruiseAcceleration : float = 0
    motionSCurveStrength : float = 0
    forwardSoftLimit : float = 0
    forwardSoftLimitEnable : bool = False
    reverseSoftLimit : float = 0
    reverseSoftLimitEnable : bool = False
    feedbackSensorCoefficient : float = 0
    voltageCompensationSaturation : float = 12
    voltageCompensationEnabled : bool = True
    inverted : bool = False
    sensorPhaseInverted : bool = False
    neutralMode : NeutralMode = NeutralMode.Coast
    openLoopRamp : float = 0
    closedLoopRamp : float = 0
    supplyCurrentLimitEnable : bool = True
    supplyCurrentLimit : float = 40
    supplyCurrentLimitThresholdCurrent : float = 0
    supplyCurrentLimitThresholdTime : float = 0
    statorCurrentLimitEnable : bool = False
    statorCurrentLimit : float = 0
    statorCurrentLimitThresholdCurrent : float = 0
    statorCurrentLimitThresholdTime : float =  0
    followingEnabled : bool = False
    followerId : int = 0
    forwardLimitSwitchSource : LimitSwitchSource = LimitSwitchSource.Deactivated
    reverseLimitSwitchSource : LimitSwitchSource = LimitSwitchSource.Deactivated
    forwardLimitSwitchNormal : LimitSwitchNormal = LimitSwitchNormal.Disabled
    reverseLimitSwitchNormal : LimitSwitchNormal = LimitSwitchNormal.Disabled
    peakOutputForward : float = 0
    peakOutputReverse : float = 0

class MotorManager:
    def __init__(self):
        self.__motorConfigs = {}
        self.__motorControls = {}
        self.__motorStatuses = {}
        self.__controlPublisher = rospy.Publisher(name='/MotorControl', data_class=Motor_Control, queue_size=50, tcp_nodelay=True)
        self.__configPublisher = rospy.Publisher(name='/MotorConfiguration', data_class=Motor_Configuration, queue_size=50, tcp_nodelay=True)
        self.__mutex = RLock()
        x = Thread(target=self.__motorMasterLoop)
        rospy.Subscriber("/MotorStatus", Motor_Status, self.__receive_motor_status)
        x.start()

    def __receive_motor_status(self, data):
        with self.__mutex:
            for motor in data.motors:
                self.__motorStatuses[motor.id] = motor

    def apply_motor_config(self, motorId : int, motorConfig : MotorConfig):
        with self.__mutex:
            self.__motorConfigs[motorId] = motorConfig

    def update_motor_control(self, motorId : int, outputControl : OutputControl):
        with self.__mutex:
            old_output_control = None
            if motorId in self.__motorControls:
                old_output_control = self.__motorControls[motorId]
            self.__motorControls[motorId] = outputControl
            if old_output_control != self.__motorControls[motorId]:
                self.__set_motor_now(motorId, outputControl)

    def get_status(self, id):
        with self.__mutex:
            if id in self.__motorStatuses:
                return self.__motorStatuses[id]
            return None

    def get_control(self, id):
        with self.__mutex:
            if id in self.__motorControls:
                return self.__motorControls[id]
            return None

    @staticmethod
    def __create_motor_control_dictionary(motorId : int, motorControl : OutputControl):
        motorControlMsg = ck_ros_base_msgs_node.msg.Motor()
        motorControlMsg.id = motorId
        motorControlMsg.controller_type = motorControl.type.value
        motorControlMsg.control_mode = motorControl.controlMode.value
        motorControlMsg.output_value = motorControl.output
        motorControlMsg.arbitrary_feedforward = motorControl.arbFF
        return motorControlMsg

    @staticmethod
    def __create_motor_config_dictionary(motorId : int, motorConfig : MotorConfig):
        motorConfigMsg = Motor_Config()
        motorConfigMsg.id = motorId
        motorConfigMsg.controller_type = motorConfig.type.value
        if motorConfig.followingEnabled:
            motorConfigMsg.controller_mode = ConfigMode.Follower.value
            motorConfigMsg.invert_type = InvertType.OpposeMaster.value if motorConfig.inverted else InvertType.FollowMaster.value
        elif motorConfig.fast_master:
            motorConfigMsg.controller_mode = ConfigMode.FastMaster.value
            motorConfigMsg.invert_type = InvertType.InvertMotorOutput.value if motorConfig.inverted else InvertType.Nope.value
        else:
            motorConfigMsg.controller_mode = ConfigMode.Master.value
            motorConfigMsg.invert_type = InvertType.InvertMotorOutput.value if motorConfig.inverted else InvertType.Nope.value
        motorConfigMsg.kP = motorConfig.kP
        motorConfigMsg.kI = motorConfig.kI
        motorConfigMsg.kD = motorConfig.kD
        motorConfigMsg.kF = motorConfig.kF
        motorConfigMsg.kP_1 = motorConfig.kP_1
        motorConfigMsg.kI_1 = motorConfig.kI_1
        motorConfigMsg.kD_1 = motorConfig.kD_1
        motorConfigMsg.kF_1 = motorConfig.kF_1
        motorConfigMsg.active_gain_slot = motorConfig.active_gain_slot
        motorConfigMsg.iZone = motorConfig.iZone
        motorConfigMsg.max_i_accum = motorConfig.maxIAccum
        motorConfigMsg.allowed_closed_loop_error = motorConfig.allowedClosedLoopError
        motorConfigMsg.max_closed_loop_peak_output = motorConfig.maxClosedLoopPeakOutput
        motorConfigMsg.motion_cruise_velocity = motorConfig.motionCruiseVelocity
        motorConfigMsg.motion_acceleration = motorConfig.motionCruiseAcceleration
        motorConfigMsg.motion_s_curve_strength = motorConfig.motionSCurveStrength
        motorConfigMsg.forward_soft_limit = motorConfig.forwardSoftLimit
        motorConfigMsg.forward_soft_limit_enable = motorConfig.forwardSoftLimitEnable
        motorConfigMsg.reverse_soft_limit = motorConfig.reverseSoftLimit
        motorConfigMsg.reverse_soft_limit_enable = motorConfig.reverseSoftLimitEnable
        motorConfigMsg.feedback_sensor_coefficient = motorConfig.feedbackSensorCoefficient
        motorConfigMsg.voltage_compensation_saturation = motorConfig.voltageCompensationSaturation
        motorConfigMsg.voltage_compensation_enabled = motorConfig.voltageCompensationEnabled
        motorConfigMsg.sensor_phase_inverted = motorConfig.sensorPhaseInverted
        motorConfigMsg.neutral_mode = motorConfig.neutralMode.value
        motorConfigMsg.open_loop_ramp = motorConfig.openLoopRamp
        motorConfigMsg.closed_loop_ramp = motorConfig.closedLoopRamp
        supplyCurrentLimit = Current_Limit_Configuration()
        supplyCurrentLimit.enable = motorConfig.supplyCurrentLimitEnable
        supplyCurrentLimit.current_limit = motorConfig.supplyCurrentLimit
        supplyCurrentLimit.trigger_threshold_current = motorConfig.supplyCurrentLimitThresholdCurrent
        supplyCurrentLimit.trigger_threshold_time = motorConfig.supplyCurrentLimitThresholdTime
        motorConfigMsg.supply_current_limit_config = supplyCurrentLimit
        statorCurrentLimit = Current_Limit_Configuration()
        statorCurrentLimit.enable = motorConfig.statorCurrentLimitEnable
        statorCurrentLimit.current_limit = motorConfig.statorCurrentLimit
        statorCurrentLimit.trigger_threshold_current = motorConfig.statorCurrentLimitThresholdCurrent
        statorCurrentLimit.trigger_threshold_time = motorConfig.statorCurrentLimitThresholdTime
        motorConfigMsg.stator_current_limit_config = statorCurrentLimit
        motorConfigMsg.forward_limit_switch_source = motorConfig.forwardLimitSwitchSource.value
        motorConfigMsg.forward_limit_switch_normal = motorConfig.forwardLimitSwitchNormal.value
        motorConfigMsg.reverse_limit_switch_source = motorConfig.reverseLimitSwitchSource.value
        motorConfigMsg.reverse_limit_switch_normal = motorConfig.reverseLimitSwitchNormal.value
        motorConfigMsg.peak_output_forward = motorConfig.peakOutputForward
        motorConfigMsg.peak_output_reverse = motorConfig.peakOutputReverse
        return motorConfigMsg

    def __transmit_motor_configs(self):
        configMessage = Motor_Configuration()
        configMessage.motors = []
        for motorId in self.__motorConfigs.keys():
            if motorId in self.__motorControls:
                configMessage.motors.append(self.__create_motor_config_dictionary(motorId, self.__motorConfigs[motorId]))
        self.__configPublisher.publish(configMessage)

    def __transmit_motor_controls(self):
        controlMessage = Motor_Control()
        controlMessage.motors = []
        for motorId in self.__motorControls.keys():
            if motorId in self.__motorConfigs:
                if self.__motorConfigs[motorId]:
                    controlStructure = self.__motorControls[motorId]
                controlMessage.motors.append(self.__create_motor_control_dictionary(motorId, controlStructure))
        self.__controlPublisher.publish(controlMessage)

    def __set_motor_now(self, motorId : int, outputControl : OutputControl):
        controlMessage = Motor_Control()
        controlMessage.motors = []
        controlMessage.motors.append(self.__create_motor_control_dictionary(motorId, self.__motorControls[motorId]))
        self.__controlPublisher.publish(controlMessage)

    def __motorMasterLoop(self):
        r = rospy.Rate(10) #10hz
        while not rospy.is_shutdown():
            with self.__mutex:
                self.__transmit_motor_controls()
                self.__transmit_motor_configs()
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

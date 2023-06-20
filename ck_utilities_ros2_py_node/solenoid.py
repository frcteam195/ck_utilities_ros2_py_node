#!/usr/bin/env python3

from typing import List
import rclpy
from dataclasses import dataclass
from threading import Thread, RLock
from ck_ros2_base_msgs_node.msg import Solenoid_Control
from ck_ros2_base_msgs_node.msg import Solenoid as SolenoidROS
from enum import Enum
from typing import TypeVar, Generic

#TODO: Convert to ROS2

ValueClass = TypeVar('ValueClass')
class TimeDelayedValue(Generic[ValueClass]):
    def __init__(self, transition_delay_sec : float, default_value : ValueClass):
        self.__curr_value : ValueClass = default_value
        self.__next_value : ValueClass = default_value
        self.__delay : float = transition_delay_sec
        self.__transition_time = -1

    def set(self, value : ValueClass):
        if self.__next_value != value:
            self.__next_value = value
            self.__transition_time = rospy.get_time()

    def get(self) -> ValueClass:
        if self.__curr_value != self.__next_value:
            if self.__transition_time != -1 and (rospy.get_time() - self.__transition_time) > self.__delay:
                self.__curr_value = self.__next_value
                self.__transition_time = -1
        return self.__curr_value

class SolenoidType(Enum):
    SINGLE = 0
    DOUBLE = 1

class SolenoidState(Enum):
    OFF = 0
    ON = 1
    FORWARD = 1
    REVERSE = 2

class __ModuleType__(Enum):
    CTREPCM = 0
    REVPH = 1

@dataclass
class SolenoidControl:
    id : int = 0
    type : SolenoidType = SolenoidType.SINGLE
    output : SolenoidState = SolenoidState.OFF

class SolenoidManager:
    def __init__(self):
        self.__solenoidControls : dict[int, SolenoidControl] = {}
        self.__controlPublisher = rospy.Publisher(name='SolenoidControl', data_class=Solenoid_Control, queue_size=50, tcp_nodelay=True)
        self.__mutex = RLock()
        x = Thread(target=self.__solenoidMasterLoop)
        x.start()

    @staticmethod
    def __create_solenoid_control_dictionary(solenoidId : int, solenoidControl : SolenoidControl):
        solenoidControlMsg = SolenoidROS()
        solenoidControlMsg.id = solenoidId
        solenoidControlMsg.solenoid_type = solenoidControl.type.value
        solenoidControlMsg.module_type = __ModuleType__.CTREPCM.value
        solenoidControlMsg.output_value = solenoidControl.output.value
        return solenoidControlMsg

    def __transmit_solenoid_controls(self):
        controlMessage = Solenoid_Control()
        controlMessage.solenoids = []
        for solenoid in self.__solenoidControls:
            controlMessage.solenoids.append(self.__create_solenoid_control_dictionary(self.__solenoidControls[solenoid].id, self.__solenoidControls[solenoid]))
        self.__controlPublisher.publish(controlMessage)

    def __set_solenoid_now(self, solenoidId : int, outputControl : SolenoidControl):
        controlMessage = Solenoid_Control()
        controlMessage.solenoids = []
        controlMessage.solenoids.append(self.__create_solenoid_control_dictionary(solenoidId, outputControl))
        self.__controlPublisher.publish(controlMessage)

    def update_solenoid_control(self, solenoidId : int, outputControl : SolenoidControl):
        with self.__mutex:
            self.__solenoidControls[solenoidId] = outputControl
            self.__set_solenoid_now(solenoidId, outputControl)

    def __solenoidMasterLoop(self):
        r = rospy.Rate(10) #10hz
        while not rospy.is_shutdown():
            with self.__mutex:
                self.__transmit_solenoid_controls()
            r.sleep()

class Solenoid:
    manager : SolenoidManager = None
    mutex = RLock()

    def __init__(self, *args):
        self.__solenoidControl : SolenoidControl = SolenoidControl()

        if isinstance(args[0], int) and isinstance(args[1], SolenoidType):
            self.__solenoidControl.id = args[0]
            self.__solenoidControl.type = args[1]
            self.__delay = 0.150
        elif isinstance(args[0], str) and isinstance(args[1], SolenoidType):
            if not rospy.has_param(rospy.get_name() + "/" + args[0] + "_solenoid_id"):
                raise Exception ("Solenoid: " + args[0] + "_id is not set!")
            if not rospy.has_param(rospy.get_name() + "/" + args[0] + "_module_id"):
                raise Exception ("Solenoid: " + args[0] + "_id is not set!")
            self.__solenoidControl.id = rospy.get_param(rospy.get_name() + "/" + args[0] + "_solenoid_id")
            module_id = rospy.get_param(rospy.get_name() + "/" + args[0] + "_module_id")
            self.__solenoidControl.id = self.__solenoidControl.id | ((module_id << 16) & 0xFFFF0000)
            self.__solenoidControl.type = args[1]
            self.__load_solenoid_config(args[0])
        else:
            raise Exception ("Solenoid: " + args[0] + "_id invalid constructor!")

        self.__solenoidControl.output = SolenoidState.OFF
        self.__stored_value = TimeDelayedValue[SolenoidState](self.__delay, SolenoidState.OFF)
        self.__spawn_solenoid_manager()

    def __ros_solenoid_config_validation(self, solenoid_string):
        config_strings = {
            "_solenoid_id",
            "_solenoid_actuation_delay"
        }

        for config_string in config_strings:
            if not rospy.has_param("/" + rospy.get_name() + "/" + solenoid_string+config_string):
                raise Exception("Solenoid: " + solenoid_string + " is missing config definition for: " + config_string)

    def __load_solenoid_config(self, solenoid_string):
        self.__ros_solenoid_config_validation(solenoid_string)
        self.__delay = rospy.get_param(rospy.get_name() + "/" + solenoid_string + "_solenoid_actuation_delay")

    @classmethod
    def __spawn_solenoid_manager(cls):
        with cls.mutex:
            if cls.manager is None:
                cls.manager = SolenoidManager()

    def __update(self):
        with self.__class__.mutex:
            self.__class__.manager.update_solenoid_control(self.__solenoidControl.id, self.__solenoidControl)

    def set(self, output : SolenoidState):
        self.__solenoidControl.output = output
        self.__stored_value.set(output)
        self.__update()
    
    def get(self) -> SolenoidState:
        return self.__stored_value.get()

    def get_id(self) -> int:
        return self.__solenoidControl.id
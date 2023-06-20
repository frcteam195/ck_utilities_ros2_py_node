#!/usr/bin/env python3

from typing import List
import rclpy
from dataclasses import dataclass, field
from threading import Thread, Lock
from ck_ros2_base_msgs_node.msg import LED_Control, LED_Control_Data, LED_Color, LED_Animation, RGBW_Color
from enum import Enum

#TODO: Convert to ROS2

@dataclass
class RGBWColor:
    R : int = 0
    G : int = 0
    B : int = 0
    W : int = 0

@dataclass
class LEDColor:
    rgbw_color : RGBWColor = RGBWColor(0, 0, 0, 0)
    start_index : int = 0
    num_leds : int = 0

class LEDStripType(Enum):
    GRB = 0
    RGB = 1
    BRG = 2
    GRBW = 6
    RGBW = 7
    BRGW = 8

class VBATConfigType(Enum):
    On = 0
    Off = 1
    Modulated = 2

class LEDControlMode(Enum):
    Static = 1
    Animated = 2

class AnimationType(Enum):
    ColorFlow = 0
    Fire = 1
    Larson = 2
    Rainbow = 3
    RGBFade = 4
    SingleFade = 5
    Twinkle = 6
    TwinkleOff = 7
    Strobe = 8
    Morse = 20

class Direction(Enum):
    Forward = 0
    Backward = 1

class CANNetwork(Enum):
    RIO_INTERNAL_CAN = 0
    RIO_CANIVORE = 1
    COPROCESSOR_CAN = 2

@dataclass
class LEDAnimation:
    index : int = 0
    brightness : float = 0
    speed : float = 0
    num_led : int = 0
    color : RGBWColor = RGBWColor(0, 0, 0, 0)
    animation_type : AnimationType = AnimationType.ColorFlow
    direction : Direction = Direction.Forward
    offset : int = 0
    slot : int = 0
    morse_message : str = ""

@dataclass
class LEDControl:
    id : int = 0
    can_network : CANNetwork = CANNetwork.RIO_CANIVORE
    led_type : LEDStripType = LEDStripType.GRB
    vbat_config : VBATConfigType = VBATConfigType.On
    vbat_duty_cycle : float = 0
    led_control_mode : LEDControlMode = LEDControlMode.Static
    color : LEDColor = LEDColor(RGBWColor(0, 0, 0, 0), 0, 0)
    animations : List[LEDAnimation] = field(default_factory=list)

class LEDManager:
    def __init__(self):
        self.__ledControls : dict[int, LEDControl] = {}
        self.__controlPublisher = rospy.Publisher(name='RioLedControl', data_class=LED_Control, queue_size=50, tcp_nodelay=True)
        self.__mutex = Lock()
        x = Thread(target=self.__ledMasterLoop)
        x.start()

    @staticmethod
    def __create_led_control_dictionary(ledId : int, ledControl : LEDControl):
        ledControlMsg = LED_Control_Data()
        ledControlMsg.id = ledId
        ledControlMsg.can_network = ledControl.can_network.value
        ledControlMsg.led_strip_type = ledControl.led_type.value
        ledControlMsg.vbat_config = ledControl.vbat_config.value
        ledControlMsg.vbat_duty_cycle = float(ledControl.vbat_duty_cycle)
        ledControlMsg.led_control_mode = ledControl.led_control_mode.value
        ledControlMsg.color = LED_Color()
        ledControlMsg.color.rgbw_color = RGBW_Color()
        ledControlMsg.color.rgbw_color.R = int(ledControl.color.rgbw_color.R)
        ledControlMsg.color.rgbw_color.G = int(ledControl.color.rgbw_color.G)
        ledControlMsg.color.rgbw_color.B = int(ledControl.color.rgbw_color.B)
        ledControlMsg.color.rgbw_color.W = int(ledControl.color.rgbw_color.W)
        ledControlMsg.color.num_leds = int(ledControl.color.num_leds)
        ledControlMsg.color.start_index = int(ledControl.color.start_index)
        for a in ledControl.animations:
            ros_animation = LED_Animation()
            ros_animation.index = a.index
            ros_animation.brightness = a.brightness
            ros_animation.speed = a.speed
            ros_animation.num_led = a.num_led
            ros_animation.color = RGBW_Color()
            ros_animation.color.R = a.color.R
            ros_animation.color.G = a.color.G
            ros_animation.color.B = a.color.B
            ros_animation.color.W = a.color.W
            ros_animation.animation_type = a.animation_type.value
            ros_animation.direction = a.direction.value
            ros_animation.offset = a.offset
            ros_animation.slot = a.slot
            ledControlMsg.animations.append(ros_animation)
        return ledControlMsg

    def __transmit_led_controls(self):
        controlMessage = LED_Control()
        controlMessage.led_control = []
        for led in self.__ledControls:
            controlMessage.led_control.append(self.__create_led_control_dictionary(self.__ledControls[led].id, self.__ledControls[led]))
        self.__controlPublisher.publish(controlMessage)

    def __set_led_now(self, ledId : int, outputControl : LEDControl):
        controlMessage = LED_Control()
        controlMessage.led_control = []
        controlMessage.led_control.append(self.__create_led_control_dictionary(ledId, outputControl))
        self.__controlPublisher.publish(controlMessage)

    def update_led_control(self, ledId : int, outputControl : LEDControl):
        with self.__mutex:
            self.__ledControls[ledId] = outputControl
            self.__set_led_now(ledId, outputControl)

    def __ledMasterLoop(self):
        r = rospy.Rate(10) #10hz
        while not rospy.is_shutdown():
            with self.__mutex:
                self.__transmit_led_controls()
            r.sleep()

class LEDStrip:
    manager : LEDManager = None
    mutex = Lock()

    def __init__(self, id : int, type : LEDStripType):
        self.__ledControl : LEDControl = LEDControl()
        self.__ledControl.id = id
        self.__ledControl.led_type = type
        self.spawn_led_manager()

    @classmethod
    def spawn_led_manager(cls):
        with cls.mutex:
            if cls.manager is None:
                cls.manager = LEDManager()

    def __update(self):
        with self.__class__.mutex:
            self.__class__.manager.update_led_control(self.__ledControl.id, self.__ledControl)

    def configVoltageOutputMode(self, mode : VBATConfigType):
        self.__ledControl.vbat_config = mode
        self.__update()

    def configVoltageModulationDutyCycle(self, duty_cycle : float):
        self.__ledControl.vbat_duty_cycle = duty_cycle
        self.__update()

    def setLEDControlMode(self, mode : LEDControlMode):
        self.__ledControl.led_control_mode = mode
        self.__update()

    def setLEDColor(self, color : LEDColor):
        self.__ledControl.color = color
        self.__update()

    def setLEDAnimations(self, animations : List[LEDAnimation]):
        self.__ledControl.animations = animations
        self.__update()
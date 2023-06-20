from abc import ABC, abstractmethod
from queue import Queue
from enum import Enum
from ck_ros2_base_msgs_node.msg import StateMachineLog
import rclpy

#TODO: Convert to ROS2

class StateMachine(ABC):

    class State(ABC):

        @abstractmethod
        def get_enum(self):
            pass

        @abstractmethod
        def entry(self):
            pass

        @abstractmethod
        def step(self):
            pass

        @abstractmethod
        def transition(self) -> Enum:
            pass

        def end(self):
            pass

        def get_name(self):
            return self.__name

    def __init__(self, states, state):
        self.state = state
        self.states = states

        self.states[self.state].entry()
        self.transition_history = Queue(10)

        self.log_count = 0
        self.transition_history.put(str(self.log_count) + ": Init: " + str(self.state))
        self.log_count += 1
        self.log_publisher = rospy.Publisher(name="/state_machines/" + str(self.__class__.__name__),
                                             data_class=StateMachineLog,
                                             queue_size=100,
                                             tcp_nodelay=False)

    def get_current_state(self) -> str:
        return self.state

    def log_data(self):
        # log_string = ""
        # for i in self.transition_history.queue:
        #     log_string += i + "\n"
        log_msg = StateMachineLog()
        log_msg.current_state = str(self.state)
        log_msg.current_state_int = self.state.value
        # log_msg.transition_history = log_string
        # improve efficiency of logging
        log_msg.transition_history = '\n'.join(self.transition_history.queue)
        self.log_publisher.publish(log_msg)

    def step(self):
        initial_state = self.state
        self.states[self.state].step()
        self.state = self.states[self.state].transition()

        ## This is the no super step implementation
        if self.state is not initial_state:
            self.states[initial_state].end()
            self.states[self.state].entry()
            if self.transition_history.full():
                self.transition_history.get()
            self.transition_history.put(f"{str(self.log_count)}: TRANSITION: {str(initial_state)} -> {str(self.state)}")
            self.log_count += 1

        ## This is the super step implementation
        # while self.state is not initial_state:
        #     if self.transition_history.full():
        #         self.transition_history.get()
        #     self.transition_history.put(str(self.log_count) + ": TRANSITION: " + str(initial_state) + " -> " + str(self.state))
        #     self.log_count += 1
        #     self.states[initial_state].end()
        #     self.states[self.state].entry()
        #     initial_state = self.state
        #     self.states[self.state].step()
        #     self.state = self.states[self.state].transition()

        # And this is regular
        self.log_data()

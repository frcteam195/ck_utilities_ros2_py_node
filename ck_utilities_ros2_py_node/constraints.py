from typing import List, Tuple
from enum import Enum

class LimitSystem(Enum):
    A = 0,
    B = 1,
    Both = 2

class KeepoutRange:
    def __init__(self, mechanism_a_lower_limit : float, mechanism_a_upper_limit : float,
                 mechanism_a_limit_value_if_constrained : float,
                 mechanism_b_lower_limit : float, mechanism_b_upper_limit : float,
                 mechanism_b_limit_value_if_constrained : float,
                 system_to_limit : LimitSystem) -> None:
        
        self.mechanism_a_lower_limit : float = mechanism_a_lower_limit
        self.mechanism_a_upper_limit : float = mechanism_a_upper_limit
        self.mechanism_b_lower_limit : float = mechanism_b_lower_limit
        self.mechanism_b_upper_limit : float = mechanism_b_upper_limit
        self.mechanism_a_limit_value_if_constrained : float = mechanism_a_limit_value_if_constrained
        self.mechanism_b_limit_value_if_constrained : float = mechanism_b_limit_value_if_constrained
        self.system_to_limit : LimitSystem = system_to_limit

    def is_conflict_present(self, mechanism_a_value : float, mechanism_b_value : float) -> bool:
        if self.mechanism_a_lower_limit <= mechanism_a_value <= self.mechanism_a_upper_limit and \
        self.mechanism_b_lower_limit <= mechanism_b_value <= self.mechanism_b_upper_limit:
            return True
        else:
            return False
        
    def perform_constraint(self, mechanism_a_value : float, mechanism_b_value : float,
                           mechanism_a_setpoint : float, mechanism_b_setpoint : float) -> Tuple[float, float]:
        if self.is_conflict_present(mechanism_a_value, mechanism_b_value):
            if self.system_to_limit == LimitSystem.Both:
                return [self.mechanism_a_limit_value_if_constrained, self.mechanism_b_limit_value_if_constrained]
            elif self.system_to_limit == LimitSystem.A:
                return [self.mechanism_a_limit_value_if_constrained, mechanism_b_setpoint]
            elif self.system_to_limit == LimitSystem.B:
                return [mechanism_a_setpoint, self.mechanism_b_limit_value_if_constrained]
        else:
            return [mechanism_a_setpoint, mechanism_b_setpoint]

class ConstrainedComponent:
    def __init__(self, constraint_list : List[KeepoutRange]):
        self.__constraint_list = constraint_list

    def get_output_value(self, mechanism_a_value : float, mechanism_b_value : float,
                         mechanism_a_setpoint : float, mechanism_b_setpoint : float) -> Tuple[float, float]:
        num_active_constraints : int = 0
        retval : Tuple[float, float] = None
        for c in self.__constraint_list:
            if c.is_conflict_present():
                num_active_constraints += 1
                retval = c.perform_constraint(mechanism_a_value, mechanism_b_value, mechanism_a_setpoint, mechanism_b_setpoint)

        if num_active_constraints > 1:
            # rospy.logerr("More than one constraint is currently active. Please double check the constraint ranges")
            return None
        
        return retval
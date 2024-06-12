#!/usr/bin/env python

import rospy
from smach import State, StateMachine

class BaseState(State):
    def __init__(self, robot, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self.robot = robot

class Start(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
    def execute(self, userdata):
        self.robot.start()
        return 'success'

class Takeoff(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
    def execute(self, userdata):
        self.robot.takeoff()
        return 'success'

class Land(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
    def execute(self, userdata):
        self.robot.land()
        return 'success'

class ForceLanding(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
    def execute(self, userdata):
        self.robot.forceLanding()
        return 'success'

class Halt(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
    def execute(self, userdata):
        self.robot.halt()
        return 'success'

class HoveringCheck(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
    def execute(self, userdata):
        while not (self.robot.getFlightState() == self.robot.HOVER_STATE):
            if self.robot.getForceSkipFlag():
                self.robot.setForceSkipFlag(False)
                return 'success'
            rospy.logwarn_throttle(3.0, "[HoveringCheck] waiting for hovering")
            continue
        return 'success'

class LandingCheck(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
    def execute(self, userdata):
        while not (self.robot.getFlightState() == self.robot.ARM_OFF_STATE):
            if self.robot.getForceSkipFlag():
                self.robot.setForceSkipFlag(False)
                return 'success'
            rospy.logwarn_throttle(3.0, "[LandingCheck] waiting for landing")
            continue
        return 'success'

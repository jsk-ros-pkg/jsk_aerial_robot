#!/usr/bin/env python

from delta.delta_interface import DeltaInterface
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from smach import State, StateMachine
import smach_ros
import tf
import time

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

class Wait(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
        time.sleep(1.0)

    def execute(self, userdata):
        time.sleep(2.0)
        return 'success'

class HoveringCheck(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
    def execute(self, userdata):
        while not (self.robot.getFlightState() == self.robot.HOVER_STATE):
            rospy.logwarn_throttle(3.0, "[HoveringCheck] waiting for hovering")
            continue
        return 'success'

class Approach(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
        self.approach_cog_pos = rospy.get_param("~approach_cog_pos")
        self.approach_wait_time = rospy.get_param("~approach_wait_time")
        time.sleep(1.0)

    def execute(self, userdata):
        # set target pos
        self.robot.setTargetPos(self.approach_cog_pos[0], self.approach_cog_pos[1], self.approach_cog_pos[2])
        rospy.logwarn("[Approach] go to {}, {}, {}".format(self.approach_cog_pos[0], self.approach_cog_pos[1], self.approach_cog_pos[2]))
        start_time = rospy.get_time()
        while(rospy.get_time() - start_time < self.approach_wait_time):
            rospy.logwarn_throttle(3.0, "[Approach] waiting after pos control command")
            pass
        converge_flag = False
        while not converge_flag:
            cog_odom = self.robot.getCogOdom()
            if self.robot.posControlConverged(self.approach_cog_pos, 0.1):
                converge_flag = True
            else:
                pos_control_error = self.robot.getPosControlError(self.approach_cog_pos)
                rospy.logwarn_throttle(3.0, "[Approach] waiting for pos control convergence. error: {} {} {}".format(pos_control_error[0], pos_control_error[1], pos_control_error[2]))

        # set target yaw angle
        self.robot.setTargetYaw(-1.57)
        rospy.logwarn("[Approach] set target yaw angle")
        start_time = rospy.get_time()
        while(rospy.get_time() - start_time < self.approach_wait_time):
            pass
        while not self.robot.yawControlConverged():
            pass

        return 'success'

class PrepareManip(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
        self.prepare_manip_joint_angle = rospy.get_param("~prepare_manip_joint_angle")
        self.prepare_manip_cog_pos = rospy.get_param("~prepare_manip_cog_pos")
        self.prepare_manip_wait_time = rospy.get_param("~prepare_manip_wait_time")

    def execute(self, userdata):
        # set joint angle
        msg = JointState()
        msg.name = ["joint1", "joint2"]
        msg.position = self.prepare_manip_joint_angle
        self.robot.sendJointState(msg)
        rospy.logwarn("[PrepareManip] set joint angle")
        start_time= rospy.get_time()
        while(rospy.get_time() - start_time < self.prepare_manip_wait_time):
            pass

        # set target pos
        self.robot.setTargetPos(self.prepare_manip_cog_pos[0], self.prepare_manip_cog_pos[1], self.prepare_manip_cog_pos[2])
        start_time= rospy.get_time()
        while(rospy.get_time() - start_time < self.prepare_manip_wait_time):
            pass
        while not self.robot.posControlConverged(self.prepare_manip_cog_pos):
            rospy.logwarn_throttle(3.0, "[PrepareManip] waiting for pos control converged")
            pass

        return 'success'

class Manip(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success', 'recover'])
        self.manip_target_pos = rospy.get_param("~manip_target_pos")
        self.manip_wait_time = rospy.get_param("~manip_wait_time")

    def execute(self, userdata):
        self.robot.setTargetPos(self.manip_target_pos[0], self.manip_target_pos[1], self.manip_target_pos[2])
        start_time = rospy.get_time()
        while(rospy.get_time() - start_time < self.manip_wait_time):
            if not self.robot.posControlConverged(0.3):
                return "recover"

        if not self.robot.posControlConverged():
            return "recover"

        return 'success'

class FinishTask(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
        self.finish_task_target_pos = rospy.get_param('~finish_task_target_pos')
        self.finish_task_wait_time = rospy.get_param('~finish_task_wait_time')

    def execute(self, userdata):
        self.robot.setTargetPos(self.finish_task_target_pos[0], self.finish_task_target_pos[1], self.finish_task_target_pos[2])

        start_time = rospy.get_time()
        while(rospy.get_time() - start_time < self.finish_task_wait_time):
            pass

        return "success"

if __name__ == "__main__":
    rospy.init_node("door_opening_demo")
    robot = DeltaInterface()

    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('WAIT_HOVERING', Wait(robot), transitions={'success':'HOVERING'})

        sm_hover = StateMachine(outcomes=['success', ])
        with sm_hover:
            StateMachine.add('START', Start(robot), transitions={'success':'TAKEOFF'})
            StateMachine.add('TAKEOFF', Takeoff(robot), transitions={'success':'HOVERING_CHECK'})
            StateMachine.add('HOVERING_CHECK', HoveringCheck(robot), transitions={'success':'success'})
        StateMachine.add('HOVERING', sm_hover, transitions={'success':'WAIT_TASK'})

        StateMachine.add('WAIT_TASK', Wait(robot), transitions={'success':'APPROACH'})
        StateMachine.add('APPROACH', Approach(robot), transitions={'success':'PREPAREMANIP'})
        StateMachine.add('PREPAREMANIP', PrepareManip(robot), transitions={'success':'MANIP'})
        StateMachine.add('MANIP', Manip(robot), transitions={'success':'FINISHTASK', 'recover':'PREPAREMANIP'})
        StateMachine.add('FINISHTASK', FinishTask(robot), transitions={'success':'LAND'})
        StateMachine.add('LAND', Land(robot), transitions={'success':'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

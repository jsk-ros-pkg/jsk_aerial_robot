#!/usr/bin/env python

from delta.delta_interface import DeltaInterface
from delta.common_states import *
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from smach import State, StateMachine
import smach_ros
from tf.transformations import *
import time

class Pick(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
        self.pos_control_converge_timeout = rospy.get_param("~pos_control_converge_timeout")
        self.joint_angle_wait_time = rospy.get_param("~joint_angle_wait_time")
        self.pick_link1_pos = np.array(rospy.get_param("~pick_link1_pos"))
        self.pick_cog_pos = [0, 0, 0]
        self.pick_cog_yaw_angle = rospy.get_param("~pick_cog_yaw_angle")
        self.pick_joint_angle = rospy.get_param("~pick_joint_angle")
        time.sleep(1.0)

    def execute(self, userdata):
        # set target yaw angle
        rospy.logwarn("[Pick] set target yaw angle")
        self.robot.setTargetYawTraj(self.pick_cog_yaw_angle)
        start_time = rospy.get_time()
        while not self.robot.yawControlConverged(target=self.pick_cog_yaw_angle, thresh=0.05):
            if rospy.get_time() - start_time > self.pos_control_converge_timeout:
                break
            rospy.logwarn_throttle(3.0, "[Pick] waiting for yaw control convergence")

        # set target position
        (trans, rot) = self.robot.getTransform("cog", "link1")
        world_R_cog = self.robot.getCogRotationMatrix()
        self.pick_cog_pos = self.pick_link1_pos - np.dot(world_R_cog, trans)
        print(self.pick_cog_pos)
        rospy.logwarn("[Pick] set target pos {}".format(self.pick_cog_pos))
        self.robot.setTargetPosTraj(self.pick_cog_pos)

        start_time = rospy.get_time()
        while not self.robot.posControlConverged(self.pick_cog_pos, 0.1):
            if rospy.get_time() - start_time > self.pos_control_converge_timeout:
                break
            rospy.logwarn_throttle(3.0, "[Pick] waiting for pos control convergence. error: {}".format(self.robot.getPosControlError(self.pick_cog_pos)))

        # set target joint state
        msg = JointState()
        msg.name = ["joint1", "joint2"]
        msg.position = self.pick_joint_angle
        self.robot.sendJointState(msg)

        time.sleep(self.joint_angle_wait_time)

        return 'success'

class Grasp(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
        self.joint_angle_wait_time = rospy.get_param("~joint_angle_wait_time")
        self.grasp_joint_angle = rospy.get_param('~grasp_joint_angle')

    def execute(self, userdata):
        msg = JointState()
        msg.name = ["joint1", "joint2"]
        msg.position = self.grasp_joint_angle
        self.robot.sendJointState(msg)

        time.sleep(self.joint_angle_wait_time)

        # TODO: add extra module

        return 'success'

class Release(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes={'success'})
        self.pos_control_converge_timeout = rospy.get_param("~pos_control_converge_timeout")
        self.joint_angle_wait_time = rospy.get_param('~joint_angle_wait_time')
        self.release_cog_pos = rospy.get_param('~release_cog_pos')
        self.release_joint_angle = rospy.get_param('~release_joint_angle')

    def execute(self, userdata):
        rospy.logwarn("[Release] set target pos {}".format(self.release_cog_pos))
        self.robot.setTargetPosTraj(self.release_cog_pos)
        start_time = rospy.get_time()
        while not self.robot.posControlConverged(self.release_cog_pos, 0.1):
            if rospy.get_time() - start_time > self.pos_control_converge_timeout:
                break
            rospy.logwarn_throttle(3.0, "[Release] waiting for pos control convergence.")
        return 'success'


if __name__ == "__main__":
    rospy.init_node("brick_pick_demo")
    robot = DeltaInterface()

    sm = StateMachine(outcomes=['success'])

    with sm:
        sm_hover = StateMachine(outcomes=['success'])
        with sm_hover:
            StateMachine.add('START', Start(robot), transitions={'success':'TAKEOFF'})
            StateMachine.add('TAKEOFF', Takeoff(robot), transitions={'success':'HOVERING_CHECK'})
            StateMachine.add('HOVERING_CHECK', HoveringCheck(robot), transitions={'success':'success'})
        StateMachine.add('HOVERING', sm_hover, transitions={'success':'PICK'})
        StateMachine.add('PICK', Pick(robot), transitions={'success':'TMP_LAND'})
        StateMachine.add('TMP_LAND', Land(robot), transitions={'success':'LANDINGCHECK'})
        StateMachine.add('LANDINGCHECK', LandingCheck(robot), transitions={'success':'GRASP'})
        StateMachine.add('GRASP', Grasp(robot), transitions={'success':'GRASP_HOVER'})
        StateMachine.add('GRASP_HOVER', sm_hover, transitions={'success':'RELEASE'})
        StateMachine.add('RELEASE', Release(robot), transitions={'success':'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

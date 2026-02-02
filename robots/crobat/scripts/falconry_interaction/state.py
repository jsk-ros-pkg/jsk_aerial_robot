#!/usr/bin/env python3

import rospy
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
import math
from robot import Flapper
from variables import *
from util import dist
import traceback

class Start(State):
        def __init__(self):
                State.__init__(self, outcomes = ['success', 'failure'])
                self.should_start = False
                self.start_sub = rospy.Subscriber('/task_start', Empty, self.start_sub_callback)

        def start_sub_callback(self, _):
                self.should_start = True
                
        def execute(self, userdata):
                while not self.should_start:
                        rospy.sleep(0.1)
                        rospy.logdebug_throttle(1.0, "waiting to start")
                        if rospy.is_shutdown():
                                print('shutdown ros')
                                return 'failure'
                return "success"

class SingleCommandState(State):
        def __init__(self, robot, func, start_flight_state, goal_flight_state, timeout = 60):
                State.__init__(self, outcomes = ['success', 'failure'])
                self.robot = robot
                self.func = func
                self.start_flight_state = start_flight_state
                self.goal_flight_state = goal_flight_state
                self.timeout = timeout
        def execute(self, userdata):
                if self.robot.state != self.start_flight_state:
                        rospy.logwarn(self.__class__.__name__ + ': the robot state ({}) is not at the start_flight_state ({}).'.format(self.robot.state, self.start_flight_state))
                        return 'failure'
                self.func()
                self.start_t = rospy.get_time()
                while rospy.get_time() < self.start_t + self.timeout:
                        if self.robot.state == self.goal_flight_state:
                                return "success"
                        rospy.sleep(0.1)
                        rospy.logdebug_throttle(1.0, "current state: ({})".format(self.robot.state))
                        if rospy.is_shutdown():
                                print("shutdown ros")
                                return 'failure'
                rospy.logwarn('timeout')
                rospy.logwarn(self.__class__.__name__ + ': the robot state ({}) is not at the goal_flight_state ({}).'.format(self.robot.state, self.goal_flight_state))
                return "failure"

class Takeoff(SingleCommandState):
        # 離陸
        def __init__(self, robot, func):   
                SingleCommandState.__init__(self, robot, func, RobotState.START, RobotState.HOVER)

        def execute(self, userdata):
                if self.robot.state == RobotState.HOVER:
                        rospy.loginfo("The robot is already hovering.")
                        return 'success'
                return SingleCommandState.execute(self, userdata)

class Land(SingleCommandState):
        def __init__(self, robot, func):
                SingleCommandState.__init__(self, robot, func, RobotState.HOVER, RobotState.STOP)
                
class PalmLand(SingleCommandState):
        def __init__(self, robot, func):
                SingleCommandState.__init__(self, robot, func, RobotState.PALM_LAND_READY, RobotState.STOP)
                        
class Approach(State):
        def __init__(self, robot, timeout=1000, safety_radius = 0.30, threshold = 0.1):
                State.__init__(self, outcomes=['stay', 'palm_land', 'failure'])
                self.robot = robot
                self.approach_start_pub = rospy.Publisher('approach_start', Empty)
                self.approach_stop_pub = rospy.Publisher('approach_stop', Empty)
                self.chest_pose_sub = rospy.Subscriber('mocap_node/mocap/chest/pose',
                                                       PoseStamped,
                                                       self.chest_pose_sub_callback)
                self.hand_pose_sub = rospy.Subscriber('mocap_node/mocap/hand/pose',
                                                      PoseStamped,
                                                      self.hand_pose_sub_callback)
                self.drone_pose_sub = rospy.Subscriber('/crobat/mocap/pose',
                                                       PoseStamped,
                                                       self.drone_pose_sub_callback)        
                self.timeout = timeout
                self.safety_radius = safety_radius
                self.threshold = threshold
        def chest_pose_sub_callback(self, msg):
                self.chest_pose = msg.pose
        
        def hand_pose_sub_callback(self, msg):
                self.hand_pose = msg.pose

        def drone_pose_sub_callback(self, msg):
                self.drone_pose = msg.pose
                
        def execute(self, userdata):
                self.approach_start_pub.publish()
                start_t = rospy.get_time()
                while rospy.get_time() < start_t + self.timeout:
                        if rospy.is_shutdown():
                                return 'failure'
                        chest_pos = self.chest_pose.position
                        hand_pos = self.hand_pose.position
                        drone_pos = self.drone_pose.position
                        distance_hand_drone_on_XY_plane = dist(hand_pos, drone_pos, True)
                        distance_chest_hand = dist(chest_pos, hand_pos)
                        rospy.loginfo(f'distance_chest_hand: {distance_chest_hand}')
                        if distance_chest_hand < self.safety_radius:
                                self.approach_stop_pub.publish()
                                return 'stay'
                        if distance_hand_drone_on_XY_plane < self.threshold:
                                self.approach_stop_pub.publish()
                                return 'palm_land'
                        rospy.sleep(0.1)
                self.approach_stop_pub.publish()
                return 'failure'


class Stay(State):
        def __init__(self, timeout=120, safety_radius=0.30):
                State.__init__(self, outcomes=['approach', 'failure'])
                self.chest_pose_sub = rospy.Subscriber('mocap_node/mocap/chest/pose',
                                                       PoseStamped,
                                                       self.chest_pose_sub_callback)
                self.hand_pose_sub = rospy.Subscriber('mocap_node/mocap/hand/pose',
                                                      PoseStamped,
                                                      self.hand_pose_sub_callback)       
                self.timeout = timeout
                self.safety_radius = safety_radius

        def chest_pose_sub_callback(self, msg):
                self.chest_pose = msg.pose
        
        def hand_pose_sub_callback(self, msg):
                self.hand_pose = msg.pose
                
        def execute(self, userdata):
                start_t = rospy.get_time()
                while rospy.get_time() < start_t + self.timeout:
                        rospy.sleep(0.1)
                        if rospy.is_shutdown():
                                return 'failure'
                        if not hasattr(self, 'chest_pose'):
                                rospy.logwarn('Unable to get chest_pose. Check mocap settings.')
                                continue
                        if not hasattr(self, 'hand_pose'):
                                rospy.logwarn('Unable to get chest_pose. Check mocap settings.')
                                continue
                        chest_pos = self.chest_pose.position
                        hand_pos = self.hand_pose.position
                        distance_chest_hand = dist(chest_pos, hand_pos)
                        rospy.loginfo(f'distance_chest_hand: {distance_chest_hand}')
                        if distance_chest_hand > self.safety_radius:
                                return 'approach'
                return 'failure'

class PalmLand(State):
        def __init__(self, robot, timeout=10):
                State.__init__(self, outcomes=['success', 'failure'])
                self.robot = robot
                self.timeout = timeout
                self.drone_pose_sub = rospy.Subscriber('crobat/mocap/pose',
                                                       PoseStamped,
                                                       self.drone_pose_sub_callback)        

        def execute(self, userdata):
                self.robot.palm_land(self.drone_pose)
                start_t = rospy.get_time()
                while rospy.get_time() < start_t + self.timeout:
                        if rospy.is_shutdown():
                                return 'failure'
                        if self.robot.state == RobotState.STOP:
                                return 'success'
                return 'failure'

        def drone_pose_sub_callback(self, msg):
                self.drone_pose = msg.pose
       

def main():
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['success', 'failure'])
    flapper = Flapper()
    
    # Open the container
    with sm:
        # Add states to the container
        StateMachine.add('Start', Start(), 
                         transitions={'success': 'Takeoff', 'failure': 'failure'})
        StateMachine.add('Takeoff', Takeoff(flapper, flapper.takeoff), 
                         transitions={'success':'Stay', 'failure': 'failure'})
        StateMachine.add('Approach', Approach(flapper),
                         transitions={'palm_land': 'PalmLand', 'stay': 'Stay','failure': 'failure'})
        StateMachine.add('Stay', Stay(),
                         transitions={'approach': 'Approach', 'failure': 'failure'})
        StateMachine.add('PalmLand', PalmLand(flapper),
                         transitions={'success': 'success', 'failure': 'failure'})

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    if outcome == 'failure':
            flapper.land()
    rospy.spin()    
    sis.stop()


if __name__ == '__main__':
        try:
                main()
        except Exception as e:
                print(traceback.format_exc())

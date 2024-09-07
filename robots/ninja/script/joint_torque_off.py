#!/usr/bin/env python

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

def switch_controller():
    for i in range(3):
        id = i+1
        rospy.wait_for_service('/ninja'+str(id)+'/controller_manager/switch_controller')
        try:
            switch_controller_service = rospy.ServiceProxy('/ninja'+str(id)+'/controller_manager/switch_controller', SwitchController)
        
            req = SwitchControllerRequest()
            if(id is 1):
                req.start_controllers = ['/ninja'+str(id)+'/servo_controller/joints/controller1_2/simulation']
                req.stop_controllers = ['/ninja'+str(id)+'/servo_controller/joints/controller1_1/simulation']
            elif(id is 3):
                req.start_controllers = ['/ninja'+str(id)+'/servo_controller/joints/controller2_2/simulation']
                req.stop_controllers = ['/ninja'+str(id)+'/servo_controller/joints/controller2_1/simulation']
            else:
                req.start_controllers = ['/ninja'+str(id)+'/servo_controller/joints/controller2_2/simulation','/ninja'+str(id)+'/servo_controller/joints/controller1_2/simulation']
                req.stop_controllers = ['/ninja'+str(id)+'/servo_controller/joints/controller2_1/simulation','/ninja'+str(id)+'/servo_controller/joints/controller1_1/simulation']                
            req.strictness = SwitchControllerRequest.STRICT
            
            res = switch_controller_service(req)
        
            if res.ok:
                rospy.loginfo("Successfully switched controllers")
            else:
                rospy.logwarn("Failed to switch controllers")
            
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('controller_switcher')

    # Example: Switch from position controller to effort controller
    switch_controller()

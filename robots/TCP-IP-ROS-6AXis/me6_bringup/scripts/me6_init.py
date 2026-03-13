#!/usr/bin/env python3
import rospy
from dobot_v4_bringup.srv import ClearError, EnableRobot, SetCollisionLevel

class DobotInitializer:
    def __init__(self):
        rospy.init_node("dobot_initializer")

        rospy.loginfo("Waiting for Dobot initialization services...")

        rospy.wait_for_service("/dobot_v4_bringup/srv/ClearError")
        rospy.wait_for_service("/dobot_v4_bringup/srv/EnableRobot")
        rospy.wait_for_service("/dobot_v4_bringup/srv/SetCollisionLevel")

        self.clear_error_srv = rospy.ServiceProxy("/dobot_v4_bringup/srv/ClearError", ClearError)
        self.enable_robot_srv = rospy.ServiceProxy("/dobot_v4_bringup/srv/EnableRobot", EnableRobot)
        self.set_collision_srv = rospy.ServiceProxy("/dobot_v4_bringup/srv/SetCollisionLevel", SetCollisionLevel)

        self.initialize_robot()

    def initialize_robot(self):
        rospy.loginfo("Initializing Dobot robot...")

        try:
            res_clear = self.clear_error_srv()
            if res_clear.res != 0:
                rospy.logerr(f"ClearError failed with code {res_clear.res}")
                return
            rospy.loginfo("ClearError succeeded")
            rospy.sleep(1.0)

            res_enable = self.enable_robot_srv()
            if res_enable.res != 0:
                rospy.logerr(f"EnableRobot failed with code {res_enable.res}")
                return
            rospy.loginfo("EnableRobot succeeded")
            rospy.sleep(1.0)

            res_collision = self.set_collision_srv(level=0)
            rospy.loginfo("SetCollisionLevel succeeded (level=0)")

            rospy.loginfo("Dobot robot initialized successfully!")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    DobotInitializer()

#!/usr/bin/env python
#!/usr/bin/env python3

import rospy
import roslaunch
import os
import signal
import sys
import logging

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)
class RemoteBringup:
    def __init__(self, bringup_launch_path):
        self.bringup_launch_path = bringup_launch_path
        self.launch = None

    def start_bringup(self):
        print("Starting bringup.launch...", flush=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s', handlers=[logging.StreamHandler(sys.stdout)])
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [self.bringup_launch_path])
        rospy.loginfo("Starting bringup.launch...")
        self.launch.start()

    def shutdown(self):
        if self.launch:
            rospy.loginfo("Shutting down bringup.launch...")
            self.launch.shutdown()

def main():
    rospy.init_node("bringup_node", anonymous=True)
    # path to bringup launch
    bringup_launch_path = "/home/leus/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/ninja/launch/bringup.launch"

    bringup_handler = RemoteBringup(bringup_launch_path)

    # terminate process
    def signal_handler(sig, frame):
        bringup_handler.shutdown()
        rospy.loginfo("Exiting bringup_node.")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        bringup_handler.start_bringup()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        bringup_handler.shutdown()

if __name__ == "__main__":
    main()

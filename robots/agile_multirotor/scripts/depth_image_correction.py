#!/usr/bin/python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class ImageCorrection:
    def __init__(self):
        rospy.init_node('depth_image_correction', anonymous=False)
        self.cv_bridge = CvBridge()

        self.img_sub = rospy.Subscriber("/multirotor/rgbd/depth/depth_registered", Image, self.img_callback,
                                        queue_size=1)

        # Command publishers
        self.convert_image_pub = rospy.Publisher("/multirotor/converted_image", Image, queue_size=1)

        print("Initialization completed!")

    def img_callback(self, img_data):
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_data, desired_encoding='passthrough')
        # print("cv_image shape")
        # print(cv_image.shape)
        # type(cv_image) = np.ndarray

        # obs_vec = self.img_to_obs(cv_image)

        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)
        

        # command = compute_command_vision_based(self.state, cv_image, rl_policy=self.rl_policy)
        # self.publish_command(command)
    
    # def img_to_obs(self, cv_image):



# https://wiki.ros.org/cv_bridge

if __name__ == '__main__':
    agile_pilot_node = ImageCorrection()
    rospy.spin()
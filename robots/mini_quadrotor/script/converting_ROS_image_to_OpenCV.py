#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
rospy.init_node('image_converter',anonymous=True)
image_pub = rospy.Publisher("/converted_image",Image,queue_size=10)

def camera_callback(msg):
  #msg = Image()
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(msg)
  depth = cv_image.item(240,320)
  print(depth)
  image_pub.publish(msg)

if __name__ == '__main__':
    rospy.Subscriber("/camera/depth/image_rect_raw",Image,camera_callback)
    rospy.spin()


# from __future__ import print_function

# import roslib
# #roslib.load_manifest('my_package')
# import sys
# import rospy
# import cv2
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# class image_converter:

#   def __init__(self):
#     self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)

#     self.bridge = CvBridge()
#     self.image_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.callback)

#   def callback(self,data):
#     try:
#       cv_image = self.bridge.imgmsg_to_cv2(data, "mono16")
#     except CvBridgeError as e:
#       print(e)

#     (rows,cols,channels) = cv_image.shape
#     #if cols > 60 and rows > 60 :
#       #cv2.circle(cv_image, (50,50), 10, 255)

#     cv2.imshow("Image window", cv_image)
#     cv2.waitKey(3)

#     try:
#       self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono16"))
#     except CvBridgeError as e:
#       print(e)

# def main(args):
#   ic = image_converter()
#   rospy.init_node('image_converter', anonymous=True)
#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")
#   cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)

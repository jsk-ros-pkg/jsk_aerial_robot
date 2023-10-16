#!/usr/bin/env python

import rospy
import message_filters
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import RectArray
rospy.init_node('miyamichi')
pub = rospy.Publisher('human', RectArray, queue_size = 4)

def callback(rects,classes):
    msg = RectArray()
    if "person" in classes.label_names:
        for i, label_name in enumerate(classes.label_names):
            if label_name == "person":
                msg.rects.append(rects.rects[i])
    msg.header = rects.header
    pub.publish(msg)

Rect_sub = message_filters.Subscriber('/edgetpu_panorama_object_detector/output/rects', RectArray)
Class_sub = message_filters.Subscriber('/edgetpu_panorama_object_detector/output/class', ClassificationResult)

ts = message_filters.TimeSynchronizer([Rect_sub, Class_sub], 10)
ts.registerCallback(callback)
rospy.spin()

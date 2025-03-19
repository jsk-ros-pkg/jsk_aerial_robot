'''
 Created by jiaxuan and jinjie on 25/01/22.
'''

from functools import wraps
import rospy


def check_first_data_received(obj: object, attr: str, object_name: str):
    """
    Waits until the position is initialized. Logs a message repeatedly
    :param obj:  The object to check the position of
    :param attr:  The attribute of the object to check
    :param object_name:  The name of the object being tracked
    :return:
    """
    while not rospy.is_shutdown() and getattr(obj, attr) is None:
        rospy.loginfo(f"Waiting for {object_name} '{attr}' msg...")
        rospy.sleep(0.2)

    if getattr(obj, attr) is not None:
        rospy.loginfo(f"{object_name} '{attr}' msg is received for the first time")


def check_topic_subscription(func):
    @wraps(func)
    def wrapper(self, topic_name, msg_type, *args, **kwargs):
        try:
            topics = rospy.get_published_topics()
            topic_names = [t[0] for t in topics]
            if topic_name not in topic_names:
                raise ValueError(f"Topic '{topic_name}' is not currently available.")
            subscriber = func(self, topic_name, msg_type, *args, **kwargs)
            return subscriber
        except Exception as e:
            rospy.logerr(f"Error: {str(e)}")
            rospy.signal_shutdown(f"Error: {str(e)}")
            raise e

    return wrapper

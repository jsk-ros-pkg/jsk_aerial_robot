'''
created by Jinjie LI, 2024/02/06

subscribe to the /mocap/pose topic and record the data in a txt file under ~/.ros/ folder
'''
import rospy
import argparse
import os
from geometry_msgs.msg import PoseStamped
import tf.transformations
from datetime import datetime


class MocapPoseSubscriber:
    def __init__(self, file_name, folder_path, convert_to_euler):
        # Initialize the ROS node
        rospy.init_node('mocap_pose_subscriber', anonymous=True)

        # Store the conversion flag and folder path
        self.convert_to_euler = convert_to_euler
        self.folder_path = os.path.expanduser(folder_path)

        # Ensure the directory exists
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)

        # Prepare the file path with a timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.file_path = os.path.join(self.folder_path, f"{file_name}_{timestamp}.txt")

        # Subscribe to the /mocap/pose topic
        self.subscriber = rospy.Subscriber('/mocap/pose', PoseStamped, self.callback)

    def callback(self, data):
        # Extract position
        position = data.pose.position
        x, y, z = position.x, position.y, position.z

        # Extract orientation
        orientation = data.pose.orientation
        qw, qx, qy, qz = orientation.w, orientation.x, orientation.y, orientation.z

        if self.convert_to_euler:
            euler = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
            roll, pitch, yaw = euler
            # change stamp to time in seconds
            formatted_data = f"{data.header.stamp.to_sec()} {x} {y} {z} {roll} {pitch} {yaw}\n"
        else:
            formatted_data = f"{data.header.stamp.to_sec()} {x} {y} {z} {qw} {qx} {qy} {qz}\n"

        # Open the file and append the formatted data
        with open(self.file_path, 'a') as file:
            file.write(formatted_data)
            rospy.loginfo("Pose data recorded.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Subscribe to /mocap/pose and record data.")
    parser.add_argument("--file_name", help="Name of the recording file", default="mocap_pose_data")
    parser.add_argument("--is_euler", help="Convert quaternion to Euler angles", action="store_true")
    parser.add_argument("--folder", help="Folder to save the recording files", default="~/.ros/")
    args = parser.parse_args()

    try:
        recorder = MocapPoseSubscriber(file_name=args.file_name, folder_path=args.folder, convert_to_euler=args.is_euler)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

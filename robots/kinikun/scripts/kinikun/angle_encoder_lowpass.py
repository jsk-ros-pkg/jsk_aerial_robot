#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
from sensor_msgs.msg import JointState
import math

class AS5600JointStatePublisher:
    def __init__(self):
        rospy.init_node('as5600_jointstate_publisher')

        self.init_angles = [None] * 4  # 起動時の初期値
        self.curr_angles = [0] * 4     # 現在の生データ（ラジアン）
        self.filtered_angles = [0] * 4 # フィルタ適用後の角度
        self.alpha = 0.2               # ローパスフィルタ係数（0<alpha<=1）

        self.reverse_flags = [True, True, True, True]

        # Subscriber
        self.subs = [
            rospy.Subscriber('/kinikun1/encoder_angle1', UInt16, self.callback_factory(0)),
            rospy.Subscriber('/kinikun1/encoder_angle2', UInt16, self.callback_factory(1)),
            rospy.Subscriber('/kinikun1/encoder_angle3', UInt16, self.callback_factory(2)),
            rospy.Subscriber('/kinikun1/encoder_angle4', UInt16, self.callback_factory(3)),
            # rospy.Subscriber('/encoder_angle1', UInt16, self.callback_factory(0)),
            # rospy.Subscriber('/encoder_angle2', UInt16, self.callback_factory(1)),
            # rospy.Subscriber('/encoder_angle3', UInt16, self.callback_factory(2)),
            # rospy.Subscriber('/encoder_angle4', UInt16, self.callback_factory(3)),
        ]

        # Publisher
        self.pub = rospy.Publisher('/kinikun1/joint_states', JointState, queue_size=1)

        # Timerで定期送信
        rospy.Timer(rospy.Duration(0.02), self.publish_joint_state)  # 50Hz

    def callback_factory(self, index):
        def callback(msg):
            raw = msg.data
            if self.init_angles[index] is None:
                self.init_angles[index] = raw
            delta = (raw - self.init_angles[index]) & 0xFFF
            if delta > 2048:
                delta -= 4096

            angle_deg = delta * (360.0 / 4096.0)
            angle_rad = math.radians(angle_deg)
            if self.reverse_flags[index]:
                angle_rad = -angle_rad

            # ローパスフィルタ（指数移動平均）
            prev = self.filtered_angles[index]
            filtered = self.alpha * angle_rad + (1 - self.alpha) * prev
            self.filtered_angles[index] = filtered
        return callback

    def publish_joint_state(self, event):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['arm1_joint', 'arm2_joint', 'arm3_joint', 'arm4_joint']
        msg.position = self.filtered_angles
        msg.velocity = []
        msg.effort = []
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        AS5600JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

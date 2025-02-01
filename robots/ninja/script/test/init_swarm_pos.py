#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
import tf.transformations as tft
from aerial_robot_msgs.msg import FlightNav

def main():
    rospy.init_node("example_ninja_transform")

    nav_pub_2 = rospy.Publisher("/ninja2/uav/nav", FlightNav, queue_size=1)
    nav_pub_3 = rospy.Publisher("/ninja3/uav/nav", FlightNav, queue_size=1)
    flight_nav = FlightNav()
    flight_nav.target = FlightNav.COG
    flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    d = 1.0          # 点Cまでの距離
    a_2 = -2*math.pi /3 # 回転角(例: π/2)
    a_3 = 2*math.pi /3 # 回転角(例: π/2)

    rate = rospy.Rate(10)  # 1Hz
    while not rospy.is_shutdown():
        try:
            #
            # 1) world→/ninja1/cog を取得 (trans_n1, rot_n1)
            #
            (trans_n1, rot_n1) = listener.lookupTransform(
                "world", "/ninja1/cog", rospy.Time(0)
            )

            # 4x4同次変換行列に
            T_w_n1 = tft.quaternion_matrix(rot_n1)
            T_w_n1[0:3, 3] = trans_n1

            #
            # 2) 「点Cフレーム(/ninjaC)」を /ninja1/cog を親としてブロードキャスト
            #
            #    /ninja1/cog 原点から (d,0,0) へ平行移動、回転なし
            #
            br.sendTransform(
                (d, 0, 0),            # x方向 d
                (0, 0, 0, 1),         # 回転なし
                rospy.Time.now(),
                "ninjaC",             # 子フレーム名
                "/ninja1/cog"         # 親フレーム名
            )

            #
            # 3) 「C を中心に /ninja1/cog を a 回転 → /ninja2/cog」用の行列を計算
            #
            #   old = n1, new = n2, 中心C=(d,0,0)、軸 = z軸、として
            #   T_n2_n1 = T(+C)*R(a)*T(-C)
            #
            T_n2_n1 = np.dot(
                tft.translation_matrix((d,0,0)),
                np.dot(
                    tft.rotation_matrix(a_2, (0,0,1)),
                    tft.translation_matrix((-d,0,0))
                )
            )

            T_n3_n1 = np.dot(
                tft.translation_matrix((d,0,0)),
                np.dot(
                    tft.rotation_matrix(a_3, (0,0,1)),
                    tft.translation_matrix((-d,0,0))
                )
            )

            # tf のチェーンに合わせて逆行列を取る
            T_n1_n2 = np.linalg.inv(T_n2_n1)
            T_n1_n3 = np.linalg.inv(T_n3_n1)

            # world→ninja2 = world→ninja1 × n1→n2
            T_w_n2 = np.dot(T_w_n1, T_n1_n2)
            T_w_n3 = np.dot(T_w_n1, T_n1_n3)

            # (xyz, quat)に戻してブロードキャスト
            trans_n2 = T_w_n2[0:3, 3]
            quat_n2  = tft.quaternion_from_matrix(T_w_n2)

            trans_n3 = T_w_n3[0:3, 3]
            quat_n3  = tft.quaternion_from_matrix(T_w_n3)

            flight_nav.target_pos_x = trans_n2[0]
            flight_nav.target_pos_y = trans_n2[1]
            flight_nav.target_yaw = tft.euler_from_quaternion(quat_n2)[2]
            nav_pub_2.publish(flight_nav)

            flight_nav.target_pos_x = trans_n3[0]
            flight_nav.target_pos_y = trans_n3[1]
            flight_nav.target_yaw = tft.euler_from_quaternion(quat_n3)[2]
            nav_pub_3.publish(flight_nav)

            br.sendTransform(
                trans_n2,
                quat_n2,
                rospy.Time.now(),
                "/ninja2/swarm_cog",
                "world"
            )


            br.sendTransform(
                trans_n3,
                quat_n3,
                rospy.Time.now(),
                "/ninja3/swarm_cog",
                "world"
            )

            rospy.loginfo(
                "Broadcasted frames:\n"
                "  - /ninjaC   (child of /ninja1/cog)\n"
                "  - /ninja2/cog (child of world)\n"
            )

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed.")

        rate.sleep()

if __name__ == "__main__":
    main()

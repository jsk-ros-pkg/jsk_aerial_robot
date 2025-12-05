#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF (/delta/cog, /delta/thrust1, ...) から
RobotModel::calcWrenchMatrixOnCoG() と同じ Q を計算し、
さらに /delta/four_axes/command の base_thrust から
各軸のレンチ w = Q * base_thrust を計算して CSV に保存するスクリプト。

- 各時刻ごとに 1 行:
  time, Q(0,0)..Q(5,5), wrench[0..5]
"""

import csv
import os
import numpy as np
import rospy
import tf

from spinal.msg import FourAxisCommand
# PoseControlPid は今回は使わないので削除してOK

# ===== ユーザー設定パラメータ =====

ROTOR_IDS = [1, 2, 3, 4, 5, 6]

rotor_direction = {
    1: +1.0,
    2: -1.0,
    3: +1.0,
    4: -1.0,
    5: +1.0,
    6: -1.0,
}

m_f_rate = {
    1: 0.0156,
    2: 0.0156,
    3: 0.0156,
    4: 0.0156,
    5: 0.0064,
    6: 0.0064,
}

COG_FRAME = "/delta/cog"
THRUST_FRAME_FMT = "/delta/thrust{}"

FOUR_AXES_BASE_THRUST_INDEX_OF_ROTOR = {
    rid: rid - 1 for rid in ROTOR_IDS
}

# ================================

_last_base_thrust = None  # np.ndarray(shape=(<=rotor_num,))
_log_writer = None
_log_file = None
_start_time = None


def four_axes_command_callback(msg: FourAxisCommand):
    global _last_base_thrust
    _last_base_thrust = np.array(msg.base_thrust, dtype=float)


def compute_wrench_matrix_on_cog(listener: tf.TransformListener):
    """
    TF から Q を計算する。

    C++ の RobotModel::calcWrenchMatrixOnCoG と同じ定義:
      Q(0:3, i) = u_i
      Q(3:6, i) = p_i × u_i + m_f_rate[i] * sigma[i+1] * u_i

    戻り値:
      Q: np.ndarray (6, rotor_num)
    """
    rotor_num = len(ROTOR_IDS)
    Q = np.zeros((6, rotor_num), dtype=float)

    ez = np.array([0.0, 0.0, 1.0])

    for col, rid in enumerate(ROTOR_IDS):
        rotor_frame = THRUST_FRAME_FMT.format(rid)

        try:
            # rotor -> COG の変換を COG フレームで取得
            trans, quat = listener.lookupTransform(
                COG_FRAME, rotor_frame, rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(1.0, "TF lookup failed for %s", rotor_frame)
            return None

        p = np.array(trans, dtype=float)

        R4 = tf.transformations.quaternion_matrix(quat)
        R = R4[0:3, 0:3]

        u = R @ ez
        if np.linalg.norm(u) > 1e-8:
            u = u / np.linalg.norm(u)

        sigma = rotor_direction.get(rid, 0.0)
        mf = m_f_rate.get(rid, 0.0)

        Q[0:3, col] = u
        Q[3:6, col] = np.cross(p, u) + mf * sigma * u

    return Q


def open_log_file():
    global _log_writer, _log_file

    # パラメータからパス取得（なければカレントの wrench_log.csv）
    path = rospy.get_param("~wrench_log_path", "wrench_log.csv")
    # ディレクトリがあれば作る
    dirname = os.path.dirname(path)
    if dirname and not os.path.exists(dirname):
        os.makedirs(dirname, exist_ok=True)

    _log_file = open(path, "w", newline="")
    _log_writer = csv.writer(_log_file)

    # ヘッダ
    header = ["time"]
    # Q の成分 (row-major: Q_00, Q_01, ..., Q_55)
    for i in range(6):
        for j in range(len(ROTOR_IDS)):
            header.append(f"Q_{i}{j}")
    # wrench 成分
    for i, name in enumerate(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]):
        header.append(name)

    _log_writer.writerow(header)
    rospy.loginfo("Logging to %s", path)


def close_log_file():
    global _log_file
    if _log_file is not None:
        _log_file.flush()
        _log_file.close()
        _log_file = None
        rospy.loginfo("Log file closed.")


def main():
    global _last_base_thrust, _start_time

    rospy.init_node("compute_wrench_and_save_csv")
    np.set_printoptions(suppress=True, linewidth=200, precision=6)

    listener = tf.TransformListener()

    rospy.Subscriber(
        "/delta/four_axes/command", FourAxisCommand, four_axes_command_callback
    )

    open_log_file()

    # TF バッファ
    rospy.sleep(1.0)

    _start_time = rospy.Time.now()

    rate = rospy.Rate(200)  # ログ頻度（必要なら下げてOK）
    rotor_num = len(ROTOR_IDS)

    try:
        while not rospy.is_shutdown():
            Q = compute_wrench_matrix_on_cog(listener)
            if Q is None:
                rate.sleep()
                continue

            if _last_base_thrust is None:
                rate.sleep()
                continue

            # base_thrust の長さチェック（足りなければスキップ）
            if _last_base_thrust.shape[0] < rotor_num:
                rospy.logwarn_throttle(1.0,
                    "base_thrust size (%d) < rotor_num (%d)",
                    _last_base_thrust.shape[0], rotor_num
                )
                rate.sleep()
                continue

            # wrench = Q * base_thrust
            thrust = _last_base_thrust[:rotor_num]
            wrench = Q.dot(thrust)  # shape=(6,)

            # 時刻 [s]（開始からの相対時間）
            t = (rospy.Time.now() - _start_time).to_sec()

            # Q を row-major でフラットに
            q_flat = Q.reshape(-1)  # (6*rotor_num,)

            row = [t]
            row.extend(q_flat.tolist())
            row.extend(wrench.tolist())

            _log_writer.writerow(row)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        close_log_file()


if __name__ == "__main__":
    main()

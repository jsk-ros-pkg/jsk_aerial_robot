#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray

from spinal.msg import FourAxisCommand

import matplotlib.pyplot as plt
import time

# ========= グローバル状態 =========

full_q_mat = None          # np.ndarray shape = (6, N_full)
full_lambda = None         # np.ndarray shape = (N_full,)
q_mat = None               # np.ndarray shape = (6, N_motor)
last_four_axes_cmd = None  # FourAxisCommand

last_target_wrench = None  # np.ndarray shape = (6,)
last_full_wrench = None    # np.ndarray shape = (6,)
last_qmat_wrench = None    # np.ndarray shape = (6,)

AXES = ["x", "y", "z", "roll", "pitch", "yaw"]

# ログ用
t0 = None
t_history = []                # 時刻
target_history = []           # 各要素が shape=(6,) の np.array
full_history = []             # 同上
qmat_history = []             # 同上


# ========= ヘルパ関数 =========

def multiarray_to_matrix(msg):
    """Float64MultiArray → 2D numpy 配列"""
    if len(msg.layout.dim) < 2:
        data = np.array(msg.data, dtype=float)
        return data.reshape((-1, 1))

    rows = msg.layout.dim[0].size
    cols = msg.layout.dim[1].size
    data = np.array(msg.data, dtype=float)
    if data.size != rows * cols:
        rospy.logwarn("Float64MultiArray size mismatch: data=%d, rows*cols=%d",
                      data.size, rows * cols)
        cols = data.size // rows
    return data.reshape((rows, cols))


def multiarray_to_vector(msg):
    """FloatXXMultiArray → 1D numpy 配列"""
    return np.array(msg.data, dtype=float)


def log_current_state():
    """target / FULL / QMAT の現時点の値を履歴に追加"""
    global t0, t_history, target_history, full_history, qmat_history
    global last_target_wrench, last_full_wrench, last_qmat_wrench

    if last_target_wrench is None:
        return

    # 時刻
    now = time.time()
    if t0 is None:
        t0 = now
    t = now - t0

    t_history.append(t)
    target_history.append(last_target_wrench.copy())

    if last_full_wrench is not None:
        full_history.append(last_full_wrench.copy())
    else:
        full_history.append(np.full(6, np.nan))

    if last_qmat_wrench is not None:
        qmat_history.append(last_qmat_wrench.copy())
    else:
        qmat_history.append(np.full(6, np.nan))


def plot_history():
    """Ctrl+C でノード終了時に、時系列グラフを表示"""
    if len(t_history) == 0:
        rospy.logwarn("No history to plot.")
        return

    t_arr = np.array(t_history)
    target_arr = np.vstack(target_history)  # (T, 6)
    full_arr = np.vstack(full_history)      # (T, 6)
    qmat_arr = np.vstack(qmat_history)      # (T, 6)

    fig, axes = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    fig.suptitle("Target vs FULL vs QMAT wrench (per axis)")

    for i, name in enumerate(AXES):
        ax = axes[i]
        ax.plot(t_arr, full_arr[:, i], label="FULL wrench")
        ax.plot(t_arr, qmat_arr[:, i], label="QMAT wrench")
        ax.plot(t_arr, target_arr[:, i], label="TARGET wrench")
        ax.set_ylabel(name)
        ax.grid(True)
        ax.legend(loc="best", fontsize=8)

    axes[-1].set_xlabel("time [s]")

    plt.tight_layout()
    plt.show()


def recompute_from_full():
    global full_q_mat, full_lambda, last_full_wrench

    if full_q_mat is None or full_lambda is None:
        return

    if full_q_mat.shape[1] != full_lambda.shape[0]:
        rospy.logwarn("full_q_mat cols (%d) != full_lambda size (%d)",
                      full_q_mat.shape[1], full_lambda.shape[0])
        return

    last_full_wrench = full_q_mat.dot(full_lambda)  # shape=(6,)

    Fx, Fy, Fz, Tx, Ty, Tz = last_full_wrench.tolist()
    rospy.loginfo(
        "[FULL] Wrench: Fx=%.4f Fy=%.4f Fz=%.4f, Tx=%.4f Ty=%.4f Tz=%.4f",
        Fx, Fy, Fz, Tx, Ty, Tz
    )

    log_current_state()


def recompute_from_qmat():
    global q_mat, last_four_axes_cmd, last_qmat_wrench

    if q_mat is None or last_four_axes_cmd is None:
        return

    thrust = np.array(last_four_axes_cmd.base_thrust, dtype=float)
    if q_mat.shape[1] != thrust.shape[0]:
        rospy.logwarn("q_mat cols (%d) != base_thrust size (%d)",
                      q_mat.shape[1], thrust.shape[0])
        return

    last_qmat_wrench = q_mat.dot(thrust)  # shape=(6,)

    Fx, Fy, Fz, Tx, Ty, Tz = last_qmat_wrench.tolist()
    rospy.loginfo(
        "[QMAT] Wrench: Fx=%.4f Fy=%.4f Fz=%.4f, Tx=%.4f Ty=%.4f Tz=%.4f",
        Fx, Fy, Fz, Tx, Ty, Tz
    )

    log_current_state()


# ========= コールバック =========

def full_q_mat_cb(msg: Float64MultiArray):
    global full_q_mat
    full_q_mat = multiarray_to_matrix(msg)
    if full_q_mat.shape[0] != 6:
        rospy.logwarn("full_q_mat rows=%d (expected 6)", full_q_mat.shape[0])
    recompute_from_full()


def full_lambda_cb(msg: Float64MultiArray):
    global full_lambda
    full_lambda = multiarray_to_vector(msg)
    recompute_from_full()


def q_mat_cb(msg: Float64MultiArray):
    global q_mat
    q_mat = multiarray_to_matrix(msg)
    if q_mat.shape[0] != 6:
        rospy.logwarn("q_mat rows=%d (expected 6)", q_mat.shape[0])
    recompute_from_qmat()


def four_axes_cmd_cb(msg: FourAxisCommand):
    global last_four_axes_cmd
    last_four_axes_cmd = msg
    recompute_from_qmat()


def target_wrench_cb(msg: Float32MultiArray):
    """ /delta/debug/wrench_target_cog のコールバック """
    global last_target_wrench
    data = np.array(msg.data, dtype=float)
    if data.size != 6:
        rospy.logwarn("wrench_target_cog size=%d (expected 6)", data.size)
        return
    last_target_wrench = data
    log_current_state()


# ========= main =========

def main():
    rospy.init_node("wrench_recompute_debug")

    rospy.Subscriber("/delta/debug/full_torque_allocation_matrix",
                     Float64MultiArray, full_q_mat_cb, queue_size=1)
    rospy.Subscriber("/delta/debug/full_lambda",
                     Float64MultiArray, full_lambda_cb, queue_size=1)
    rospy.Subscriber("/delta/debug/torque_allocation_matrix",
                     Float64MultiArray, q_mat_cb, queue_size=1)
    rospy.Subscriber("/delta/four_axes/command",
                     FourAxisCommand, four_axes_cmd_cb, queue_size=1)
    rospy.Subscriber("/delta/debug/target_wrench_cog",
                     Float32MultiArray, target_wrench_cb, queue_size=1)

    rospy.loginfo("wrench_recompute_debug node started.")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # ノード終了時にグラフ表示
        plot_history()


if __name__ == "__main__":
    main()

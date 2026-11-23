import matplotlib as mpl
mpl.rcParams['pdf.fonttype'] = 42     # TrueTypeフォントで埋め込む
mpl.rcParams['ps.fonttype'] = 42      # EPS出力も同様に
mpl.rcParams['font.sans-serif'] = ['Arial']  # 代替フォント
mpl.rcParams['font.family'] = 'sans-serif'

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from sklearn.metrics import mean_squared_error
from scipy.spatial.transform import Rotation as R
from scipy.signal import correlate
import rospy

# ====== 設定 ======
bag_path = "../rosbag/2025-07-25-16-55-00_hydrus_xi_teleop.bag"
flight_nav = "/hydrus_xi/uav/nav"
robot_odom = "/hydrus_xi/uav/cog/odom"
robot_mocap = "/hydrus_xi/mocap/pose"
joints_ctrl = "/hydrus_xi/joints_ctrl"
joint_states = "/hydrus_xi/joint_states"

def nav_extract_data(bag, topic):
    times = []
    values = []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        time_sec = t.to_sec()
        arr = [msg.target_pos_x, msg.target_pos_y, msg.target_pos_z, msg.target_yaw]
        times.append(time_sec)
        values.append(arr)
    return np.array(times), np.array(values)

def odom_extract_data(bag, topic):
    times = []
    values = []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        time_sec = t.to_sec()
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        rot = R.from_quat(q)
        e = rot.as_euler('xyz')
        arr = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, e[2]]
        times.append(time_sec)
        values.append(arr)
    return np.array(times), np.array(values)

def mocap_extract_data(bag, topic):
    times = []
    values = []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        time_sec = t.to_sec()
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rot = R.from_quat(q)
        e = rot.as_euler('xyz')
        arr = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, e[2]]
        times.append(time_sec)
        values.append(arr)
    return np.array(times), np.array(values)

def joints_ctrl_extract_data(bag, topic):
    times = []
    values = []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        time_sec = t.to_sec()
        arr = [msg.position[0], msg.position[1], msg.position[2]]
        times.append(time_sec)
        values.append(arr)
    return np.array(times), np.array(values)

def joint_states_extract_data(bag, topic):
    times = []
    values = []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        time_sec = t.to_sec()
        arr = [msg.position[4], msg.position[5], msg.position[6]]
        times.append(time_sec)
        values.append(arr)
    return np.array(times), np.array(values)

def synchronize_and_interpolate(t_ref, y_ref, t_target, y_target):
    """基準側(t_ref)に合わせて、target側のデータを線形補間"""
    interp_funcs = [interp1d(t_target, y_target[:, i], bounds_error=False, fill_value="extrapolate") for i in range(y_target.shape[1])]
    y_interp = np.stack([f(t_ref) for f in interp_funcs], axis=1)
    return y_interp

def compute_rmse(y_true, y_pred):
    return np.sqrt(np.mean((y_true - y_pred) ** 2, axis=0))

def estimate_delay(t, y_target, y_measured, labels, t_start=None, t_end=None):
    delays = {}

    # 時間インデックスの範囲を指定（ない場合は全体）
    if t_start is None:
        t_start = t[0]
    if t_end is None:
        t_end = t[-1]

    mask = (t >= t_start) & (t <= t_end)
    t_crop = t[mask]
    y_target_crop = y_target[mask]
    y_measured_crop = y_measured[mask]

    dt = np.mean(np.diff(t_crop))  # サンプリング周期

    for i, label in enumerate(labels):
        sig_target = y_target_crop[:, i] - np.mean(y_target_crop[:, i])
        sig_measured = y_measured_crop[:, i] - np.mean(y_measured_crop[:, i])
        
        corr = correlate(sig_measured, sig_target, mode='full')
        lag_idx = np.argmax(corr) - (len(sig_target) - 1)
        delay_time = lag_idx * dt
        
        delays[label] = delay_time
        print(f"Delay for {label} (in {t_start:.1f}s - {t_end:.1f}s): {delay_time:.3f} sec")

    return delays

def plot_comparison(t, y_target, y_measured, labels, title_prefix):
    for i in range(len(labels)):
        plt.rcParams['font.size'] = 28
        fig, ax = plt.subplots()

        ax.plot(t - t[0], y_target[:, i], label='Target')
        ax.plot(t - t[0], y_measured[:, i], label='Measured', alpha=0.7)

        # ax.set_xlabel("Time [s]")
        # X軸の目盛りを消す（軸線は残す）
        ax.tick_params(axis='x', which='both', bottom=False, labelbottom=False)

        # Y軸ラベル設定
        if labels[i] in ['x', 'y', 'z']:
            ylabel = labels[i] + " [m]"
            ax.set_ylabel(ylabel)
        elif labels[i] in ['yaw', 'q1', 'q2', 'q3']:
            ylabel = labels[i] + " [rad]"
            ax.set_ylabel(ylabel)

        # ax.title(f"{title_prefix}: {labels[i]}")
        # ax.legend()
        ax.grid(True)
        plt.tight_layout()
        plt.show()

import matplotlib.pyplot as plt
import numpy as np

def plot_all_comparisons_with_time_window(
    t_pose, y_target_pose, y_measured_pose,
    t_joint, y_target_joint, y_measured_joint,
    width=10, height_per_plot=2,
    start_time=None, end_time=None
):

    labels_pose = [r"$x$", r"$y$", r"$z$", r'$yaw$']
    labels_joint = [r'$q_1$', r'$q_2$', r'$q_3$']

    # 時間範囲でのフィルタリング
    if start_time is not None and end_time is not None:
        mask_pose = (t_pose >= start_time) & (t_pose <= end_time)
        mask_joint = (t_joint >= start_time) & (t_joint <= end_time)
        t_pose = t_pose[mask_pose]
        y_target_pose = y_target_pose[mask_pose]
        y_measured_pose = y_measured_pose[mask_pose]
        t_joint = t_joint[mask_joint]
        y_target_joint = y_target_joint[mask_joint]
        y_measured_joint = y_measured_joint[mask_joint]

    # プロット数の合計
    total_plots = len(labels_pose) + len(labels_joint)
    fig, axes = plt.subplots(total_plots, 1, figsize=(width, height_per_plot * total_plots), sharex=True)

    # Pose部分
    for i in range(len(labels_pose)):
        ax = axes[i]
        ax.plot(t_pose, y_target_pose[:, i], label='Target', linestyle='--')
        ax.plot(t_pose, y_measured_pose[:, i], label='Measured', alpha=0.8)
        if labels_pose[i] in [r"$x$", r"$y$", r"$z$"]:
            ylabel = labels_pose[i] + " [m]"
        elif labels_pose[i] == r"$yaw$":
            ylabel = labels_pose[i] + " [rad]"
        ax.set_ylabel(ylabel)
        ax.grid(True)

    # Joint部分
    for i in range(len(labels_joint)):
        ax = axes[len(labels_pose) + i]
        ax.plot(t_joint, y_target_joint[:, i], label='Target', linestyle='--')
        ax.plot(t_joint, y_measured_joint[:, i], label='Measured', alpha=0.8)
        ylabel = labels_joint[i] + " [rad]"
        ax.set_ylabel(ylabel)
        ax.grid(True)

    # 最後の軸にx軸ラベルを付ける
    axes[-1].set_xlabel("Time [s]")

    # 凡例
    axes[0].legend(loc='upper right')

    # レイアウト調整
    plt.tight_layout()
    plt.show()


def plot_all_signals(
    t_pose, y_target_pose, y_measured_pose,
    t_joint, y_target_joint, y_measured_joint,
    width=10, height_per_plot=2
):
    labels = [r"$x$", r"$y$", r"$z$", r"$yaw$", r"$q_1$", r"$q_2$", r"$q_3$"]
    num_plots = len(labels)

    # 結合データ（横軸をそろえる）
    t_pose_rel = t_pose - t_pose[0]
    t_joint_rel = t_joint - t_pose[0]  # t_pose[0]基準に揃える（同じroscoreなら同じ基準のはず）

    # 図サイズ
    fig, axes = plt.subplots(num_plots, 1, figsize=(width, height_per_plot * num_plots), sharex=True)
    # fig.suptitle("Target vs Measured Signals", fontsize=14)

    # 統一された描画ループ
    for i in range(4):  # x, y, z, yaw
        ax = axes[i]
        ax.plot(t_pose_rel, y_target_pose[:, i], label='Target', color='blue')
        ax.plot(t_pose_rel, y_measured_pose[:, i], label='Measured', color='red')
        if labels[i] in [r"$x$", r"$y$", r"$z$"]:
            ylabel = labels[i] + " [m]"
            ax.set_ylabel(ylabel)
        elif labels[i] == r"$yaw$":
            ylabel = labels[i] + " [rad]"
            ax.set_ylabel(ylabel)
        ax.grid(True)
        if i == 0:
            ax.legend(loc='upper right')

    for i in range(3):  # q1, q2, q3
        ax = axes[i + 4]
        ax.plot(t_joint_rel, y_target_joint[:, i], label='Target', color='blue')
        ax.plot(t_joint_rel, y_measured_joint[:, i], label='Measured', color='red')
        ylabel = labels[i + 4] + " [rad]"
        ax.set_ylabel(ylabel)
        ax.grid(True)

    axes[-1].set_xlabel("Time [s]")

    plt.tight_layout(rect=[0, 0, 1, 0.97])  # タイトルスペース確保
    plt.show()


# ====== メイン処理 ======
with rosbag.Bag(bag_path) as bag:
    t_fn, y_fn = nav_extract_data(bag, flight_nav)
    t_ro, y_ro = odom_extract_data(bag, robot_odom)
    t_rm, y_rm = mocap_extract_data(bag, robot_mocap)
    t_jc, y_jc = joints_ctrl_extract_data(bag, joints_ctrl)
    t_js, y_js = joint_states_extract_data(bag, joint_states)

# 共通開始時間：目標値の方が遅い → その中で最も遅い時刻に合わせる
start_time = max(t_fn[0], t_jc[0])

# 各データをスタート時刻以降に切り詰め
def trim(t, y, start_time):
    idx = t >= start_time
    return t[idx], y[idx]

t_fn, y_fn = trim(t_fn, y_fn, start_time)
t_ro, y_ro = trim(t_ro, y_ro, start_time)
t_rm, y_rm = trim(t_rm, y_rm, start_time)
t_jc, y_jc = trim(t_jc, y_jc, start_time)
t_js, y_js = trim(t_js, y_js, start_time)

# 実測値を目標値に補間
y_ro_interp = synchronize_and_interpolate(t_fn, y_fn, t_ro, y_ro)
y_rm_interp = synchronize_and_interpolate(t_fn, y_fn, t_rm, y_rm)
y_js_interp = synchronize_and_interpolate(t_jc, y_jc, t_js, y_js)

# RMSE 計算
print("==== RMSE (x, y, z, yaw) ====")
rmse_pose = compute_rmse(y_fn, y_ro_interp)
print("Pose RMSE (x, y, z, yaw):", rmse_pose)
# rmse_pose = compute_rmse(y_fn, y_rm_interp)
print("==== RMSE (q1, q2, q3) ====")
rmse_q = compute_rmse(y_jc, y_js_interp)
print("Joint RMSE (q1, q2, q3):", rmse_q)

print("==== Pose delay (x, y, z, yaw) ====")
delay_pose = estimate_delay(t_fn, y_fn, y_ro_interp, ['x', 'y', 'z', 'yaw'], t_fn[0], t_fn[-1])

print("==== Joint delay (q1, q2, q3) ====")
delay_joint = estimate_delay(t_jc, y_jc, y_js_interp, ['q1', 'q2', 'q3'], t_jc[0], t_jc[-1])

# プロット
# plot_all_comparisons_with_time_window(
#     t_pose=t_fn,
#     y_target_pose=y_fn,
#     y_measured_pose=y_ro_interp,
#     t_joint=t_jc,
#     y_target_joint=y_jc,
#     y_measured_joint=y_js_interp,
#     width=6,          # 横サイズ（インチ）
#     height_per_plot=1.9,  # 各プロットの縦サイズ
#     start_time = 0.0,
#     end_time = 60.0
# )
#  ERROR 何も表示されない

plot_all_signals(
    t_pose=t_fn,
    y_target_pose=y_fn,
    y_measured_pose=y_ro_interp,
    t_joint=t_jc,
    y_target_joint=y_jc,
    y_measured_joint=y_js_interp,
    width=6,          # 横サイズ（インチ）
    height_per_plot=1.9  # 各プロットの縦サイズ
)

# plot_comparison(t_fn, y_fn, y_ro_interp, ['x', 'y', 'z', 'yaw'], "Pose")
# plot_comparison(t_jc, y_jc, y_js_interp, ['q1', 'q2', 'q3'], "Joint Angle")

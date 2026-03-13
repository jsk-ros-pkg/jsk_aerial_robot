#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String, Empty

def get_start_end_times_for_vel_mode(bag, mode_topic):
    start_time = None
    end_time = None
    for _, msg, t in bag.read_messages(topics=[mode_topic]):
        # if not isinstance(msg, String):
        #     continue
        if msg.data == 'vel' and start_time is None:
            start_time = t.to_sec()
            print(f"[Trigger] START detected at {start_time:.3f} [s]")
        elif msg.data == 'pos' and start_time is not None and end_time is None:
            end_time = t.to_sec()
            print(f"[Trigger] END detected at {end_time:.3f} [s]")
            break  # 最初の区間だけ
    return start_time, end_time

def get_start_end_times_for_pos_mode(bag, mode_topic, land_topic):
    start_time = None
    end_time = None
    for topic, msg, t in bag.read_messages(topics=[mode_topic,land_topic]):
        # if not isinstance(msg, String):
        #     continue
        if topic == mode_topic and msg.data == 'pos' and start_time is None:
            start_time = t.to_sec()
            print(f"[Trigger] START detected at {start_time:.3f} [s]")
        elif topic == land_topic and start_time is not None and end_time is None:
            end_time = t.to_sec()
            print(f"[Trigger] END detected at {end_time:.3f} [s]")
            break  # 最初の区間だけ
    return start_time, end_time

def read_uav_nav_data(bag, topic, start_time=None, end_time=None, t0=None):
    xs, ys, zs, rolls, pitchs, yaws, times = [], [], [], [], [], [], []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        t_sec = t.to_sec()
        if start_time and t_sec < start_time:
            continue
        if end_time and t_sec > end_time:
            continue
        xs.append(msg.target_pos_x)
        ys.append(msg.target_pos_y)
        zs.append(msg.target_pos_z)
        rolls.append(msg.target_roll)
        pitchs.append(msg.target_pitch)
        yaws.append(msg.target_yaw)
        if t0 is not None:
            t_sec = t_sec - t0
        times.append(t_sec)
    return np.array(xs), np.array(ys), np.array(zs), np.array(rolls), np.array(pitchs), np.array(yaws), np.array(times)

def read_mocap_data(bag, topic, start_time=None, end_time=None, t0=None):
    xs, ys, zs, rolls, pitchs, yaws, times = [], [], [], [], [], [], []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        t_sec = t.to_sec()
        if start_time and t_sec < start_time:
            continue
        if end_time and t_sec > end_time:
            continue
        xs.append(msg.pose.position.x)
        ys.append(msg.pose.position.y)
        zs.append(msg.pose.position.z)
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rot = R.from_quat(q)
        e = rot.as_euler('xyz')
        rolls.append(e[0])
        pitchs.append(e[1])
        yaws.append(e[2])
        if t0 is not None:
            t_sec = t_sec - t0
        times.append(t_sec)
    return np.array(xs), np.array(ys), np.array(zs), np.array(rolls), np.array(pitchs), np.array(yaws), np.array(times)

def read_wrench_data(bag, topic, start_time=None, end_time=None, t0=None):
    xs, ys, zs, rolls, pitchs, yaws, times = [], [], [], [], [], [], []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        t_sec = t.to_sec()
        if start_time and t_sec < start_time:
            continue
        if end_time and t_sec > end_time:
            continue
        if topic == '/cfs/data':
            xs.append(msg.wrench.force.z)
            ys.append(msg.wrench.force.x)
            zs.append(msg.wrench.force.y)
            rolls.append(msg.wrench.torque.z)
            pitchs.append(msg.wrench.torque.x)
            yaws.append(msg.wrench.torque.y)
        else:
            xs.append(msg.wrench.force.x)
            ys.append(msg.wrench.force.y)
            zs.append(msg.wrench.force.z)
            rolls.append(msg.wrench.torque.x)
            pitchs.append(msg.wrench.torque.y)
            yaws.append(msg.wrench.torque.z)
        if t0 is not None:
            t_sec = t_sec - t0
        times.append(t_sec)
    return np.array(xs), np.array(ys), np.array(zs), np.array(rolls), np.array(pitchs), np.array(yaws), np.array(times)

def compute_rmse(x1, x2):
    return np.sqrt(np.mean((x1 - x2) ** 2))

def dynamic_ylim(data1, data2, margin_ratio=0.05):
    """データ範囲に基づいてY軸を自動調整"""
    dmin = min(np.min(data1), np.min(data2))
    dmax = max(np.max(data1), np.max(data2))
    margin = (dmax - dmin) * margin_ratio
    return [dmin - margin, dmax + margin]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot two PoseStamped topics and compute RMSE.")
    parser.add_argument("bagfile", help="Path to rosbag file")
    parser.add_argument("--pos_or_vel", default="vel")
    # parser.add_argument("--topic_a", required=True, help="Topic name for A")
    # parser.add_argument("--topic_b", required=True, help="Topic name for B")
    args = parser.parse_args()

    bag = rosbag.Bag(args.bagfile)

    if args.pos_or_vel == 'vel':
      start_time, end_time = get_start_end_times_for_vel_mode(bag, '/twin_hammer/teleop_mode')
    elif args.pos_or_vel == 'pos':
      start_time, end_time = get_start_end_times_for_pos_mode(bag, '/twin_hammer/teleop_mode', '/twin_hammer/teleop_command/land')
    if start_time is None or end_time is None:
        print("[Error] トリガー文字が見つかりませんでした。")
        bag.close()
        exit(1)

    t0 = bag.get_start_time()
    # Read data
    x_nav, y_nav, z_nav, roll_nav, pitch_nav, yaw_nav, t_nav = read_uav_nav_data(bag, '/gimbalrotor/uav/nav', start_time, end_time, t0)
    x_mocap, y_mocap, z_mocap, roll_mocap, pitch_mocap, yaw_mocap, t_mocap = read_mocap_data(bag, '/gimbalrotor/mocap/pose', start_time, end_time, t0)
    x_device_mocap, y_device_mocap, z_device_mocap, roll_device_mocap, pitch_device_mocap, yaw_device_mocap, t_device_mocap = read_mocap_data(bag, '/twin_hammer/mocap/pose', start_time, end_time, t0)
    x_wrench_cfs, y_wrench_cfs, z_wrench_cfs, roll_wrench_cfs, pitch_wrench_cfs, yaw_wrench_cfs, t_wrench_cfs = read_wrench_data(bag, '/cfs/data', start_time, end_time, t0)
    x_wrench_fb, y_wrench_fb, z_wrench_fb, roll_wrench_fb, pitch_wrench_fb, yaw_wrench_fb, t_wrench_fb = read_wrench_data(bag, '/twin_hammer/haptics_wrench', start_time, end_time, t0)

    bag.close()

    device_init_pos = [x_device_mocap[0], y_device_mocap[0], z_device_mocap[0]]
    device_init_att = [roll_device_mocap[0], pitch_device_mocap[0], yaw_device_mocap[0]]

    # 補間 (uavとmocapのタイムスタンプを合わせる)
    if len(t_nav) == 0 or len(t_mocap) == 0:
        print("指定された時間範囲でデータがありません。")
        exit()

    # 共通の時間基準を作る（robotのuavを基準にrobotのmocapを補間）
    t_common = np.linspace(max(t_nav[0], t_mocap[0]), min(t_nav[-1], t_mocap[-1]), min(len(t_nav), len(t_mocap)))
    x_nav_interp = np.interp(t_common, t_nav, x_nav)
    y_nav_interp = np.interp(t_common, t_nav, y_nav)
    z_nav_interp = np.interp(t_common, t_nav, z_nav)
    roll_nav_interp = np.interp(t_common, t_nav, roll_nav)
    pitch_nav_interp = np.interp(t_common, t_nav, pitch_nav)
    yaw_nav_interp = np.interp(t_common, t_nav, yaw_nav)
    x_mocap_interp = np.interp(t_common, t_mocap, x_mocap)
    y_mocap_interp = np.interp(t_common, t_mocap, y_mocap)
    z_mocap_interp = np.interp(t_common, t_mocap, z_mocap)
    roll_mocap_interp = np.interp(t_common, t_mocap, roll_mocap)
    pitch_mocap_interp = np.interp(t_common, t_mocap, pitch_mocap)
    yaw_mocap_interp = np.interp(t_common, t_mocap, yaw_mocap)

    # RMSE計算
    rmse_x = compute_rmse(x_nav_interp, x_mocap_interp)
    rmse_y = compute_rmse(y_nav_interp, y_mocap_interp)
    rmse_z = compute_rmse(z_nav_interp, z_mocap_interp)
    rmse_roll = compute_rmse(roll_nav_interp, roll_mocap_interp)
    rmse_pitch = compute_rmse(pitch_nav_interp, pitch_mocap_interp)
    rmse_yaw = compute_rmse(yaw_nav_interp, yaw_mocap_interp)
    # rmse_total = np.sqrt(np.mean((x_nav_interp - x_mocap_interp)**2 + (y_nav_interp - y_mocap_interp)**2 + (z_nav_interp - z_mocap_interp)**2))

    print(f"RMSE X: {rmse_x:.4f}")
    print(f"RMSE Y: {rmse_y:.4f}")
    print(f"RMSE Z: {rmse_z:.4f}")
    print(f"RMSE ROLL: {rmse_roll:.4f}")
    print(f"RMSE PITCH: {rmse_pitch:.4f}")
    print(f"RMSE YAW: {rmse_yaw:.4f}")
    # print(f"RMSE Total (3D): {rmse_total:.4f}")

    # plot
    plt.rcParams['font.size'] = 28
    # Robot 
    plt.figure(figsize=(15, 15))
    plt.plot(x_nav, y_nav, 'b-', label='target')
    plt.plot(x_mocap, y_mocap, 'r-', label='measured')
    if args.pos_or_vel == 'vel':
      plt.plot([0.25,0.25], [0.50,2.0], 'g-', linewidth=5) # obstacle
    elif args.pos_or_vel == 'pos':
      plt.plot([1.57,1.81], [0.60,0.60], 'g-', linewidth=5) # wall
      plt.plot([1.81,1.81], [0.60,1.50], 'g-', linewidth=5)
      plt.plot([1.81,1.57], [1.50,1.50], 'g-', linewidth=5)
      plt.plot([1.57,1.57], [1.50,0.60], 'g-', linewidth=5)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    # plt.title('Robot Trajectory Comparison')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()
    '''
    plt.figure(figsize=(15, 15))
    plt.plot(x_nav, z_nav, 'b-', label='target')
    plt.plot(x_mocap, z_mocap, 'r-', label='measured')
    # if args.pos_or_vel == 'vel':
      # plt.plot([0.25,0.25], [0.0,1.0], 'g-', linewidth=5) # obstacle
    # elif args.pos_or_vel == 'pos':
      # plt.plot([1.5,1.5], [0.75,1.525], 'g-', linewidth=5) # wall
    plt.xlabel('X [m]')
    plt.ylabel('Z [m]')
    plt.title('Robot Trajectory Comparison')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

    plt.figure(figsize=(15, 15))
    plt.plot(y_nav, z_nav, 'b-', label='target')
    plt.plot(y_mocap, z_mocap, 'r-', label='measured')
    plt.xlabel('Y [m]')
    plt.ylabel('Z [m]')
    plt.title('Robot Trajectory Comparison')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()
    

    if args.pos_or_vel == 'pos':
      plt.figure(figsize=(15, 8))
      plt.plot(t_nav, x_nav, 'b-', label='target')
      plt.plot(t_mocap, x_mocap, 'r-', label='measured')
      plt.ylabel('X [m]')
      plt.xlabel('time [s]')
      # plt.title('Robot Trajectory Comparison')
      plt.legend()
      plt.grid(True)
      plt.show()

      plt.figure(figsize=(15, 8))
      plt.plot(t_nav, y_nav, 'b-', label='target')
      plt.plot(t_mocap, y_mocap, 'r-', label='measured')
      plt.ylabel('Y [m]')
      plt.xlabel('time [s]')
      # plt.title('Robot Trajectory Comparison')
      plt.legend()
      plt.grid(True)
      plt.show()

      plt.figure(figsize=(15, 8))
      plt.plot(t_nav, z_nav, 'b-', label='target')
      plt.plot(t_mocap, z_mocap, 'r-', label='measured')
      plt.ylabel('Z [m]')
      plt.xlabel('time [s]')
      # plt.title('Robot Trajectory Comparison')
      plt.legend()
      plt.grid(True)
      plt.show()

      plt.figure(figsize=(15, 8))
      plt.plot(t_nav, roll_nav, 'b-', label='target')
      plt.plot(t_mocap, roll_mocap, 'r-', label='measured')
      plt.ylabel('ROLL [rad]')
      plt.xlabel('time [s]')
      # plt.title('Robot Trajectory Comparison')
      plt.legend()
      plt.grid(True)
      plt.show()

      plt.figure(figsize=(15, 8))
      plt.plot(t_nav, pitch_nav, 'b-', label='target')
      plt.plot(t_mocap, pitch_mocap, 'r-', label='measured')
      plt.ylabel('PITCH [rad]')
      plt.xlabel('time [s]')
      # plt.title('Robot Trajectory Comparison')
      plt.legend()
      plt.grid(True)
      plt.show()

      plt.figure(figsize=(15, 8))
      plt.plot(t_nav, yaw_nav, 'b-', label='target')
      plt.plot(t_mocap, yaw_mocap, 'r-', label='measured')
      plt.ylabel('YAW [rad]')
      plt.xlabel('time [s]')
      # plt.title('Robot Trajectory Comparison')
      plt.legend()
      plt.grid(True)
      plt.show()
    '''
    # device
    plt.figure(figsize=(15, 15))
    plt.plot(x_device_mocap, y_device_mocap, 'r-')
    if args.pos_or_vel == 'vel': # draw stop zone
      cx,cy = device_init_pos[0], device_init_pos[1]
      r = 1/3
      square = plt.Rectangle((cx-r, cy-r), r*2, r*2, color='gray', alpha=0.3, zorder=0)
      plt.gca().add_patch(square)
      plt.scatter(cx, cy, color='black', s=30, marker='x')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    # plt.title('Device Trajectory')
    # plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

    if args.pos_or_vel == 'pos':
      '''
      plt.figure(figsize=(15, 8))
      plt.plot(t_wrench_fb, x_wrench_fb, 'r-', label='feedback_x')
      plt.plot(t_wrench_fb, y_wrench_fb, 'g-', label='feedback_y')
      plt.plot(t_wrench_fb, z_wrench_fb, 'b-', label='feedback_z')
      plt.xlabel('Time [s]')
      plt.ylabel('Force [N]')
      # plt.title('Wrench Compare X')
      plt.legend()
      plt.grid(True)
      # y_lim = dynamic_ylim(x_wrench_fb, x_wrench_cfs)
      # plt.ylim(y_lim[0],y_lim[1])
      plt.tight_layout()
      plt.show()

      plt.figure(figsize=(15, 8))
      plt.plot(t_wrench_fb, roll_wrench_fb, 'r-', label='feedback_roll')
      plt.plot(t_wrench_fb, pitch_wrench_fb, 'g-', label='feedback_pitch')
      plt.plot(t_wrench_fb, yaw_wrench_fb, 'b-', label='feedback_yaw')
      plt.xlabel('Time [s]')
      plt.ylabel('Torque [Nm]')
      # plt.title('Wrench Compare ROLL')
      plt.legend()
      plt.grid(True)
      # y_lim = dynamic_ylim(roll_wrench_fb, roll_wrench_cfs)
      # plt.ylim(y_lim[0],y_lim[1])
      plt.tight_layout()
      plt.show()
      '''

      plt.figure(figsize=(15, 8))
      plt.plot(t_wrench_cfs, x_wrench_cfs, 'r-', label='measured_x')
      plt.plot(t_wrench_cfs, y_wrench_cfs, 'g-', label='measured_y')
      plt.plot(t_wrench_cfs, z_wrench_cfs, 'b-', label='measured_z')
      plt.xlabel('Time [s]')
      plt.ylabel('Force [N]')
      # plt.title('Wrench Compare X')
      plt.legend()
      plt.grid(True)
      # y_lim = dynamic_ylim(x_wrench_fb, x_wrench_cfs)
      plt.ylim(-9,9)
      plt.tight_layout()
      plt.show()

      plt.figure(figsize=(15, 8))
      plt.plot(t_wrench_cfs, roll_wrench_cfs, 'r-', label='measured_roll')
      plt.plot(t_wrench_cfs, pitch_wrench_cfs, 'g-', label='measured_pitch')
      plt.plot(t_wrench_cfs, yaw_wrench_cfs, 'b-', label='measured_yaw')
      plt.xlabel('Time [s]')
      plt.ylabel('Torque [Nm]')
      # plt.title('Wrench Compare ROLL')
      plt.legend()
      plt.grid(True)
      # y_lim = dynamic_ylim(x_wrench_fb, x_wrench_cfs)
      plt.ylim(-0.25,0.25)
      plt.tight_layout()
      plt.show()

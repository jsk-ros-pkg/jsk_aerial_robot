#!/usr/bin/env python3
"""Extract reproducible hover windows and full 6-D mocap disturbances from ROS1 bags."""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation


def analyze(path, out, seconds=10.):
    poses, states, nav = [], [], []
    with rosbag.Bag(str(path)) as bag:
        start = bag.get_start_time()
        topics = bag.get_type_and_topic_info().topics
        candidates = [t for t in topics if t.endswith('/mocap/pose')]
        if len(candidates) != 1:
            raise ValueError('Expected one mocap topic: ' + str(candidates))
        topic = candidates[0]
        ns = topic[:-len('/mocap/pose')]
        for t, m, stamp in bag.read_messages(topics=[topic, ns+'/flight_state', ns+'/uav/nav']):
            ts = stamp.to_sec() - start
            if t == topic:
                p, q = m.pose.position, m.pose.orientation
                poses.append([ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w, m.header.stamp.to_sec()])
            elif t.endswith('flight_state'):
                if not states or states[-1][1] != m.data:
                    states.append([ts, m.data])
            else:
                nav.append([ts, m.target_pos_x, m.target_pos_y, m.target_pos_z, m.target_yaw])
    data = np.array(poses)
    hover = np.zeros(len(data), dtype=bool)
    for i, (t, state) in enumerate(states):
        end = states[i+1][0] if i+1 < len(states) else data[-1, 0]
        if state == 5:
            hover |= (data[:, 0] >= t+3.) & (data[:, 0] < end-3.)
    # Reject windows around commanded navigation changes (including settling).
    for t, *unused in nav:
        hover &= ~((data[:, 0] >= t-1.) & (data[:, 0] <= t+4.))
    windows = []
    for t in np.arange(data[0, 0], data[-1, 0]-seconds, 1.):
        mask = (data[:, 0] >= t) & (data[:, 0] < t+seconds)
        seg = data[mask]
        if len(seg) < seconds*30 or not hover[mask].all() or np.max(np.diff(seg[:, 0])) > 0.25:
            continue
        residual = seg[:, 1:4]-np.mean(seg[:, 1:4], axis=0)
        windows.append((float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))), t, seg))
    if not windows:
        summary = dict(bag=str(path), topic=topic, status='no_valid_hover_window',
                       flight_states=states, navigation=nav,
                       reason='No uninterrupted %.1fs HOVER_STATE window. Takeoff/landing and ground motion must not be treated as hover sway.' % seconds)
        (out / (path.stem+'_summary.json')).write_text(json.dumps(summary, indent=2)+'\n')
        np.savez_compressed(out/(path.stem+'_full_mocap.npz'), pose=data)
        print(json.dumps(summary, indent=2), flush=True)
        return
    windows.sort(key=lambda w: w[0])
    # Median eligible window avoids cherry-picking the best/worst hover.
    _, t, seg = windows[len(windows)//2]
    pos = seg[:, 1:4] - np.mean(seg[:, 1:4], axis=0)
    rotations = Rotation.from_quat(seg[:, 4:8])
    reference = rotations.mean()
    rot = (reference.inv() * rotations).as_rotvec()
    residual = np.column_stack((pos, rot))
    sample_time = seg[:, 8] - seg[0, 8]
    if np.any(np.diff(sample_time) <= 0):
        raise ValueError('Mocap header timestamps must be strictly increasing')
    np.savez_compressed(out / (path.stem+'_sway.npz'), time=sample_time, residual=residual,
                        pose=seg, mean_quaternion=reference.as_quat())
    summary = dict(bag=str(path), topic=topic, time_basis='window selection: bag receipt time; waveform: mocap header time',
                   window_start_s=float(t), window_duration_s=float(sample_time[-1]),
                   selection='median position RMS among %.1fs hover windows; 3s flight-state margins; nav [-1,+4]s excluded' % seconds,
                   eligible_windows=len(windows), flight_states=states, navigation=nav,
                   rate_hz=float(1/np.median(np.diff(sample_time))),
                   max_header_gap_s=float(np.max(np.diff(sample_time))),
                   max_receipt_gap_s=float(np.max(np.diff(seg[:, 0]))),
                   axis_names=['x_m','y_m','z_m','rx_rad','ry_rad','rz_rad'],
                   rms=np.sqrt(np.mean(residual**2, axis=0)).tolist(),
                   abs_p95=np.percentile(np.abs(residual), 95, axis=0).tolist(),
                   peak_to_peak=np.ptp(residual, axis=0).tolist(),
                   position_norm_rms_m=float(np.sqrt(np.mean(np.sum(pos**2, axis=1)))),
                   rotation_norm_rms_deg=float(np.rad2deg(np.sqrt(np.mean(np.sum(rot**2, axis=1))))))
    (out / (path.stem+'_summary.json')).write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2), flush=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(sample_time, pos*1000); axes[0].set_ylabel('position residual [mm]')
    axes[1].plot(sample_time, np.rad2deg(rot)); axes[1].set_ylabel('rotation vector [deg]')
    for ax in axes:
        ax.legend(['x','y','z']); ax.grid(True)
    axes[1].set_xlabel('time [s]'); fig.suptitle(path.stem + ' / median hover window')
    fig.tight_layout(); fig.savefig(out/(path.stem+'_sway.png')); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=10.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for path in args.bags:
        analyze(path, args.output, args.seconds)

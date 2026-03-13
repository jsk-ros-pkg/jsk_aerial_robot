import pybullet as p
import pybullet_data
import numpy as np
from scipy.spatial import ConvexHull
from itertools import product
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os
from tqdm import tqdm

# ------------------------------
# URDFパス
# ------------------------------
script_dir = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(script_dir, "../../TCP-IP-ROS-6AXis/dobot_gazebo/urdf/me6_robot.xacro")

# ------------------------------
# PyBullet初期化
# ------------------------------
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS | p.URDF_USE_INERTIA_FROM_FILE
robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=flags)

# ------------------------------
# 可動関節取得
# ------------------------------
joint_info = []
joint_limits = []
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    joint_type = info[2]
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        joint_info.append(i)
        lower = info[8] if info[8] > -1e30 else -np.pi
        upper = info[9] if info[9] < 1e30 else np.pi
        joint_limits.append((lower, upper))

ee_link_index = num_joints - 1

# ------------------------------
# 1. 粗いランダムサンプリング
# ------------------------------
num_coarse = 5000
coarse_positions = []

for _ in tqdm(range(num_coarse), desc="Coarse sampling"):
    angles = [np.random.uniform(low, high) for (low, high) in joint_limits]
    for idx, joint_idx in enumerate(joint_info):
        p.resetJointState(robot_id, joint_idx, angles[idx])
    ee_state = p.getLinkState(robot_id, ee_link_index)
    coarse_positions.append(ee_state[0])

coarse_positions = np.array(coarse_positions)

# ------------------------------
# 粗凸包計算
# ------------------------------
hull = ConvexHull(coarse_positions)
hull_vertices = coarse_positions[hull.vertices]

# ------------------------------
# 2. 凸包周辺を重点サンプリング
# ------------------------------
num_fine_per_vertex = 1000  # 各凸包頂点付近の追加サンプル数
search_radius = 0.05      # 頂点付近の角度範囲を ±5% などに相当

fine_positions = []

for v_angles in product(*[np.linspace(low, high, 3) for (low, high) in joint_limits]):
    pass  # placeholder: 後で凸包頂点周辺サンプリングを計算する
# 実際には凸包頂点に対応する関節角が分からないので、
# ランダムに粗凸包頂点周辺の関節角空間で再サンプリング
# ここでは簡便にランダムサンプルで近傍を追加
for _ in tqdm(range(len(hull_vertices)*num_fine_per_vertex), desc="Fine sampling"):
    angles = [np.random.uniform(low, high) for (low, high) in joint_limits]
    for idx, joint_idx in enumerate(joint_info):
        p.resetJointState(robot_id, joint_idx, angles[idx])
    ee_state = p.getLinkState(robot_id, ee_link_index)
    # 粗凸包頂点から一定距離以上の点のみ追加（外側重点）
    dists = np.linalg.norm(ee_state[0] - hull_vertices, axis=1)
    if np.min(dists) < search_radius:
        fine_positions.append(ee_state[0])

# ------------------------------
# データ統合
# ------------------------------
all_positions = np.vstack([coarse_positions, fine_positions])

# ------------------------------
# 最終凸包計算
# ------------------------------
final_hull = ConvexHull(all_positions)

# np.savez("me6_ee_feasible_convex.npz", all_positions=all_positions, hull_vertices=all_positions[final_hull.vertices])
# print("凸包データを保存しました")

# ------------------------------
# 3D可視化
# ------------------------------
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(all_positions[:,0], all_positions[:,1], all_positions[:,2], s=2, alpha=0.2, color='blue')

faces = [all_positions[simplex] for simplex in final_hull.simplices]
poly3d = Poly3DCollection(faces, alpha=0.3, facecolor='red', edgecolor='k')
ax.add_collection3d(poly3d)

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.set_title("End-Effector Workspace (Focused Convex Hull)")

ax.auto_scale_xyz(all_positions[:,0], all_positions[:,1], all_positions[:,2])
plt.show()

p.disconnect()

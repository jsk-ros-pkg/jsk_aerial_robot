#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import sys
import pybullet as p
import pybullet_data
import numpy as np
import subprocess
import tempfile

# =============================
# 設定
# =============================
NUM_SAMPLES = 500000          # サンプリング数（必要に応じて変更）
OUTPUT_FILE = "me6_unsafe_configs.npy"

# xacro / urdf のパス
script_dir = os.path.dirname(os.path.realpath(__file__))
xacro_path = os.path.join(script_dir, "../../TCP-IP-ROS-6AXis/dobot_gazebo/urdf/me6_robot.xacro")
urdf_path  = os.path.join(script_dir, "me6_robot_converted.urdf")

# STL 実体のあるディレクトリ（package:// をここに置き換える）
mesh_root = os.path.join(script_dir, "../../TCP-IP-ROS-6AXis/dobot_description/meshes/me6/")

# “地面らしい” リンク名の候補（URDF内に存在すれば地面ボディとして扱う）
GROUND_NAME_CANDIDATES = ["ground", "floor", "plane", "table", "world"]


def convert_xacro_to_urdf(xacro_in: str, urdf_out: str):
    print("[INFO] Converting xacro to urdf...")
    subprocess.run(["xacro", xacro_in, "-o", urdf_out], check=True)

    print("[INFO] Rewriting mesh paths in URDF...")
    with open(urdf_out, "r", encoding="utf-8") as f:
        urdf_text = f.read()

    # package://dobot_description/meshes/me6/ を絶対パスに置換
    urdf_text = urdf_text.replace(
        "package://dobot_description/meshes/me6/",
        os.path.join(mesh_root)
    )

    with open(urdf_out, "w", encoding="utf-8") as f:
        f.write(urdf_text)


def find_ground_links_in_urdf(urdf_file: str):
    """URDF から地面っぽいリンク名を抽出（文字列マッチ）。
       該当がなければ空リストを返す。
    """
    with open(urdf_file, "r", encoding="utf-8") as f:
        text = f.read()

    # <link name="..."> を抽出
    link_names = re.findall(r'<\s*link\s+name\s*=\s*"([^"]+)"', text)
    link_names = [ln.strip() for ln in link_names]

    # ground/floor/plane/table/world などを含むリンクを拾う
    ground_like = []
    for ln in link_names:
        low = ln.lower()
        if any(key in low for key in GROUND_NAME_CANDIDATES):
            ground_like.append(ln)

    return ground_like


def main():
    # 1) xacro → urdf 変換＆メッシュパス修正
    convert_xacro_to_urdf(xacro_path, urdf_path)

    # 2) URDF から ground らしいリンク名を探す
    ground_like_links = find_ground_links_in_urdf(urdf_path)
    if ground_like_links:
        print(f"[INFO] Found ground-like links in URDF: {ground_like_links}")
    else:
        print("[INFO] No ground-like links found in URDF. Will add plane.urdf at z=0.")

    # 3) PyBullet 初期化
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # 4) ロボット読み込み（自己衝突フラグを有効化）
    flags = (
        p.URDF_USE_SELF_COLLISION
        | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        | p.URDF_USE_INERTIA_FROM_FILE
    )
    print("[INFO] Loading URDF into PyBullet...")
    robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=flags)
    num_joints = p.getNumJoints(robot_id)
    print(f"[INFO] URDF loaded! robot_id={robot_id}, num_joints={num_joints}")

    # 5) “地面”の扱い
    #   - URDF に ground-like なリンクが含まれていても、同一ボディ(robot_id)内のリンクだと
    #     PyBullet の getContactPoints では self-collision の一部として返る可能性があります。
    #   - 確実に地面との衝突を拾いたいので、**別ボディ**として plane.urdf を z=0 に追加します。
    #   - これで bodyB==ground_id の接触を地面衝突として判定できます。
    ground_id = p.loadURDF("plane.urdf", basePosition=[0, 0, 0])
    print(f"[INFO] Ground plane body loaded. ground_id={ground_id}")

    # 6) 可動関節の index と上下限
    joint_indices = []
    joint_limits = []  # [(lower, upper), ...]
    for j in range(num_joints):
        ji = p.getJointInfo(robot_id, j)
        jtype = ji[2]
        if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            lo, hi = ji[8], ji[9]
            if not np.isfinite(lo) or not np.isfinite(hi) or lo > hi:
                lo, hi = -np.pi, np.pi  # 無限回転などをおおまかに扱う
            joint_indices.append(j)
            joint_limits.append((lo, hi))

    joint_limits = np.array(joint_limits, dtype=float)
    dof = len(joint_indices)
    print(f"[INFO] Controllable joints: {dof}")

    # 7) サンプリング＆衝突チェック（自己衝突＋地面）
    unsafe_configs = []

    # 衝突チェックの関数を用意：自己衝突 or ground 衝突で True
    def is_unsafe():
        # a) 自己衝突（ロボット内リンク同士）
        #    robot_id 内の接触（self-collision）→ bodyA==robot_id, bodyB==robot_id
        for c in p.getContactPoints(bodyA=robot_id, bodyB=robot_id):
            # もし親子リンク間の常時接触を除外したいなら、ここで linkIndex の組をフィルタ
            return True

        # b) 地面との接触（ロボット vs ground_id）
        if len(p.getContactPoints(bodyA=robot_id, bodyB=ground_id)) > 0:
            return True

        return False

    rng = np.random.default_rng()
    for i in range(NUM_SAMPLES):
        # ランダムに関節角を生成
        q = rng.uniform(joint_limits[:, 0], joint_limits[:, 1])

        # 関節状態を直接セット（高速・安定）
        for qi, ji in zip(q, joint_indices):
            p.resetJointState(robot_id, ji, float(qi))

        # 衝突判定（ステップを進めなくても OK だが、明示的に実行しておく）
        p.performCollisionDetection()

        if is_unsafe():
            unsafe_configs.append(q)

        # 進捗表示（必要なら）
        if (i + 1) % 10000 == 0:
            print(f"[INFO] {i+1}/{NUM_SAMPLES} checked, unsafe so far: {len(unsafe_configs)}")

    # 8) 保存
    unsafe_configs = np.array(unsafe_configs, dtype=float)
    out_path = os.path.join(script_dir, OUTPUT_FILE)
    np.save(out_path, unsafe_configs)
    print(f"[INFO] Saved {out_path} with shape {unsafe_configs.shape}")

    p.disconnect()


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        print("[ERROR] xacro conversion failed. Is 'xacro' installed and in PATH?", file=sys.stderr)
        raise

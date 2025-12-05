#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
2つの wrench ログ CSV (time, Q_**, Fx..Tz) を読み込んで、
レンチと Q を重ねてプロットするスクリプト。

使い方:
    python plot_two_wrench_q_logs.py log1.csv log2.csv

前提:
    ログは compute_wrench_and_save_csv.py が出力した形式:
        time, Q_00..Q_(5, N-1), Fx, Fy, Fz, Tx, Ty, Tz
    （Q は 6×N を row-major でならべたもの）
"""

import sys
import csv
import os
import numpy as np
import matplotlib.pyplot as plt

WRENCH_NAMES = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
AXES = ["x", "y", "z", "roll", "pitch", "yaw"]


def load_log(path):
    """CSV を読み込んで (time, wrench(= Nx6), Q(= N x 6 x rotor_num)) を返す"""
    times = []
    rows = []

    with open(path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)  # 1行目はヘッダ

        for line_no, row in enumerate(reader, start=2):
            if not row:
                continue
            try:
                vals = [float(x) for x in row]
            except ValueError:
                print(f"[WARN] {path}:{line_no} に数値でない値があります。スキップします。")
                continue

            times.append(vals[0])
            rows.append(vals)

    if not rows:
        raise RuntimeError(f"{path} に有効なデータ行がありません。")

    data = np.array(rows, dtype=float)  # shape=(N, n_cols)
    time = data[:, 0]
    n_cols = data.shape[1]

    # 最後の6列が Fx..Tz
    wrench = data[:, -6:]  # shape=(N, 6)

    # 残りが Q: 1列目から (n_cols-1-6) 列ぶん
    n_q = n_cols - 1 - 6
    if n_q % 6 != 0:
        raise RuntimeError(
            f"{path}: Qの列数 n_q={n_q} が 6 の倍数ではありません。"
        )
    rotor_num = n_q // 6

    q_flat = data[:, 1:1 + 6 * rotor_num]  # shape=(N, 6*rotor_num)
    Q = q_flat.reshape((-1, 6, rotor_num))  # shape=(N, 6, rotor_num)

    return time, wrench, Q


def main():
    if len(sys.argv) != 3:
        print("使い方: python plot_two_wrench_q_logs.py log1.csv log2.csv")
        sys.exit(1)

    path1, path2 = sys.argv[1], sys.argv[2]

    print(f"Loading {path1} ...")
    t1, w1, Q1 = load_log(path1)
    print(f"  -> {len(t1)} samples, rotor_num={Q1.shape[2]}")

    print(f"Loading {path2} ...")
    t2, w2, Q2 = load_log(path2)
    print(f"  -> {len(t2)} samples, rotor_num={Q2.shape[2]}")

    # rotor_num が違ったら怒る
    if Q1.shape[2] != Q2.shape[2]:
        raise RuntimeError(
            f"rotor_num mismatch: {path1}={Q1.shape[2]}, {path2}={Q2.shape[2]}"
        )
    rotor_num = Q1.shape[2]

    # 時間軸はそれぞれ先頭を 0 にそろえる
    t1_rel = t1 - t1[0]
    t2_rel = t2 - t2[0]

    base1 = os.path.basename(path1)
    base2 = os.path.basename(path2)

    # ---------- 図1: Wrench (Fx..Tz) ----------
    fig_w, axes_w = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    fig_w.suptitle(f"Wrench comparison: {base1} vs {base2}")

    for i, name in enumerate(WRENCH_NAMES):
        ax = axes_w[i]
        ax.plot(t1_rel, w1[:, i], label=base1)
        ax.plot(t2_rel, w2[:, i], label=base2, linestyle="--")
        ax.set_ylabel(name)
        ax.grid(True)
        ax.legend(loc="best", fontsize=8)

    axes_w[-1].set_xlabel("time [s] (relative)")

    # ---------- 図2: Q の各行（各軸） ----------
    fig_q, axes_q = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    fig_q.suptitle(f"Q rows per axis, per rotor: {base1} vs {base2}")

    for axis_idx, axis_name in enumerate(AXES):
        ax = axes_q[axis_idx]

        for col in range(rotor_num):
            label1 = f"{axis_name}, r{col+1} ({base1})"
            label2 = f"{axis_name}, r{col+1} ({base2})"

            ax.plot(
                t1_rel, Q1[:, axis_idx, col],
                label=label1
            )
            ax.plot(
                t2_rel, Q2[:, axis_idx, col],
                label=label2,
                linestyle="--"
            )

        ax.set_ylabel(axis_name)
        ax.grid(True)
        # ラベルが多いので最初の行だけ legend 表示（他は同じ）
        if axis_idx == 0:
            ax.legend(loc="best", fontsize=7, ncol=2)

    axes_q[-1].set_xlabel("time [s] (relative)")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

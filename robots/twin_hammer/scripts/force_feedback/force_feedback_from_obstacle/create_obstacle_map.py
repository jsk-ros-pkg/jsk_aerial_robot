#!/usr/bin/env python3
"""
Developer writes wall coordinates directly in the script.
Generates 3D map, saves JSON, and draws walls.
"""

import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

WALLS = [
    # boundary1
    [
        [-2, -2.0, 0],
        [-2, 2, 0],
        [-2, 2, 2],
        [-2, -2.0, 2],
    ],
    # boundary2
    [
        [-2, -2.0, 0],
        [2, -2.0, 0],
        [2, -2.0, 2],
        [-2, -2.0, 2],
    ],
    # boundary3
    [
        [-2, -2.0, 0],
        [-2, 2, 0],
        [2, 2, 0],
        [2, -2.0, 0],
    ],
    # boundary4
    [
        [-2, -2.0, 2],
        [-2, 2, 2],
        [2, 2, 2],
        [2, -2.0, 2],
    ],
    # boundary5
    [
        [-2, 2, 0],
        [2, 2, 0],
        [2, 2, 2],
        [-2, 2, 2],
    ],
    # boundary6
    [
        [2, -2.0, 0],
        [2, 2, 0],
        [2, 2, 2],
        [2, -2.0, 2],
    ],

    # wall 1
    [
        [-1.5, -1.5, 0],
        [1.5, -1.5, 0],
        [1.5, -1.5, 1.5],
        [-1.5, -1.5, 1.5],
    ],

    # wall 2
    [
        [1.5, -1.5, 0],
        [1.5, 1.5, 0],
        [1.5, 1.5, 1.5],
        [1.5, -1.5, 1.5],
    ],

    # wall 3
    [
        [-1.5, 0, 0],
        [0, 0, 0],
        [0, 0, 1.5],
        [-1.5, 0, 1.5],
    ],

    # wall 4
    [
        [0, 0, 0],
        [0, 1.5, 0],
        [0, 1.5, 1.5],
        [0, 0, 1.5],
    ]
]

# ==========================================================


def compute_bbox(walls):
    pts = [p for wall in walls for p in wall]
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
    return {
        "xmin": min(xs), "xmax": max(xs),
        "ymin": min(ys), "ymax": max(ys),
        "zmin": min(zs), "zmax": max(zs),
    }


def draw_walls(walls):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    for quad in walls:
        poly = Poly3DCollection([quad], alpha=0.5, edgecolor='k')
        ax.add_collection3d(poly)

    pts = [p for wall in walls for p in wall]
    xs = [p[0] for p in pts]; ys=[p[1] for p in pts]; zs=[p[2] for p in pts]

    ax.set_xlim(min(xs), max(xs))
    ax.set_ylim(min(ys), max(ys))
    ax.set_zlim(min(zs), max(zs))

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.title("3D Map (Walls)")

    plt.show()


def main():
    out_file = "wall_map.json"

    # JSON 形式に整形
    data = {
        "walls": [{"corners": w} for w in WALLS],
        "bbox": compute_bbox(WALLS)
    }

    # 保存
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"Saved map to {out_file}")

    # 描画
    draw_walls(WALLS)


if __name__ == "__main__":
    main()

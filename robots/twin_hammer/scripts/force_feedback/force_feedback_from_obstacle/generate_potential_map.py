#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import os

# -------------------------
# User parameters
# -------------------------
MAP_JSON_PATH = "wall_map.json"        # input
OUT_NPZ = "sdf_potential.npz"          # output
OUT_META = "sdf_potential_meta.json"

GRID_RESOLUTION = 0.01       # [m] voxel resolution
SURFACE_SAMPLE_RES = 0.006   # [m] sampling resolution on wall quads
BBOX_PADDING = 0.0           # [m] bbox padding

# potential parameters
# choose DECAY_MODE: "wendland", or "exp", "quad", "inv", "exp2", "rational", "powerlaw"
DECAY_MODE = "rational"
MAX_POTENTIAL = 1.0
CUTOFF_DISTANCE = 2.0   # R: radius for kernel (meters). For wendland, potential=0 for d>=R.
POT_EPS = 0.01
POWER_P = 2

# visualization
VIS_Z = None             # z coordinate for slice visualization; None => center slice
SAVE_VIS_PNG = False
VIS_PNG_PATH = "sdf_potential_slice.png"

VERBOSE = True

# -------------------------
# Utilities
# -------------------------
def load_walls(path):
    with open(path, "r", encoding="utf-8") as f:
        mp = json.load(f)
    walls = mp.get("walls", [])
    if not walls:
        raise RuntimeError("No walls found in map JSON.")
    for w in walls:
        if "corners" not in w or len(w["corners"]) != 4:
            raise RuntimeError("Each wall must have 4 corners.")
    bbox = mp.get("bbox", None)
    return walls, bbox

def compute_bbox_from_walls(walls):
    pts = [pt for w in walls for pt in w["corners"]]
    arr = np.array(pts, dtype=float)
    xmin, ymin, zmin = np.min(arr, axis=0)
    xmax, ymax, zmax = np.max(arr, axis=0)
    return {"xmin": float(xmin), "xmax": float(xmax),
            "ymin": float(ymin), "ymax": float(ymax),
            "zmin": float(zmin), "zmax": float(zmax)}

# -------------------------
# Rasterization
# -------------------------
def sample_quad(corners, sample_res):
    p0 = np.array(corners[0], dtype=float)
    p1 = np.array(corners[1], dtype=float)
    p2 = np.array(corners[2], dtype=float)
    p3 = np.array(corners[3], dtype=float)

    v1 = p1 - p0
    v2 = p3 - p0

    len1 = np.linalg.norm(v1)
    len2 = np.linalg.norm(v2)

    n1 = max(2, int(math.ceil(len1 / sample_res)) + 1)
    n2 = max(2, int(math.ceil(len2 / sample_res)) + 1)

    u = np.linspace(0.0, 1.0, n1)
    v = np.linspace(0.0, 1.0, n2)
    uu, vv = np.meshgrid(u, v, indexing="xy")
    uu = uu.flatten()
    vv = vv.flatten()

    pts = p0[None, :] + np.outer(uu, v1) + np.outer(vv, v2)
    return pts

def rasterize_walls_to_voxels(walls, bbox, res, sample_res, pad=0.0):
    xmin = bbox["xmin"] - pad
    xmax = bbox["xmax"] + pad
    ymin = bbox["ymin"] - pad
    ymax = bbox["ymax"] + pad
    zmin = bbox["zmin"] - pad
    zmax = bbox["zmax"] + pad

    xs = np.arange(xmin, xmax + 1e-12, res)
    ys = np.arange(ymin, ymax + 1e-12, res)
    zs = np.arange(zmin, zmax + 1e-12, res)
    nx, ny, nz = len(xs), len(ys), len(zs)

    if VERBOSE:
        print(f"[rasterize] grid dims = {nx} x {ny} x {nz} (total {nx*ny*nz})")

    occupancy = np.zeros((nx, ny, nz), dtype=bool)

    all_samples = []
    for i, w in enumerate(walls):
        pts = sample_quad(w["corners"], sample_res)
        all_samples.append(pts)
        if VERBOSE:
            print(f"[rasterize] wall {i}: sampled {pts.shape[0]} points")
    if len(all_samples) == 0:
        raise RuntimeError("No samples produced for walls.")
    samples = np.vstack(all_samples)

    inds = np.floor((samples - np.array([xmin, ymin, zmin])) / res).astype(int)
    inds[:, 0] = np.clip(inds[:, 0], 0, nx - 1)
    inds[:, 1] = np.clip(inds[:, 1], 0, ny - 1)
    inds[:, 2] = np.clip(inds[:, 2], 0, nz - 1)

    occupancy[inds[:, 0], inds[:, 1], inds[:, 2]] = True

    return occupancy, xs, ys, zs

# -------------------------
# Potential mapping functions
# -------------------------
def wendland_c2(d, R, maxpot=1.0):
    """
    Wendland C2 radial basis kernel:
      r = d/R
      phi(r) = (1 - r)^4 * (4r + 1)   for 0 <= r < 1
      phi(r) = 0                      for r >= 1
    returns potential values scaled by maxpot.
    """
    d = np.asarray(d, dtype=np.float32)
    R = float(R)
    if R <= 0:
        raise ValueError("R must be > 0")
    r = d / R
    pot = np.zeros_like(r, dtype=np.float32)
    inside = (r < 1.0)
    if np.any(inside):
        rin = r[inside]
        # compute (1-r)^4 * (4r + 1)
        w = (1.0 - rin)**4 * (4.0 * rin + 1.0)
        pot[inside] = maxpot * w
    return pot

def distance_to_potential(dist_array, maxpot, cutoff, mode="wendland",
                          eps=0.01, power_p=2):
    """
    Map distance -> potential. Supported modes:
      - "wendland" : Wendland C2 kernel (compact support)
      - "exp"      : simple exponential decay (existing)
      - "quad"     : quadratic decay
      - "inv"      : inverse decay
      - "exp2"     : NEW — exponential decay from func_test_potential.py
      - "rational" : NEW — rational decay
      - "powerlaw" : NEW — power-law decay
    eps  : potential(cutoff) = eps * maxpot となるようにパラメータを設定
    power_p : power-law の次数
    """

    d = np.asarray(dist_array, dtype=np.float32)

    # ---------------------------------------------------------
    # (1) Wendland C2 kernel  --- recommended
    # ---------------------------------------------------------
    if mode == "wendland":
        r = d / cutoff
        pot = np.zeros_like(r, dtype=np.float32)
        inside = (r < 1.0)
        if np.any(inside):
            rin = r[inside]
            w = (1.0 - rin)**4 * (4.0 * rin + 1.0)
            pot[inside] = maxpot * w
        return pot

    # ---------------------------------------------------------
    # (2) Simple quadratic tail (existing)
    # ---------------------------------------------------------
    if mode == "quad":
        R = float(cutoff)
        a = maxpot / (R**2)
        b = -2 * maxpot / R
        c = maxpot
        pot = a * d**2 + b * d + c
        pot[d >= R] = 0.0
        pot = np.maximum(pot, 0.0)
        return pot

    # ---------------------------------------------------------
    # (3) Inverse type (existing)
    # ---------------------------------------------------------
    if mode == "inv":
        s = cutoff * 0.1
        denom = (1.0 / s - 1.0 / (cutoff + s))
        if denom == 0:
            return np.zeros_like(d)
        a = maxpot / denom
        pot = a / (d + s) - a / (cutoff + s)
        pot[d >= cutoff] = 0.0
        return np.maximum(pot, 0.0)

    # ---------------------------------------------------------
    # (4) OLD exponential (existing)
    # ---------------------------------------------------------
    if mode == "exp":
        s = cutoff / 5.0
        return maxpot * np.exp(-d / s)

    # ---------------------------------------------------------
    # (5) NEW exponential decay: pot_exp()
    # ---------------------------------------------------------
    if mode == "exp2":
        k = np.log(1 / eps) / cutoff   # potential(cutoff) = eps * maxpot
        return maxpot * np.exp(-k * d)

    # ---------------------------------------------------------
    # (6) NEW rational decay: pot_rational()
    # ---------------------------------------------------------
    if mode == "rational":
        a = (1 / np.sqrt(eps) - 1) / cutoff
        return maxpot / (1 + a * d)

    # ---------------------------------------------------------
    # (7) NEW power-law decay: pot_powerlaw()
    # ---------------------------------------------------------
    if mode == "powerlaw":
        b = (eps**(-1/(power_p+1)) - 1) / cutoff
        return maxpot / (1 + b * d)**power_p

    # ---------------------------------------------------------
    raise ValueError("Unknown mode. Choose from wendland/exp/quad/inv/exp2/rational/powerlaw")

# -------------------------
# Visualization helpers
# -------------------------
def visualize_slice(potential, xs, ys, zs, vis_z=None, save_png=False, png_path="slice.png"):
    if vis_z is None:
        iz = len(zs) // 2
    else:
        iz = int(np.argmin(np.abs(zs - vis_z)))
    zval = float(zs[iz])
    if VERBOSE:
        print(f"[visualize] using z slice index {iz}, z = {zval:.4f}")

    pot2d = potential[:, :, iz].T
    fig, ax = plt.subplots(figsize=(7,6))
    im = ax.imshow(pot2d, origin="lower",
                   extent=[float(xs[0]), float(xs[-1]), float(ys[0]), float(ys[-1])],
                   aspect="equal")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title(f"Potential slice z={zval:.3f} mode={DECAY_MODE}")
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("Potential")
    if save_png:
        fig.savefig(png_path, dpi=200)
        if VERBOSE:
            print(f"[visualize] saved png {png_path}")
    plt.show()

def visualize_slice_tkinter(potential, xs, ys, zs):
    root = tk.Tk()
    root.title("Potential Slice Viewer")

    frame = ttk.Frame(root)
    frame.pack(padx=10, pady=10)

    fig, ax = plt.subplots(figsize=(6,5))
    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.get_tk_widget().grid(row=0, column=0)

    nz = potential.shape[2]

    def update_plot(z_index):
        z_index = int(float(z_index))
        ax.clear()
        slice_2d = potential[:, :, z_index].T
        extent = [xs[0], xs[-1], ys[0], ys[-1]]
        ax.imshow(slice_2d, extent=extent, origin="lower")
        ax.set_title(f"Z Slice = {zs[z_index]:.3f}")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        fig.colorbar(ax.images[0], ax=ax)
        canvas.draw()

    slider = tk.Scale(
        frame,
        from_=0,
        to=nz-1,
        orient=tk.HORIZONTAL,
        length=500,
        resolution=1,
        label="Z index",
        command=update_plot
    )
    slider.set(nz // 2)
    slider.grid(row=1, column=0, pady=10)
    update_plot(nz // 2)
    root.mainloop()

def visualize_walls_3d(walls, xs, ys, zs):
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection="3d")
    for w in walls:
        quad = w["corners"]
        poly = Poly3DCollection([quad], alpha=0.4, edgecolor="k")
        ax.add_collection3d(poly)
    ax.set_xlim(float(xs[0]), float(xs[-1]))
    ax.set_ylim(float(ys[0]), float(ys[-1]))
    ax.set_zlim(float(zs[0]), float(zs[-1]))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Walls (3D)")
    plt.show()

# -------------------------
# Main
# -------------------------
def main():
    if VERBOSE:
        print("[main] loading map:", MAP_JSON_PATH)
    walls, bbox = load_walls(MAP_JSON_PATH)
    if bbox is None:
        bbox = compute_bbox_from_walls(walls)
    else:
        bbox = {k: float(v) for k, v in bbox.items()}

    bbox["xmin"] -= BBOX_PADDING
    bbox["xmax"] += BBOX_PADDING
    bbox["ymin"] -= BBOX_PADDING
    bbox["ymax"] += BBOX_PADDING
    bbox["zmin"] -= BBOX_PADDING
    bbox["zmax"] += BBOX_PADDING

    if VERBOSE:
        print("[main] bbox:", bbox)
        print(f"[main] grid_res={GRID_RESOLUTION}, sample_res={SURFACE_SAMPLE_RES}")

    occupancy, xs, ys, zs = rasterize_walls_to_voxels(walls, bbox, GRID_RESOLUTION, SURFACE_SAMPLE_RES, pad=0.0)

    if VERBOSE:
        print("[main] computing distance transform (EDT)...")
    dist_vox = distance_transform_edt(~occupancy)  # distance in voxels to nearest occupied voxel
    dist_m = dist_vox * float(GRID_RESOLUTION)    # convert to meters

    if VERBOSE:
        print(f"[main] mapping distances -> potential (mode={DECAY_MODE})...")
    potential = distance_to_potential(dist_m, MAX_POTENTIAL, CUTOFF_DISTANCE, mode=DECAY_MODE, eps=POT_EPS, power_p=POWER_P)

    if VERBOSE:
        print(f"[main] saving .npz -> {OUT_NPZ} and meta -> {OUT_META}")
    np.savez_compressed(OUT_NPZ, potential=potential, xs=xs, ys=ys, zs=zs, distances=dist_m)

    meta = {
        "map_json": MAP_JSON_PATH,
        "out_npz": OUT_NPZ,
        "out_meta": OUT_META,
        "grid": {"nx": int(len(xs)), "ny": int(len(ys)), "nz": int(len(zs)), "res": float(GRID_RESOLUTION),
                 "xmin": float(xs[0]), "xmax": float(xs[-1]), "ymin": float(ys[0]), "ymax": float(ys[-1]),
                 "zmin": float(zs[0]), "zmax": float(zs[-1])},
        "sample_res": float(SURFACE_SAMPLE_RES),
        "decay_mode": DECAY_MODE,
        "max_potential": float(MAX_POTENTIAL),
        "cutoff_distance": float(CUTOFF_DISTANCE),
        "num_walls": len(walls)
    }
    with open(OUT_META, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    if VERBOSE:
        print("[main] visualizing slice...")
    visualize_slice(potential, xs, ys, zs, vis_z=VIS_Z, save_png=SAVE_VIS_PNG, png_path=VIS_PNG_PATH)
    if VERBOSE:
        visualize_walls_3d(walls, xs, ys, zs)

    if VERBOSE:
        print("[main] done. outputs:", OUT_NPZ, OUT_META)

if __name__ == "__main__":
    main()

"""
 Created by li-jinjie on 2026/3/1.
 Refactored on 2026/3/2 to support multi-configuration comparison.
"""

import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import scienceplots

from analyze_allocation import get_alloc_mtx_tilt_qd, get_alloc_mtx_tilt_tri, get_alloc_mtx_tilt_general
from analyze_allocation import get_cmd_w_inv_mat
from analyze_allocation import mass, gravity


def compute_hover_efficiency(num_rotor, roll_grid, yaw_grid, fg_w):
    """
    Compute hover efficiency map for a given rotor configuration.

    Parameters:
        num_rotor (int): Number of rotors (3, 4, 5, 6, etc.)
        roll_grid (np.ndarray): Array of roll angles in degrees
        yaw_grid (np.ndarray): Array of yaw angles in degrees
        fg_w (np.ndarray): Gravity force vector in world frame

    Returns:
        result_hover_thrust_map (np.ndarray): 2D array of total thrust required
    """
    # Get allocation matrix based on rotor configuration
    if num_rotor == 4:
        alloc_mat = get_alloc_mtx_tilt_qd()
    elif num_rotor == 3:
        alloc_mat = get_alloc_mtx_tilt_tri()
    else:
        alloc_mat = get_alloc_mtx_tilt_general(num_rotor)

    alloc_pinv = np.linalg.pinv(alloc_mat)

    result_hover_thrust_map = np.zeros((len(roll_grid), len(yaw_grid)))

    for i, roll in enumerate(roll_grid):
        for j, yaw in enumerate(yaw_grid):
            # Compute the rotation matrix for the given roll and yaw (assuming zero pitch)
            r = R.from_euler("zyx", [yaw, 0, roll], degrees=True)
            rot_mat = r.as_matrix()

            # Rotate the gravity force into the body frame
            fg_b = rot_mat.T @ fg_w

            tgt_wrench = np.array([[fg_b.item(0), fg_b.item(1), fg_b.item(2), 0, 0, 0]]).T

            # Compute reference values for thrust
            ft_ref, a_ref = get_cmd_w_inv_mat(alloc_pinv, tgt_wrench)

            # Compute the total thrust
            total_thrust = sum(ft_ref)

            # Store the total thrust in the result map
            result_hover_thrust_map[i, j] = total_thrust

    return result_hover_thrust_map


def annotate_keypoints(
    ax,
    data,
    roll_grid,
    yaw_grid,
    points,
    text_color="white",
    point_color="red",
    fontsize=12,
    point_size=15,
    bbox=None,
    value_fmt="{:.3f}",
    xytext=(6, 6),
):
    """
    Annotate selected keypoints on a heatmap.

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        Target axes.
    data : np.ndarray
        Heatmap data with shape (len(roll_grid), len(yaw_grid)),
        indexed as data[i_roll, j_yaw].
    roll_grid, yaw_grid : np.ndarray
        1D grids used to generate the heatmap.
    points : list[tuple]
        List of (roll_deg, yaw_deg, label_str_or_None).
        If label is None, an automatic label will be generated.
    text_color, point_color : str
        Text and marker colors.
    fontsize : int
        Font size for annotation text.
    point_size : float
        Marker size for the keypoint.
    bbox : dict or None
        Matplotlib bbox style dict for the text background.
    value_fmt : str
        Format string for the displayed value (e.g., "{:.4f}").
    xytext : tuple[int, int]
        Pixel offset of the text relative to the point.
    """
    if bbox is None:
        bbox = dict(boxstyle="round,pad=0.25", fc="black", ec="none", alpha=0.6)

    for roll_deg, yaw_deg, label in points:
        # Find nearest indices on the roll/yaw grids (robust to non-1deg resolution)
        i = int(np.argmin(np.abs(roll_grid - roll_deg)))
        j = int(np.argmin(np.abs(yaw_grid - yaw_deg)))

        val = float(data[i, j])

        # Draw the keypoint marker
        ax.scatter([yaw_grid[j]], [roll_grid[i]], s=point_size, c=point_color, marker="o", zorder=5)

        # Compose label text
        if label is None:
            label = f"r={roll_grid[i]:g}, y={yaw_grid[j]:g}\n{value_fmt.format(val)}"
        else:
            label = f"{label}\n{value_fmt.format(val)}"

        # Add annotation
        ax.annotate(
            label,
            (yaw_grid[j], roll_grid[i]),
            textcoords="offset points",
            xytext=xytext,
            ha="center",
            va="center",
            color=text_color,
            fontsize=fontsize,
            bbox=bbox,
            zorder=6,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Analyze hovering efficiency under different attitudes for multiple rotor configurations."
    )
    parser.add_argument(
        "--resolution",
        "-r",
        type=float,
        default=1,
        help="Angular resolution in degrees for the attitude grid. Covering 0 to 180 degrees in roll and yaw.",
    )
    parser.add_argument(
        "--show_thrust_or_ratio",
        "-s",
        choices=["thrust", "ratio"],
        default="ratio",
        help="Choose whether to show absolute thrust values ('thrust') or thrust-to-weight ratio ('ratio') in the plot.",
    )

    args = parser.parse_args()

    fg_w = np.array([0, 0, mass * gravity])

    # Create grids of yaw ∈ [-180°, +180°], roll ∈ [0°, 180°]
    YAW_LOW = -180
    YAW_HIGH = 180
    ROLL_LOW = 0
    ROLL_HIGH = 180
    yaw_grid = np.arange(YAW_LOW, YAW_HIGH + 1, args.resolution)
    roll_grid = np.arange(ROLL_LOW, ROLL_HIGH + 1, args.resolution)

    # Define configurations: (num_rotor, title, subplot_position)
    configurations = [(3, "Tri", (0, 0)), (4, "Quad", (0, 1)), (5, "Penta", (1, 0)), (6, "Hex", (1, 1))]

    # Compute hover efficiency for all configurations
    print("Computing hover efficiency for all configurations...")
    results = {}
    for num_rotor, title, _ in configurations:
        print(f"  Processing {title} configuration...")
        results[num_rotor] = compute_hover_efficiency(num_rotor, roll_grid, yaw_grid, fg_w)

    # Convert to ratio if needed
    if args.show_thrust_or_ratio == "ratio":
        data_maps = {k: v / (mass * gravity) for k, v in results.items()}
    else:
        data_maps = results

    # Find global min/max for unified color scale
    all_values = np.concatenate([data_maps[k].flatten() for k in data_maps.keys()])
    vmin = np.min(all_values)
    vmax = np.max(all_values)

    # Create 2x2 subplot layout
    plt.style.use(["science", "grid"])
    plt.rcParams.update({"font.size": 14})
    label_size = 16

    # =========================
    # Global annotation settings (tune these in one place)
    # =========================
    ANNO_TEXT_COLOR = "white"
    ANNO_POINT_COLOR = "red"
    ANNO_FONTSIZE = 14
    ANNO_POINT_SIZE = 12
    ANNO_BBOX = dict(boxstyle="round,pad=0.25", fc="black", ec="none", alpha=0.0)

    # Each point is: (roll_deg, yaw_deg, label_str_or_None)
    # - label is a custom string, or None to auto-generate (coords + value)
    ANNO_POINTS_BY_CONFIG = {
        3: [  # Tri
            (90, -45, "(-45$^\circ$, 90$^\circ$)"),
            (90, 45, "(45$^\circ$, 90$^\circ$)"),
        ],
        4: [  # Quad
            (90, -90, "(-90$^\circ$, 90$^\circ$)"),
            (90, 45, "(45$^\circ$, 90$^\circ$)"),
        ],
        5: [  # Penta
            (90, -90, "(-90$^\circ$, 90$^\circ$)"),
            (90, 36, "(36$^\circ$, 90$^\circ$)"),
        ],
        6: [  # Hex
            (90, -60, "(-60$^\circ$, 90$^\circ$)"),
            (90, 30, "(30$^\circ$, 90$^\circ$)"),
        ],
    }

    fig, axes = plt.subplots(2, 2, figsize=(10, 8), sharex=True, sharey=True)

    # Plot each configuration
    images = []
    for num_rotor, title, (row, col) in configurations:
        ax = axes[row, col]
        data = data_maps[num_rotor]

        im = ax.imshow(
            data,
            extent=(YAW_LOW, YAW_HIGH, ROLL_LOW, ROLL_HIGH),
            origin="lower",
            cmap="viridis",
            vmin=vmin,
            vmax=vmax,
            aspect="auto",
        )
        # =========================
        # Add keypoint annotations
        # =========================
        if num_rotor in ANNO_POINTS_BY_CONFIG:
            ax.text(
                0.02,
                0.98,
                f"$N_p$={num_rotor}",
                transform=ax.transAxes,  # use axes coordinates
                ha="left",
                va="top",
                fontsize=16,
                color="white",
                bbox=dict(boxstyle="round,pad=0.25", fc="black", ec="none", alpha=0.0),
                zorder=10,
            )
            annotate_keypoints(
                ax=ax,
                data=data,
                roll_grid=roll_grid,
                yaw_grid=yaw_grid,
                points=ANNO_POINTS_BY_CONFIG[num_rotor],
                text_color=ANNO_TEXT_COLOR,
                point_color=ANNO_POINT_COLOR,
                fontsize=ANNO_FONTSIZE,
                point_size=ANNO_POINT_SIZE,
                bbox=ANNO_BBOX,
                value_fmt="{:.3f}",  # Increase precision if needed, e.g., "{:.4f}"
                xytext=(0, -60),  # Text offset (pixels)
            )
        images.append(im)

        # ax.set_title(title, fontsize=label_size)
        ax.grid(False)

        # Set labels only for edge subplots
        if row == 1:  # Bottom row
            ax.set_xlabel("Yaw $\\psi$ [$^{\circ}$]", fontsize=label_size)
            # set x-ticks
            ax.set_xticks(np.array([-180, -90, 0, 90, 180]))
        if col == 0:  # Left column
            ax.set_ylabel("Roll $\\phi$ [$^{\circ}$]", fontsize=label_size)
            # set y-ticks
            ax.set_yticks(np.array([0, 45, 90, 135, 180]))

    # Add a single horizontal colorbar at the bottom
    fig.subplots_adjust(bottom=0.15, hspace=0.1, wspace=0.1)
    cbar_ax = fig.add_axes([0.12, 0.06, 0.78, 0.02])  # [left, bottom, width, height]
    cbar = fig.colorbar(images[0], cax=cbar_ax, orientation="horizontal")

    if args.show_thrust_or_ratio == "ratio":
        cbar.set_label(
            "Required Thrust-to-Weight Ratio (TWR) in Hovering \ $\sum_{i=1}^{N_p} f_i / (mg)$", fontsize=label_size
        )
    else:
        cbar.set_label("Total Thrust [N]", fontsize=label_size)

    plt.show()

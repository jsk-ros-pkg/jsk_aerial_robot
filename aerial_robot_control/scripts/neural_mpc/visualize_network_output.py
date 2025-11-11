import numpy as np
import torch
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import seaborn as sns
from config.configurations import EnvConfig
from utils.model_utils import load_model


def visualize_network_output_per_dimension(
    neural_model, state_feats, u_feats, output_dim=0, num_points=500, save_path=None
):
    """
    Visualize how the network output varies with each input dimension independently.
    """
    # Set up seaborn style for nicer plots
    sns.set_style("whitegrid")
    sns.set_palette("husl")

    # Define input dimension names and ranges
    input_configs = [
        # # Position (3D)
        # {'name': 'Position X', 'range': (-3.0, 3.0), 'default': 0.0, 'idx': 0},
        # {'name': 'Position Y', 'range': (-3.0, 3.0), 'default': 0.0, 'idx': 1},
        {"name": "Position Z", "range": (-3.0, 3.0), "default": 0.0, "idx": 0},
        # Velocity (3D)
        {"name": "Velocity X", "range": (-2.0, 2.0), "default": 0.0, "idx": 1},
        {"name": "Velocity Y", "range": (-2.0, 2.0), "default": 0.0, "idx": 2},
        {"name": "Velocity Z", "range": (-2.0, 2.0), "default": 0.0, "idx": 3},
        # Quaternion (4D)
        {"name": "Quaternion 0", "range": (-1.0, 1.0), "default": 1.0, "idx": 4},
        {"name": "Quaternion 1", "range": (-1.0, 1.0), "default": 0.0, "idx": 5},
        {"name": "Quaternion 2", "range": (-1.0, 1.0), "default": 0.0, "idx": 6},
        {"name": "Quaternion 3", "range": (-1.0, 1.0), "default": 0.0, "idx": 7},
        # # Angular velocity (3D)
        # {'name': 'Angular Velocity X', 'range': (-3.0, 3.0), 'default': 0.0, 'idx': 10},
        # {'name': 'Angular Velocity Y', 'range': (-3.0, 3.0), 'default': 0.0, 'idx': 11},
        # {'name': 'Angular Velocity Z', 'range': (-3.0, 3.0), 'default': 0.0, 'idx': 12},
        # # Alpha state (3D)
        # {'name': 'Alpha State 0', 'range': (-3.2, 3.2), 'default': 0.0, 'idx': 13},
        # {'name': 'Alpha State 1', 'range': (-3.2, 3.2), 'default': 0.0, 'idx': 14},
        # {'name': 'Alpha State 2', 'range': (-3.2, 3.2), 'default': 0.0, 'idx': 15},
        # Thrust (4D)
        {"name": "Thrust 0", "range": (-1.0, 30.0), "default": 5.0, "idx": 8},
        {"name": "Thrust 1", "range": (-1.0, 30.0), "default": 5.0, "idx": 9},
        {"name": "Thrust 2", "range": (-1.0, 30.0), "default": 5.0, "idx": 10},
        {"name": "Thrust 3", "range": (-1.0, 30.0), "default": 5.0, "idx": 11},
        # Alpha control (4D)
        {"name": "Alpha Control 0", "range": (-3.2, 3.2), "default": 0.0, "idx": 12},
        {"name": "Alpha Control 1", "range": (-3.2, 3.2), "default": 0.0, "idx": 13},
        {"name": "Alpha Control 2", "range": (-3.2, 3.2), "default": 0.0, "idx": 14},
        {"name": "Alpha Control 3", "range": (-3.2, 3.2), "default": 0.0, "idx": 15},
    ]

    # Create figure with subplots
    n_cols = 4
    n_rows = int(np.ceil(len(input_configs) / n_cols))
    fig = plt.figure(figsize=(20, 4 * n_rows))
    gs = GridSpec(n_rows, n_cols, figure=fig, hspace=0.3, wspace=0.3)

    # Color map for smooth gradients
    cmap = plt.cm.viridis

    for plot_idx, config in enumerate(input_configs):
        row = plot_idx // n_cols
        col = plot_idx % n_cols
        ax = fig.add_subplot(gs[row, col])

        # Create baseline input (all defaults)
        baseline_input = np.array([cfg["default"] for cfg in input_configs])

        # Vary the current dimension
        dim_values = np.linspace(config["range"][0], config["range"][1], num_points)
        outputs = []

        for val in dim_values:
            current_input = baseline_input.copy()
            current_input[config["idx"]] = val

            with torch.no_grad():
                input_tensor = torch.tensor(current_input.reshape(1, -1), dtype=torch.float32)
                output_tensor = neural_model(input_tensor)
                output_value = output_tensor.numpy()[0, output_dim]
                outputs.append(output_value)

        outputs = np.array(outputs)

        # Create color-coded line plot
        points = np.array([dim_values, outputs]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # Color by gradient (to visualize smoothness)
        gradients = np.abs(np.diff(outputs))

        # Plot the line with color coding
        scatter = ax.scatter(
            dim_values, outputs, c=np.arange(len(outputs)), cmap=cmap, s=10, alpha=0.6, edgecolors="none"
        )
        ax.plot(dim_values, outputs, "-", color="gray", alpha=0.3, linewidth=0.5)

        # Highlight discontinuities (large gradients)
        threshold = np.percentile(gradients, 99)
        discontinuities = np.where(gradients > threshold)[0]
        if len(discontinuities) > 0:
            for idx in discontinuities:
                ax.axvline(x=dim_values[idx], color="red", alpha=0.3, linestyle="--", linewidth=1)

        ax.set_xlabel(config["name"], fontsize=10, fontweight="bold")
        ax.set_ylabel(f"Output Dim {output_dim}", fontsize=9)
        # ax.set_title(f'{config["name"]}\n(Range: [{config["range"][0]:.1f}, {config["range"][1]:.1f}])',
        #             fontsize=10)
        ax.grid(True, alpha=0.3)

        # Add statistics to the plot
        mean_grad = np.mean(gradients)
        max_grad = np.max(gradients)
        textstr = f"Mean Δ: {mean_grad:.2e}\nMax Δ: {max_grad:.2e}"
        ax.text(
            0.02,
            0.98,
            textstr,
            transform=ax.transAxes,
            fontsize=8,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )

    fig.suptitle(
        f"Network Output (Dim {output_dim}) vs. Each Input Dimension\n"
        + "Red dashed lines indicate high gradients (potential discontinuities)",
        fontsize=16,
        fontweight="bold",
        y=0.995,
    )

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Figure saved to {save_path}")

    plt.show()

    return fig


def visualize_2d_output_heatmap(
    neural_model, state_feats, u_feats, dim1_idx=0, dim2_idx=1, output_dim=0, num_points=100, save_path=None
):
    """
    Create a 2D heatmap showing network output as a function of two input dimensions.

    Args:
        neural_model: The trained neural network
        dim1_idx: First input dimension index
        dim2_idx: Second input dimension index
        output_dim: Which output dimension to visualize
        num_points: Number of points per dimension
        save_path: Path to save the figure
    """
    # Define input ranges
    ranges = {
        0: (-3.0, 3.0),
        1: (-3.0, 3.0),
        2: (-0.5, 3.0),  # Position
        3: (-2.0, 2.0),
        4: (-2.0, 2.0),
        5: (-2.0, 2.0),  # Velocity
        6: (-1.0, 1.0),
        7: (-1.0, 1.0),
        8: (-1.0, 1.0),
        9: (-1.0, 1.0),  # Quaternion
        10: (-3.0, 3.0),
        11: (-3.0, 3.0),
        12: (-3.0, 3.0),  # Angular velocity
        13: (-3.2, 3.2),
        14: (-3.2, 3.2),
        15: (-3.2, 3.2),
        16: (-3.2, 3.2),  # Alpha state
        17: (-1.0, 30.0),
        18: (-1.0, 30.0),
        19: (-1.0, 30.0),
        20: (-1.0, 30.0),  # Thrust
        21: (-3.2, 3.2),
        22: (-3.2, 3.2),
        23: (-3.2, 3.2),
        24: (-3.2, 3.2),  # Alpha control
    }

    defaults = [0.0] * 1 + [0.0] * 3 + [1.0, 0.0, 0.0, 0.0] + [0.0] * 3 + [0.0] * 3 + [5.0] * 4 + [0.0] * 4

    dim_names = [
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "qw",
        "qx",
        "qy",
        "qz",
        "wx",
        "wy",
        "wz",
        "alpha_s1",
        "alpha_s2",
        "alpha_s3",
        "alpha_s4",
        "thrust1",
        "thrust2",
        "thrust3",
        "thrust4",
        "alpha_c1",
        "alpha_c2",
        "alpha_c3",
        "alpha_c4",
    ]

    # Create grid
    dim1_vals = np.linspace(ranges[dim1_idx][0], ranges[dim1_idx][1], num_points)
    dim2_vals = np.linspace(ranges[dim2_idx][0], ranges[dim2_idx][1], num_points)

    output_grid = np.zeros((num_points, num_points))

    for i, val1 in enumerate(dim1_vals):
        for j, val2 in enumerate(dim2_vals):
            current_input = np.array(defaults.copy())
            current_input[dim1_idx] = val1
            current_input[dim2_idx] = val2

            x = current_input[state_feats]
            u = current_input[u_feats]
            current_input_full = np.concatenate([x, u])

            with torch.no_grad():
                input_tensor = torch.tensor(current_input_full.reshape(1, -1), dtype=torch.float32)
                output_tensor = neural_model(input_tensor)
                output_grid[j, i] = output_tensor.numpy()[0, output_dim]

    # Create heatmap
    fig, ax = plt.subplots(figsize=(10, 8))

    im = ax.imshow(
        output_grid,
        extent=[ranges[dim1_idx][0], ranges[dim1_idx][1], ranges[dim2_idx][0], ranges[dim2_idx][1]],
        origin="lower",
        aspect="auto",
        cmap="RdYlBu_r",
        interpolation="bilinear",
    )

    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label(f"Output Dimension {output_dim}", fontsize=12, fontweight="bold")

    ax.set_xlabel(dim_names[dim1_idx], fontsize=12, fontweight="bold")
    ax.set_ylabel(dim_names[dim2_idx], fontsize=12, fontweight="bold")
    ax.set_title(
        f"Network Output Heatmap: {dim_names[dim1_idx]} vs {dim_names[dim2_idx]}", fontsize=14, fontweight="bold"
    )

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Heatmap saved to {save_path}")

    plt.show()

    return fig


def main():
    # === Define and load model
    model_options = EnvConfig.model_options
    solver_options = EnvConfig.solver_options
    sim_options = EnvConfig.sim_options
    run_options = EnvConfig.run_options
    neural_model, mlp_metadata = load_model(model_options, sim_options, run_options)
    neural_model.eval()

    state_feats = eval(mlp_metadata["ModelFitConfig"]["state_feats"])
    u_feats = eval(mlp_metadata["ModelFitConfig"]["u_feats"])
    y_reg_dims = np.array(eval(mlp_metadata["ModelFitConfig"]["y_reg_dims"]))
    input_transform = mlp_metadata["ModelFitConfig"]["input_transform"]
    label_transform = mlp_metadata["ModelFitConfig"]["label_transform"]

    # Visualize each output dimension
    for output_dim in range(y_reg_dims.shape[0]):
        print(f"\nVisualizing output dimension {output_dim}...")

        # 1D visualization for each input dimension
        fig1 = visualize_network_output_per_dimension(
            neural_model,
            state_feats,
            u_feats,
            output_dim=output_dim,
            num_points=500,
            # save_path=f'network_output_dim{output_dim}_1d.png'
        )

    for output_dim in range(y_reg_dims.shape[0]):
        # 2D heatmaps for selected dimension pairs (example: position x vs y)
        print(f"Creating 2D heatmap for Position X vs Position Y...")
        fig2 = visualize_2d_output_heatmap(
            neural_model,
            state_feats,
            u_feats,
            dim1_idx=2,  # z
            dim2_idx=5,  # vz
            output_dim=output_dim,
            num_points=100,
            # save_path=f'network_output_dim{output_dim}_2d_pos.png'
        )

        print(f"Creating 2D heatmap for Thrust 0 vs Thrust 1...")
        fig3 = visualize_2d_output_heatmap(
            neural_model,
            state_feats,
            u_feats,
            dim1_idx=17,  # Thrust 0
            dim2_idx=18,  # Thrust 1
            output_dim=output_dim,
            num_points=100,
            # save_path=f'network_output_dim{output_dim}_2d_thrust.png'
        )

    print("\nVisualization complete!")


if __name__ == "__main__":
    main()

import numpy as np
import torch
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import seaborn as sns
from config.configurations import EnvConfig
from utils.model_utils import load_model


def draw_network_weights(neural_model, save_path=None):
    pass


def visualize_network_output_per_dimension(
    neural_model, input_configs, output_dim, output_names, num_points=500, save_path=None
):
    """
    Visualize how the network output varies with each input dimension independently.
    """
    # Set up seaborn style for nicer plots
    sns.set_style("whitegrid")
    sns.set_palette("husl")

    # Create figure with subplots
    n_cols = 4
    n_rows = int(np.ceil(len(input_configs) / n_cols))
    fig = plt.figure(figsize=(20, 4 * n_rows))
    gs = GridSpec(n_rows, n_cols, figure=fig, hspace=0.3, wspace=0.3)

    # Color map for smooth gradients
    cmap = plt.cm.viridis

    for config_idx, config in enumerate(input_configs):
        row = config_idx // n_cols
        col = config_idx % n_cols
        ax = fig.add_subplot(gs[row, col])

        # Create baseline input (all defaults)
        baseline_input = np.array([cfg["default"] for cfg in input_configs])

        # Vary the current dimension
        dim_values = np.linspace(config["range"][0], config["range"][1], num_points)
        outputs = []

        for val in dim_values:
            current_input = baseline_input.copy()
            current_input[config_idx] = val

            with torch.no_grad():
                input_tensor = torch.tensor(current_input, dtype=torch.float32)
                output_tensor = neural_model(input_tensor)
                output_value = output_tensor.numpy()[output_dim]
                outputs.append(output_value)

        outputs = np.array(outputs)

        # Create color-coded line plot
        points = np.array([dim_values, outputs]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # Color by gradient (to visualize smoothness)
        gradients = np.abs(np.diff(outputs))

        # Plot the line with color coding
        ax.scatter(dim_values, outputs, c=np.arange(len(outputs)), cmap=cmap, s=10, alpha=0.6, edgecolors="none")
        ax.plot(dim_values, outputs, "-", color="gray", alpha=0.3, linewidth=0.5)

        # Highlight discontinuities (large gradients)
        threshold = 0.01 * (np.max(outputs) - np.min(outputs))
        discontinuities = np.where(gradients > threshold)[0]
        if len(discontinuities) > 0:
            for idx in discontinuities:
                ax.axvline(x=dim_values[idx], color="red", alpha=0.3, linestyle="--", linewidth=1)

        ax.set_xlabel(config["name"], fontsize=10, fontweight="bold")
        ax.set_ylabel(f"Output {output_names[output_dim]}", fontsize=9)
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
        f"Network Output {output_names[output_dim]} vs. Each Input Dimension\n"
        + "Red dashed lines indicate high gradients (potential discontinuities)\n"
        + "Defaults: "
        + ", ".join([f"{cfg['name']}={cfg['default']:.2f}" for cfg in input_configs]),
        fontsize=16,
        fontweight="bold",
        y=0.995,
    )

    plt.tight_layout()
    figManager = plt.get_current_fig_manager()
    figManager.resize(*figManager.window.maxsize())

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Figure saved to {save_path}")


def visualize_2d_output_heatmap(neural_model, input_configs, output_dim, output_names, num_points=100, save_path=None):
    """
    Create a 2D heatmap showing network output as a function of two input dimensions.

    Args:
        neural_model: The trained neural network
        input_configs: List of input configuration dictionaries
        output_dim: Which output dimension to visualize
        output_names: Names of output dimensions
        num_points: Number of points per dimension
        save_path: Path to save the figure
    """
    names = [cfg["name"] for cfg in input_configs]
    ranges = [cfg["range"] for cfg in input_configs]
    defaults = [cfg["default"] for cfg in input_configs]

    # Only plot unique pairs (dim1_idx < dim2_idx), omit mirrored and same-dimension plots
    n_dims = len(input_configs)
    pairs = [(i, j) for i in range(n_dims) for j in range(i + 1, n_dims)]
    n_plots = len(pairs)
    n_cols = int(np.ceil(np.sqrt(n_plots)))
    n_rows = int(np.ceil(n_plots / n_cols))
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 5 * n_rows))
    axes = np.array(axes).reshape(-1)

    for idx, (dim1_idx, dim2_idx) in enumerate(pairs):
        ax = axes[idx]

        # Create grid
        dim1_vals = np.linspace(ranges[dim1_idx][0], ranges[dim1_idx][1], num_points)
        dim2_vals = np.linspace(ranges[dim2_idx][0], ranges[dim2_idx][1], num_points)

        output_grid = np.zeros((num_points, num_points))

        for i, val1 in enumerate(dim1_vals):
            for j, val2 in enumerate(dim2_vals):
                current_input = np.array(defaults.copy())
                current_input[dim1_idx] = val1
                current_input[dim2_idx] = val2

                with torch.no_grad():
                    input_tensor = torch.tensor(current_input, dtype=torch.float32)
                    output_tensor = neural_model(input_tensor)
                    output_grid[j, i] = output_tensor.numpy()[output_dim]

        # Create heatmap
        ax.grid(False)
        im = ax.imshow(
            output_grid,
            extent=[ranges[dim1_idx][0], ranges[dim1_idx][1], ranges[dim2_idx][0], ranges[dim2_idx][1]],
            origin="lower",
            aspect="auto",
            cmap="RdYlBu_r",
            interpolation="bilinear",
        )
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label(f"{output_names[output_dim]}", fontsize=8)

        ax.set_xlabel(names[dim1_idx], fontsize=9)
        ax.set_ylabel(names[dim2_idx], fontsize=9)
        ax.set_title(f"{names[dim1_idx]} vs. {names[dim2_idx]}", fontsize=10)
        ax.axes.xaxis.set_ticklabels([])

    # Hide unused axes
    for ax in axes[n_plots:]:
        ax.axis("off")

    fig.suptitle(
        f"Network Output {output_names[output_dim]} - All Input Dimension Pairs",
        fontsize=16,
        fontweight="bold",
        y=0.995,
    )

    figManager = plt.get_current_fig_manager()
    figManager.resize(*figManager.window.maxsize())
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Heatmap saved to {save_path}")


def main():
    # === Define and load model
    model_options = EnvConfig.model_options
    sim_options = EnvConfig.sim_options
    run_options = EnvConfig.run_options
    neural_model, mlp_metadata = load_model(model_options, sim_options, run_options)
    neural_model.eval()

    state_feats = eval(mlp_metadata["ModelFitConfig"]["state_feats"])
    u_feats = eval(mlp_metadata["ModelFitConfig"]["u_feats"])
    y_reg_dims = np.array(eval(mlp_metadata["ModelFitConfig"]["y_reg_dims"]))

    # Define input dimension names and ranges
    state_configs = [
        # Position
        {"name": "x", "range": (-3.0, 3.0), "default": 0.0},
        {"name": "y", "range": (-3.0, 3.0), "default": 0.0},
        {"name": "z", "range": (-0.5, 3.0), "default": 1.0},
        # Velocity
        {"name": "vx", "range": (-2.0, 2.0), "default": 0.0},
        {"name": "vy", "range": (-2.0, 2.0), "default": 0.0},
        {"name": "vz", "range": (-2.0, 2.0), "default": 0.0},
        # Quaternion
        {"name": "qw", "range": (-1.1, 1.1), "default": 1.0},
        {"name": "qx", "range": (-1.1, 1.1), "default": 0.0},
        {"name": "qy", "range": (-1.1, 1.1), "default": 0.0},
        {"name": "qz", "range": (-1.1, 1.1), "default": 0.0},
        # Angular velocity
        {"name": "wx", "range": (-3.0, 3.0), "default": 0.0},
        {"name": "wy", "range": (-3.0, 3.0), "default": 0.0},
        {"name": "wz", "range": (-3.0, 3.0), "default": 0.0},
        # Servo angle state
        {"name": "alpha_s1", "range": (-3.2, 3.2), "default": 0.0},
        {"name": "alpha_s2", "range": (-3.2, 3.2), "default": 0.0},
        {"name": "alpha_s3", "range": (-3.2, 3.2), "default": 0.0},
        {"name": "alpha_s4", "range": (-3.2, 3.2), "default": 0.0},
    ]

    control_configs = [
        # Thrust command
        {"name": "thrust_cmd1", "range": (-1.0, 30.0), "default": 7.5},
        {"name": "thrust_cmd2", "range": (-1.0, 30.0), "default": 7.5},
        {"name": "thrust_cmd3", "range": (-1.0, 30.0), "default": 7.5},
        {"name": "thrust_cmd4", "range": (-1.0, 30.0), "default": 7.5},
        # Servo angle command
        {"name": "alpha_cmd1", "range": (-3.2, 3.2), "default": 0.0},
        {"name": "alpha_cmd2", "range": (-3.2, 3.2), "default": 0.0},
        {"name": "alpha_cmd3", "range": (-3.2, 3.2), "default": 0.0},
        {"name": "alpha_cmd4", "range": (-3.2, 3.2), "default": 0.0},
    ]

    input_configs = list(np.array(state_configs)[state_feats]) + list(np.array(control_configs)[u_feats])

    if len(y_reg_dims) == 1:
        output_names = ["az"]
    else:
        output_names = ["ax", "ay", "az"]

    # Visualize each output dimension
    for output_dim in range(y_reg_dims.shape[0]):
        print(f"\nVisualizing output dimension {output_dim}...")

        # 1D visualization for each input dimension
        visualize_network_output_per_dimension(
            neural_model,
            input_configs,
            output_dim,
            output_names,
            num_points=500,
            # save_path=f"network_output_dim{output_dim}_1d.png"
        )

    for output_dim in range(y_reg_dims.shape[0]):
        # 2D heatmaps for selected dimension pairs (example: position x vs y)
        print(f"Creating 2D heatmaps for {output_dim}...")
        visualize_2d_output_heatmap(
            neural_model,
            input_configs,
            output_dim,
            output_names,
            num_points=100,
            # save_path=f"network_output_dim{output_dim}_2d_pos.png"
        )

    plt.show()


if __name__ == "__main__":
    main()

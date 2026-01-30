""" Miscellaneous visualization functions.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""
import os
import numpy as np
import torch
import tikzplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # DON'T REMOVE THIS LINE, IT IS NEEDED FOR 3D PLOTTING
import matplotlib.animation as animation

from config.configurations import DirectoryConfig
from utils.geometry_utils import v_dot_q, quaternion_to_euler, quaternion_inverse, q_dot_q
from utils.data_utils import safe_mkdir_recursive
from sim_environment.forward_prop import init_forward_prop
from neural_controller import NeuralMPC


frames = []


def initialize_plotter(world_rad, n_properties):
    """
    Initializes the real-time 3D plot for the robot simulation.
    :param world_rad: Radius of the world in meters.
    :param n_properties: Number of properties to visualize.
    :param full_traj: Optional full trajectory to plot as a dashed line.

    Setup figure size based on resolution
    resolution_settings = {
        '720p': {'figsize': (12.8, 7.2), 'dpi': 100, 'bitrate': 3000},
        '1080p': {'figsize': (19.2, 10.8), 'dpi': 100, 'bitrate': 5000},
        '1440p': {'figsize': (25.6, 14.4), 'dpi': 100, 'bitrate': 8000},
        '4k': {'figsize': (38.4, 21.6), 'dpi': 100, 'bitrate': 15000}
    }
    """
    # Set size, aspect ratio and resolution
    fig = plt.figure(figsize=(8, 8), dpi=96)
    fig.show()

    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())

    # Create 3D renderer object
    ax = fig.add_subplot(111, projection="3d")

    # Set limits
    ax.set_xlim([-world_rad, world_rad])
    ax.set_ylim([-world_rad, world_rad])
    ax.set_zlim([0, 2.25 * world_rad])

    # Set labels
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    fig.canvas.draw()
    plt.draw()

    # Cache the background for effiecient rendering
    background = fig.canvas.copy_from_bbox(ax.bbox)

    artists = {
        "trajectory": ax.plot([], [])[0],
        "robot": ax.plot([], [], "o-")[0],
        "robot_first_rotor": ax.plot([], [], "o-", color="r")[0],  # Highlight first rotor
        "missing_targets": ax.plot([], [], [], color="r", marker="o", linestyle="None", markersize=12)[0],
        "reached_targets": ax.plot([], [], [], color="g", marker="o", linestyle="None", markersize=12)[0],
        "sim_trajectory": [
            ax.plot([], [], [], "-", color="tab:blue", alpha=0.9 - i * 0.2 / n_properties)[0]
            for i in range(n_properties)
        ],
        "int_trajectory": [
            ax.plot([], [], [], "-", color="tab:orange", alpha=0.9 - i * 0.5 / n_properties)[0]
            for i in range(n_properties + 1)
        ],
        "prop_trajectory": [
            ax.plot([], [], [], "-", color="tab:red", alpha=0.9 - i * 0.2 / n_properties)[0]
            for i in range(n_properties)
        ],
        "prop_covariance": [
            ax.plot([], [], [], color="r", alpha=0.5 - i * 0.45 / n_properties)[0] for i in range(n_properties)
        ],
        "projection_traj": [
            ax.plot([], [], [], "-", color="tab:blue", alpha=0.2)[0],
            ax.plot([], [], [], "-", color="tab:blue", alpha=0.2)[0],
            ax.plot([], [], [], "-", color="tab:blue", alpha=0.2)[0],
        ],
        "projection_target": [
            ax.plot([], [], [], marker="o", color="r", linestyle="None", alpha=0.2)[0],
            ax.plot([], [], [], marker="o", color="r", linestyle="None", alpha=0.2)[0],
            ax.plot([], [], [], marker="o", color="r", linestyle="None", alpha=0.2)[0],
        ],
    }

    art_pack = fig, ax, artists, background, world_rad
    return art_pack


def draw_robot(
    art_pack,
    targets,
    targets_reached,
    state_curr,
    state_traj,
    trajectory_history,
    rotor_positions,
    follow_robot=False,
    animation=False,
):
    """
    Animates the robot's state, previous and predicted trajectories, and their projections in a 3D plot.
    :param art_pack: Tuple containing the figure, axis, artists, background, and world radius.
    :param targets: Array of target positions.
    :param targets_reached: Boolean array indicating which targets have been reached.
    :param state_curr: Current state of the robot.
    :param state_traj: Predicted trajectory of the robot.
    :param trajectory_history: History of the robot's trajectory.
    :param rotor_positions: Positions of the rotors in the robot's body frame.
    :param follow_robot: If True, the view will follow the robot's position.
    :param animation: If True, record each frame for animation
    """
    # Unpack the art_pack
    fig, ax, artists, background, world_rad = art_pack

    # Unpack 3D artists
    missing_targets_artist = artists["missing_targets"] if "missing_targets" in artists.keys() else []
    reached_targets_artist = artists["reached_targets"] if "reached_targets" in artists.keys() else []
    projection_target_artists = artists["projection_target"] if "projection_target" in artists.keys() else []
    trajectories_artist = artists["trajectory"] if "trajectory" in artists.keys() else []
    projected_traj_artists = artists["projection_traj"] if "projection_traj" in artists.keys() else []
    robot_sketch_artist = artists["robot"] if "robot" in artists.keys() else []
    robot_sketch_artist_first_rotor = artists["robot_first_rotor"] if "robot_first_rotor" in artists.keys() else []
    sim_traj_artists = artists["sim_trajectory"] if "sim_trajectory" in artists.keys() else []
    # int_traj_artists = artists["int_trajectory"] if "int_trajectory" in artists.keys() else []
    # pred_traj_artists = artists["prop_trajectory"] if "prop_trajectory" in artists.keys() else []

    # Restore background
    fig.canvas.restore_region(background)

    # Draw missing and reached targets
    if targets is not None and targets_reached is not None:
        missing = targets[targets_reached == False, :][:, :3]
        missing_targets_artist.set_data_3d(missing[:, 0], missing[:, 1], missing[:, 2])
        ax.draw_artist(missing_targets_artist)

        reached = targets[targets_reached == True, :][:, :3]
        reached = reached[-2:, :]  # Only draw last two reached targets
        reached_targets_artist.set_data_3d(reached[:, 0], reached[:, 1], reached[:, 2])
        ax.draw_artist(reached_targets_artist)

        # Draw projected target
        if missing.any():
            # Project first/next missing target onto the x-y axes
            projection_target_artists[0].set_data_3d([ax.get_xlim()[0]], [missing[0, 1]], [missing[0, 2]])
            projection_target_artists[1].set_data_3d([missing[0, 0]], [ax.get_ylim()[1]], [missing[0, 2]])
            projection_target_artists[2].set_data_3d([missing[0, 0]], [missing[0, 1]], [ax.get_zlim()[0]])
            [ax.draw_artist(projected_tar_artist) for projected_tar_artist in projection_target_artists]

    # Draw robot art
    robot_coords = compute_robot_coords(state_curr[:3], state_curr[6:10], rotor_positions)
    robot_sketch_artist.set_data_3d(robot_coords)
    robot_sketch_artist_first_rotor.set_data_3d(robot_coords[0][0], robot_coords[1][0], robot_coords[2][0])
    ax.draw_artist(robot_sketch_artist)
    ax.draw_artist(robot_sketch_artist_first_rotor)

    # Draw previous trajectory
    trajectory_start_pt = max(len(trajectory_history) - 100, 0)  # Start from the last 100 points
    trajectories_artist.set_data_3d(
        trajectory_history[trajectory_start_pt:, 0],
        trajectory_history[trajectory_start_pt:, 1],
        trajectory_history[trajectory_start_pt:, 2],
    )
    ax.draw_artist(trajectories_artist)

    # Draw previous trajectory projections
    projected_traj_artists[0].set_data_3d(
        np.full_like(trajectory_history[trajectory_start_pt:, 1], ax.get_xlim()[0]),
        trajectory_history[trajectory_start_pt:, 1],
        trajectory_history[trajectory_start_pt:, 2],
    )
    projected_traj_artists[1].set_data_3d(
        trajectory_history[trajectory_start_pt:, 0],
        np.full_like(trajectory_history[trajectory_start_pt:, 2], ax.get_ylim()[1]),
        trajectory_history[trajectory_start_pt:, 2],
    )
    projected_traj_artists[2].set_data_3d(
        trajectory_history[trajectory_start_pt:, 0],
        trajectory_history[trajectory_start_pt:, 1],
        np.full_like(trajectory_history[trajectory_start_pt:, 0], ax.get_zlim()[0]),
    )
    [ax.draw_artist(projected_traj_artist) for projected_traj_artist in projected_traj_artists]

    # Draw predicted trajectory
    if state_traj is not None:
        draw_fading_traj(state_traj, sim_traj_artists)
        for sim_traj_artist in sim_traj_artists:
            ax.draw_artist(sim_traj_artist)

    # if int_traj is not None:
    #     draw_fading_traj(int_traj, int_traj_artists)
    #     for int_traj_artist in int_traj_artists:
    #         ax.draw_artist(int_traj_artist)

    # if pred_traj is not None:
    #     draw_fading_traj(pred_traj, pred_traj_artists)
    #     for pred_traj_artist in pred_traj_artists:
    #         ax.draw_artist(pred_traj_artist)

    if follow_robot:
        # Center view on robot
        ax.set_xlim([trajectory_history[-1, 0] - world_rad, trajectory_history[-1, 0] + world_rad])
        ax.set_ylim([trajectory_history[-1, 1] - world_rad, trajectory_history[-1, 1] + world_rad])
        ax.set_zlim([trajectory_history[-1, 2] - world_rad, trajectory_history[-1, 2] + world_rad])

    # Fill in the axes rectangle
    fig.canvas.blit(ax.bbox)

    # Record frame
    if animation:
        # Capture the current frame
        fig.canvas.draw()
        frame = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        frame = frame.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        frames.append(frame)


def animate_robot(file_name):
    """
    Create video from frames
    """

    def animate(frame_num):
        plt.clf()
        plt.imshow(frames[frame_num])
        plt.axis("off")

    anim = animation.FuncAnimation(plt.figure(), animate, frames=len(frames))
    anim.save(file_name, writer="ffmpeg", fps=30, dpi=600, bitrate=5000)


def draw_fading_traj(traj, traj_artists):
    """
    Draws a trajectory by updating the data of the 3D artists.
    :param traj: The trajectory to draw, as an array of 3D points.
    :param traj_artists: A list of artists to update with the trajectory data.
    """
    traj = np.squeeze(traj)
    for j in range(min(traj.shape[0] - 1, len(traj_artists))):
        # Get the x,y,z coords from two consecutive points along traj with length N+1 and draw line
        traj_artists[j].set_data_3d(
            [traj[j, 0], traj[j + 1, 0]], [traj[j, 1], traj[j + 1, 1]], [traj[j, 2], traj[j + 1, 2]]
        )


def compute_robot_coords(pos, quaternions, rotor_positions):
    # Define quadrotor extremities in Body frame
    r1 = np.array(rotor_positions[0])
    r2 = np.array(rotor_positions[1])
    r3 = np.array(rotor_positions[2])
    r4 = np.array(rotor_positions[3])

    # Convert to World frame and add quadrotor center point
    r1 = v_dot_q(r1, quaternions) + pos
    r2 = v_dot_q(r2, quaternions) + pos
    r3 = v_dot_q(r3, quaternions) + pos
    r4 = v_dot_q(r4, quaternions) + pos

    # Build set of coordinates for plotting
    return (
        [r1[0], r3[0], pos[0], r2[0], r4[0]],
        [r1[1], r3[1], pos[1], r2[1], r4[1]],
        [r1[2], r3[2], pos[2], r2[2], r4[2]],
    )


def plot_dataset(
        x,
        y,
        dt, state_in, state_out, state_prop, control, use_moving_average_filter=False, state_in_filtered=None, control_in_filtered=None, y_raw=None, y_filtered=None, save_file_path=None, save_file_name=None, mode=None):
    """
    Plot the dataset features and labels.
    :param x: Input features to the network.
    :param y: Labels of the dataset, i.e., ground truth values.
    """
    figures = []
    # State features
    # --- Position
    fig = plt.figure()
    plt.subplot(3, 1, 1)
    plt.title("State In & Out & Prop (Position)")
    plt.plot(state_in[:, 0], label="state_in")
    plt.plot(state_out[:, 0], label="state_out")
    plt.plot(state_prop[:, 0], label="state_prop")
    plt.legend()
    plt.ylabel("x [m]")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(3, 1, 2)
    plt.plot(state_in[:, 1])
    plt.plot(state_out[:, 1])
    plt.plot(state_prop[:, 1])
    plt.ylabel("y [m]")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(3, 1, 3)
    plt.plot(state_in[:, 2])
    plt.plot(state_out[:, 2])
    plt.plot(state_prop[:, 2])
    plt.ylabel("z [m]")
    plt.grid("on")
    plt.tight_layout()
    figures.append(fig)

    if use_moving_average_filter:
        fig = plt.figure()
        plt.subplot(3, 1, 1)
        plt.title("State In Filtered vs. Unfiltered (Position)")
        plt.plot(state_in[:, 0], label="state_in")
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 0], label="state_in_filtered")
        plt.legend()
        plt.ylabel("x [m]")
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])
        plt.subplot(3, 1, 2)
        plt.plot(state_in[:, 1])
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 1])
        plt.ylabel("y [m]")
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])
        plt.subplot(3, 1, 3)
        plt.plot(state_in[:, 2])
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 2])
        plt.ylabel("z [m]")
        plt.grid("on")
        plt.tight_layout()
        figures.append(fig)

    # --- Velocity
    # NOTE: Velocity is transformed to Body frame already
    fig = plt.figure()
    plt.subplot(3, 1, 1)
    plt.title("State In & Out & Prop (Velocity, transformed)")
    plt.plot(state_in[:, 3], label="state_in")
    plt.plot(state_out[:, 3], label="state_out")
    plt.plot(state_prop[:, 3], label="state_prop")
    plt.legend()
    plt.ylabel("vx [m]")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(3, 1, 2)
    plt.plot(state_in[:, 4])
    plt.plot(state_out[:, 4])
    plt.plot(state_prop[:, 4])
    plt.ylabel("vy [m]")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(3, 1, 3)
    plt.plot(state_in[:, 5])
    plt.plot(state_out[:, 5])
    plt.plot(state_prop[:, 5])
    plt.ylabel("vz [m]")
    plt.grid("on")
    plt.tight_layout()
    figures.append(fig)

    if use_moving_average_filter:
        fig = plt.figure()
        plt.subplot(3, 1, 1)
        plt.title("State In Filtered vs. Unfiltered (Velocity)")
        plt.plot(state_in[:, 3], label="state_in")
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 3], label="state_in_filtered")
        plt.legend()
        plt.ylabel("vx [m]")
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])
        plt.subplot(3, 1, 2)
        plt.plot(state_in[:, 4])
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 4])
        plt.ylabel("vy [m]")
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])
        plt.subplot(3, 1, 3)
        plt.plot(state_in[:, 5])
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 5])
        plt.ylabel("vz [m]")
        plt.grid("on")
        plt.tight_layout()
        figures.append(fig)

    # --- Quaternion
    fig = plt.figure()
    plt.subplot(4, 1, 1)
    plt.title("State In & Out & Prop (Quaternion)")
    plt.plot(state_in[:, 6], label="state_in")
    plt.plot(state_out[:, 6], label="state_out")
    plt.plot(state_prop[:, 6], label="state_prop")
    plt.legend()
    plt.ylabel("qw")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(4, 1, 2)
    plt.plot(state_in[:, 7])
    plt.plot(state_out[:, 7])
    plt.plot(state_prop[:, 7])
    plt.ylabel("qx")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(4, 1, 3)
    plt.plot(state_in[:, 8])
    plt.plot(state_out[:, 8])
    plt.plot(state_prop[:, 8])
    plt.ylabel("qy")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(4, 1, 4)
    plt.plot(state_in[:, 9])
    plt.plot(state_out[:, 9])
    plt.plot(state_prop[:, 9])
    plt.ylabel("qz")
    plt.grid("on")
    plt.tight_layout()
    figures.append(fig)

    # --- Angular Velocity
    fig = plt.figure()
    plt.subplot(3, 1, 1)
    plt.title("State In & Out & Prop (Angular Velocity)")
    plt.plot(state_in[:, 10], label="state_in")
    plt.plot(state_out[:, 10], label="state_out")
    plt.plot(state_prop[:, 10], label="state_prop")
    plt.legend()
    plt.ylabel("wx [rad/s]")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(3, 1, 2)
    plt.plot(state_in[:, 11])
    plt.plot(state_out[:, 11])
    plt.plot(state_prop[:, 11])
    plt.ylabel("wy [rad/s]")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(3, 1, 3)
    plt.plot(state_in[:, 12])
    plt.plot(state_out[:, 12])
    plt.plot(state_prop[:, 12])
    plt.ylabel("wz [rad/s]")
    plt.grid("on")
    plt.tight_layout()
    figures.append(fig)

    if use_moving_average_filter:
        fig = plt.figure()
        plt.subplot(4, 1, 1)
        plt.title("State In Filtered vs. Unfiltered (Quaternion)")
        plt.plot(state_in[:, 6], label="state_in")
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 6], label="state_in_filtered")
        plt.legend()
        plt.ylabel("qw")
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])
        plt.subplot(4, 1, 2)
        plt.plot(state_in[:, 7])
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 7])
        plt.ylabel("qx")
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])
        plt.subplot(4, 1, 3)
        plt.plot(state_in[:, 8])
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 8])
        plt.ylabel("qy")
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])
        plt.subplot(4, 1, 4)
        plt.plot(state_in[:, 9])
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 9])
        plt.ylabel("qz")
        plt.grid("on")
        plt.tight_layout()
        figures.append(fig)

    # --- Servo Angle State
    fig = plt.figure()
    plt.subplot(4, 1, 1)
    plt.title("State In & Out & Prop (Servo Angle)")
    plt.plot(state_in[:, 13], label="state_in")
    plt.plot(state_out[:, 13], label="state_out")
    plt.plot(state_prop[:, 13], label="state_prop")
    plt.legend()
    plt.ylabel("alpha 1 [rad]")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(4, 1, 2)
    plt.plot(state_in[:, 14])
    plt.plot(state_out[:, 14])
    plt.plot(state_prop[:, 14])
    plt.ylabel("alpha 2 [rad]")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(4, 1, 3)
    plt.plot(state_in[:, 15])
    plt.plot(state_out[:, 15])
    plt.plot(state_prop[:, 15])
    plt.ylabel("alpha 3 [rad]")
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])
    plt.subplot(4, 1, 4)
    plt.plot(state_in[:, 16])
    plt.plot(state_out[:, 16])
    plt.plot(state_prop[:, 16])
    plt.ylabel("alpha 4 [rad]")
    plt.grid("on")
    plt.tight_layout()
    figures.append(fig)

    if use_moving_average_filter:
        fig = plt.figure()
        plt.subplot(4, 1, 1)
        plt.title("State In Filtered vs. Unfiltered (Servo Angle)")
        plt.plot(state_in[:, 13], label="state_in")
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 13], label="state_in_filtered")
        plt.legend()
        plt.ylabel("alpha 1 [rad]")
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])
        plt.subplot(4, 1, 2)
        plt.plot(state_in[:, 14])
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 14])
        plt.ylabel("alpha 2 [rad]")
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])
        plt.subplot(4, 1, 3)
        plt.plot(state_in[:, 15])
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 15])
        plt.ylabel("alpha 3 [rad]")
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])
        plt.subplot(4, 1, 4)
        plt.plot(state_in[:, 16])
        if use_moving_average_filter:
            plt.plot(state_in_filtered[:, 16])
        plt.ylabel("alpha 4 [rad]")
        plt.grid("on")
        plt.tight_layout()
        figures.append(fig)

    # Control inputs
    # --- Thrust Command
    fig = plt.figure()
    plt.subplot(2, 1, 1)
    plt.title("Control Inputs")
    plt.plot(control[:, 0], label="thrust_cmd_1")
    plt.plot(control[:, 1], label="thrust_cmd_2")
    plt.plot(control[:, 2], label="thrust_cmd_3")
    plt.plot(control[:, 3], label="thrust_cmd_4")
    if use_moving_average_filter:
        plt.plot(control_in_filtered[:, 0], label="thrust_cmd_1_filtered", linestyle="--")
        plt.plot(control_in_filtered[:, 1], label="thrust_cmd_2_filtered", linestyle="--")
        plt.plot(control_in_filtered[:, 2], label="thrust_cmd_3_filtered", linestyle="--")
        plt.plot(control_in_filtered[:, 3], label="thrust_cmd_4_filtered", linestyle="--")
    plt.ylabel("Thrust Command [N]")
    plt.legend()
    plt.grid("on")
    plt.tight_layout()
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    # --- Servo Angle Command
    plt.subplot(2, 1, 2)
    plt.plot(control[:, 4], label="servo_cmd_1")
    plt.plot(control[:, 5], label="servo_cmd_2")
    plt.plot(control[:, 6], label="servo_cmd_3")
    plt.plot(control[:, 7], label="servo_cmd_4")
    if use_moving_average_filter:
        plt.plot(control_in_filtered[:, 4], label="servo_cmd_1_filtered", linestyle="--")
        plt.plot(control_in_filtered[:, 5], label="servo_cmd_2_filtered", linestyle="--")
        plt.plot(control_in_filtered[:, 6], label="servo_cmd_3_filtered", linestyle="--")
        plt.plot(control_in_filtered[:, 7], label="servo_cmd_4_filtered", linestyle="--")
    plt.ylabel("Servo Angle Command [rad]")
    plt.legend()
    plt.grid("on")
    plt.tight_layout()
    figures.append(fig)

    # Network Inputs
    fig = plt.figure()
    for dim in range(x.shape[1]):
        plt.subplot(x.shape[1], 1, dim + 1)
        plt.plot(x[:, dim])
        if dim == 0:
            plt.title("Network Inputs (filtered & pruned & transformed)")
        plt.grid("on")
        plt.ylabel(f"D{dim}")
        if dim != x.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    figures.append(fig)

    # Labels
    fig = plt.figure()
    for dim in range(y.shape[1]):
        plt.subplot(y.shape[1], 1, dim + 1)
        if use_moving_average_filter:
            plt.plot(y_raw[:, dim], color="tab:red")
            plt.plot(y_filtered[:, dim], label="filtered", color="tab:blue")
        else:
            plt.plot(y[:, dim], color="tab:red")
        if dim == 0:
            plt.title("Labels (filtered & pruned & possibly transformed)")
        plt.grid("on")
        plt.ylabel(f"D{dim}")
        if dim != y.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    figures.append(fig)

    diff = state_out - state_prop
    y_all = diff / np.expand_dims(dt, 1)

    fig, _ = plt.subplots(figsize=(20, 5))
    for dim in range(state_in.shape[1]):
        plt.subplot(state_in.shape[1], 2, dim * 2 + 1)
        plt.plot(diff[:, dim], color="red")
        if dim == 0:
            plt.title("State Out - State Pred")
        plt.grid("on")
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    for dim in range(state_in.shape[1]):
        plt.subplot(state_in.shape[1], 2, dim * 2 + 2)
        plt.plot(y_all[:, dim], color="green")
        if dim == 0:
            plt.title("(State Out - State Pred) / dt")
        plt.grid("on")
    figures.append(fig)

    plt.show()

    if save_file_path is not None and save_file_name is not None:
        os.makedirs(os.path.join(save_file_path, "plot"), exist_ok=True)
        for i, fig in enumerate(figures):
            fig.savefig(
                os.path.join(save_file_path + "/plot", f"{save_file_name}_{mode}_dataset_plot_{i}.png"),
                dpi=600,
                bbox_inches="tight",
            )


def plot_trajectory(model_options, sim_options, rec_dict, rtnmpc: NeuralMPC, dist_dict=None, save=False):
    figures = []
    state_in = rec_dict["state_in"]
    state_out = rec_dict["state_out"]
    state_prop = rec_dict["state_prop"]
    state_ref = rec_dict["state_ref"]
    control = rec_dict["control"]
    timestamp = rec_dict["timestamp"]

    # Plot state features
    fig, _ = plt.subplots(figsize=(20, 5))
    for dim in range(state_in.shape[1] - 1, -1, -1):
        plt.subplot(state_in.shape[1], 1, dim + 1)
        plt.plot(timestamp, state_ref[:, dim], label="state_ref", color="r", linestyle="--", alpha=0.8)
        plt.plot(timestamp, state_in[:, dim], label="state_in")
        plt.plot(timestamp, state_out[:, dim], label="state_out")
        plt.plot(timestamp, state_prop[:, dim], label="state_prop")
        plt.ylabel(f"D{dim}")
        if dim == 0:
            plt.title("State In & State Out")
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    # Plot control features
    fig, _ = plt.subplots(figsize=(20, 5))
    plt.title("Control input")
    for dim in range(control.shape[1]):
        plt.subplot(control.shape[1], 1, dim+1)
        plt.plot(timestamp, control[:, dim])
        if dim < 4:
            plt.ylabel(f"Thrust {dim+1}")
        else:
            plt.ylabel(f"Servo {dim+1 - 4}")
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != control.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    # Plot in z feature
    fig = plt.figure(figsize=(20, 5))
    plt.plot(timestamp, state_ref[:, 2], label="state_ref", color="r", linestyle="--", alpha=0.8)
    plt.plot(timestamp, state_in[:, 2], label="state_in")
    plt.plot(timestamp, state_out[:, 2], label="state_out")
    plt.plot(timestamp, state_prop[:, 2], label="state_prop")
    plt.grid("on")
    plt.title("Dim 2 zoom in")
    plt.legend()
    plt.xlim(timestamp[0], timestamp[-1])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    # Plot in vz feature
    fig = plt.figure(figsize=(20, 5))
    plt.plot(timestamp, state_ref[:, 5], label="state_ref", color="r", linestyle="--", alpha=0.8)
    plt.plot(timestamp, state_in[:, 5], label="state_in")
    plt.plot(timestamp, state_out[:, 5], label="state_out")
    plt.plot(timestamp, state_prop[:, 5], label="state_prop")
    plt.grid("on")
    plt.title("Dim 5 zoom in")
    plt.legend()
    plt.xlim(timestamp[0], timestamp[-1])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    # Plot computation time
    fig = plt.figure(figsize=(20, 5))
    plt.plot(timestamp, rec_dict["comp_time"])
    plt.plot(
        [0, rec_dict["comp_time"].shape[0]],
        [np.mean(rec_dict["comp_time"]), np.mean(rec_dict["comp_time"])],
        color="tab:red",
        label=f"Avg = {np.mean(rec_dict['comp_time']):.4f} ms",
    )
    plt.xlabel("Simulation time [s]")
    plt.ylabel("Computation time [ms]")
    plt.legend()
    plt.grid()
    plt.xlim(timestamp[0], timestamp[-1])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    # Plot labels for neural network regression
    dt = np.expand_dims(rec_dict["dt"], 1)
    diff = state_out - state_prop
    y_true = diff / dt
    fig, _ = plt.subplots(figsize=(20, 5))
    for dim in range(state_in.shape[1] - 1, -1, -1):
        plt.subplot(state_in.shape[1], 2, dim * 2 + 1)
        plt.plot(timestamp, diff[:, dim])
        plt.ylabel(f"D{dim}")
        if dim == 0:
            plt.title("State Out - State Pred")
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    for dim in range(state_in.shape[1] - 1, -1, -1):
        plt.subplot(state_in.shape[1], 2, dim * 2 + 2)
        plt.plot(timestamp, y_true[:, dim], color="red")
        if dim == 0:
            plt.title("(State Out - State Pred) / dt")
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    # Plot regression of neural network
    if rtnmpc.use_mlp:
        # Transform velocity of state to Body frame
        state_b = state_in.copy()
        if rtnmpc.mlp_metadata["ModelFitConfig"]["input_transform"]:
            for t in range(state_in.shape[0]):
                v_b = v_dot_q(state_in[t, 3:6], quaternion_inverse(state_in[t, 6:10]))
                state_b[t, :] = np.concatenate((state_in[t, :3], v_b, state_in[t, 6:]), axis=0)
        state_b_torch = torch.from_numpy(state_b[:, rtnmpc.state_feats]).type(torch.float32).to(torch.device("cpu"))

        if rtnmpc.mlp_metadata["ModelFitConfig"]["control_averaging"]:
            if {0, 1, 2, 3}.issubset(rtnmpc.u_feats):
                control_in = np.sum(control[:, 0:4], axis=1) / 4.0
            if {4, 5, 6, 7}.issubset(rtnmpc.u_feats):
                control_in = np.concatenate(
                    (control_in[:, np.newaxis], np.sum(control[:, 4:8], axis=1)[:, np.newaxis] / 4.0), axis=1
                )
        else:
            control_in = control[:, rtnmpc.u_feats]
        control_torch = torch.from_numpy(control_in).type(torch.float32).to(torch.device("cpu"))

        mlp_in = torch.cat((state_b_torch, control_torch), dim=1)
        # Forward call MLP
        rtnmpc.neural_model.eval()
        mlp_out = rtnmpc.neural_model(mlp_in).cpu().detach().numpy()

        # Transform velocity back to world frame
        if rtnmpc.mlp_metadata["ModelFitConfig"]["label_transform"]:
            for t in range(state_in.shape[0]):
                if set([3, 4, 5]).issubset(set(rtnmpc.y_reg_dims)):
                    v_idx = np.where(rtnmpc.y_reg_dims == 3)[0][0]  # Assumed that v_x, v_y, v_z are consecutive
                    v_b = mlp_out[t, v_idx : v_idx + 3]
                    v_w = v_dot_q(v_b.T, state_in[t, 6:10]).T
                    mlp_out[t, :] = np.concatenate((mlp_out[t, :v_idx], v_w, mlp_out[t, v_idx + 3 :]), axis=0)
                elif set([4, 5]).issubset(set(rtnmpc.y_reg_dims)):
                    v_idx = np.where(rtnmpc.y_reg_dims == 4)[0][0]  # Assumed that v_y, v_z are consecutive
                    v_b = np.append(0, mlp_out[t, v_idx : v_idx + 2])
                    v_w = v_dot_q(v_b.T, state_in[t, 6:10]).T
                    mlp_out[t, :] = np.concatenate((mlp_out[t, :v_idx], v_w, mlp_out[t, v_idx + 2 :]), axis=0)
                elif set([5]).issubset(set(rtnmpc.y_reg_dims)):
                    # Predict only v_z so set v_x and v_y to 0 in Body frame and then transform to World frame
                    # The predicted v_z therefore also has influence on the x and y velocities in World frame
                    # Adjust mapping later on
                    v_idx = np.where(rtnmpc.y_reg_dims == 5)[0][0]
                    v_b = np.append(np.array([0, 0]), mlp_out[t, v_idx])
                    v_w = v_dot_q(v_b.T, state_in[t, 6:10])[:, np.newaxis]
                    mlp_out[t, :] = np.concatenate((mlp_out[t, :v_idx], v_w, mlp_out[t, v_idx + 1 :]), axis=0)

        # Plot true labels vs. actual regression
        y = mlp_out
        fig, _ = plt.subplots(figsize=(10, 5))
        for i, dim in enumerate(rtnmpc.y_reg_dims):
            plt.subplot(y.shape[1], 1, i + 1)
            plt.plot(timestamp, y[:, i])  # , label="y_regressed")
            # plt.plot(timestamp, y[:, i] - y_true[:, dim], label="error", color="r", linestyle="--", alpha=0.5)
            # plt.plot(timestamp, y_true[:, dim], label="y_true", color="orange")
            plt.ylabel(f"D{dim}")
            # if i == 0:
            #     plt.legend()
            plt.grid("on")
            plt.xlim(timestamp[0], timestamp[-1])
            if i != y.shape[1] - 1:
                ax = plt.gca()
                ax.axes.xaxis.set_ticklabels([])
        plt.title("Model Output")  # vs. Labels")
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figures.append(fig)

        # Plot loss per dimension
        loss = np.square(y_true[:, rtnmpc.y_reg_dims] - y)
        fig, _ = plt.subplots(figsize=(10, 5))
        for i, dim in enumerate(rtnmpc.y_reg_dims):
            plt.subplot(y.shape[1], 1, i + 1)
            plt.plot(timestamp, loss[:, i], color="red")
            plt.plot(
                [timestamp[0], timestamp[-1]],
                [np.mean(loss[:, i]), np.mean(loss[:, i])],
                color="tab:blue",
                linestyle="--",
                label=f"Mean = {np.mean(loss[:, i]):.6f}",
            )
            plt.legend()
            plt.ylabel(f"Loss D{dim}")
            plt.grid("on")
            plt.xlim(timestamp[0], timestamp[-1])
            if i != y.shape[1] - 1:
                ax = plt.gca()
                ax.axes.xaxis.set_ticklabels([])
        plt.title("Neural Model Loss per Dimension")
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figures.append(fig)

        # Plot total loss and RMSE
        fig = plt.figure(figsize=(10, 5))
        plt.title("Neural Model Total Loss and RMSE")
        total_loss = np.sum(loss, axis=1)
        rmse = np.sqrt(total_loss / y.shape[1])
        plt.plot(timestamp, total_loss, label="Total Loss", color="red")
        plt.plot(timestamp, rmse, label="RMSE", color="green")
        plt.plot(
            [timestamp[0], timestamp[-1]],
            [np.mean(total_loss), np.mean(total_loss)],
            color="tab:red",
            linestyle="--",
            alpha=0.7,
            label=f"Mean Total Loss = {np.mean(total_loss):.6f}",
        )
        plt.plot(
            [timestamp[0], timestamp[-1]],
            [np.mean(rmse), np.mean(rmse)],
            color="tab:green",
            linestyle="--",
            alpha=0.7,
            label=f"Mean RMSE = {np.mean(rmse):.6f}",
        )
        plt.xlabel("Time [s]")
        plt.ylabel("Loss / RMSE")
        plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figures.append(fig)

        # Simulate intermediate acceleration vector before integration
        dynamics, _, _ = init_forward_prop(rtnmpc)
        x_dot = np.empty(state_in.shape)
        for t in range(state_in.shape[0]):
            x_dot[t, :] = np.array(dynamics(x=state_in[t, :], u=control[t, :])["x_dot"]).squeeze()
        lin_acc = x_dot[:, 3:6]
        if sim_options["disturbances"]["cog_dist"]:
            lin_acc_dist = lin_acc + dist_dict["cog_dist"][1::2, :3] / rtnmpc.phys.mass

        fig, _ = plt.subplots(figsize=(10, 5))
        plt.title("Model Output")
        for i, dim in enumerate(rtnmpc.y_reg_dims):
            plt.subplot(len(rtnmpc.y_reg_dims), 1, i + 1)
            plt.plot(timestamp, lin_acc[:, i], label="Acceleration by undisturbed model")
            if sim_options["disturbances"]["cog_dist"]:
                plt.plot(timestamp, lin_acc_dist[:, i], label="Acceleration by disturbed model", color="olive")
                plt.plot(timestamp, lin_acc_dist[:, i] + y[:, i], label="Neural Compensation (+)", color="orange")
                plt.plot(timestamp, lin_acc_dist[:, i] - y[:, i], label="Neural Compensation (-)", color="brown")
            else:
                plt.plot(timestamp, lin_acc[:, i] + y[:, i], label="Neural Compensation (+)", color="orange")
            plt.ylabel(f"D{dim}")
            if i == 0:
                plt.legend()
            plt.grid("on")
            plt.xlim(timestamp[0], timestamp[-1])
            if i != y.shape[1] - 1:
                ax = plt.gca()
                ax.axes.xaxis.set_ticklabels([])
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figures.append(fig)

    if save:
        save_dir = DirectoryConfig.SIMULATION_DIR
        safe_mkdir_recursive(save_dir)
        for i, fig in enumerate(figures):
            fig.savefig(os.path.join(save_dir, f"simulation_results_fig{i}.png"), dpi=500, bbox_inches="tight")
    halt = 1


def plot_fitting(total_losses, inference_times, learning_rates, save_file_path=None, save_file_name=None):
    """
    Plot the training and validation losses.
    """
    os.makedirs(os.path.join(save_file_path, "plot"), exist_ok=True)

    fig, axs = plt.subplots(1, 2, figsize=(14, 6))

    ax1 = axs[0]
    ax1.loglog(total_losses["train"], label=f"Train Loss (final = {total_losses['train'][-1]:.4f})", color="blue")
    ax1.loglog(total_losses["val"], label=f"Validation Loss (final = {total_losses['val'][-1]:.4f})", color="orange")
    if "test" in total_losses.keys():
        ax1.loglog(
            [0, len(total_losses["train"])],
            [total_losses["test"], total_losses["test"]],
            label=f"Test Loss = {total_losses['test']:.4f}",
            color="green",
        )
    # ax1.set_xlim([0, len(total_losses["train"])])
    ax1.set_xlabel("Epochs")
    ax1.set_ylabel("Loss")
    ax1.set_title(f"Losses")
    ax1.grid()
    ax1.legend(loc="upper left")
    ax1_right = ax1.twinx()
    ax1_right.plot(learning_rates, label="Learning Rate", color="red", alpha=0.7)
    ax1_right.set_ylabel("Learning Rate")
    ax1_right.legend(loc="upper right")

    ax2 = axs[1]
    ax2.plot(inference_times)
    mean = np.mean(inference_times)
    ax2.plot([0, len(inference_times)], [mean, mean], label=f"Avg = {mean:.2f} ms", color="red")
    ax2.set_xlim([0, len(inference_times)])
    ax2.set_xlabel("Epochs")
    ax2.set_ylabel("Inference Time (ms)")
    ax2.set_title("Inference Times")
    ax2.grid()
    ax2.legend()
    # Adjust layout and display
    plt.tight_layout()
    if save_file_path is not None and save_file_name is not None:
        fig.savefig(os.path.join(save_file_path + "/plot", f"{save_file_name}_plot.png"), dpi=300, bbox_inches="tight")
    plt.show()


def plot_disturbances(dist_dict, save=False):
    figures = []
    # CoG disturbance
    if "cog_dist" in dist_dict:
        fig, _ = plt.subplots(2, 1, figsize=(20, 5))
        plt.subplot(2, 1, 1)
        ax = plt.gca()
        ax.plot(dist_dict["timestamp"], dist_dict["cog_dist"][:, :3], alpha=0.7, label=["x", "y", "z"])
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("cog_dist force [N]")
        ax.grid()
        ax.set_xlim([0, dist_dict["timestamp"][-1]])
        ax_right = ax.twinx()
        ax_right.plot(dist_dict["timestamp"], dist_dict["z"], label="height")
        ax_right.set_ylabel("Height z [m]")
        plt.legend()
        plt.tight_layout()

        plt.subplot(2, 1, 2)
        ax = plt.gca()
        ax.plot(dist_dict["timestamp"], dist_dict["cog_dist"][:, 3:], alpha=0.7, label=["x", "y", "z"])
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("cog_dist moment [Nm]")
        ax.grid()
        ax.set_xlim([0, dist_dict["timestamp"][-1]])
        ax_right = ax.twinx()
        ax_right.plot(dist_dict["timestamp"], dist_dict["z"], label="height")
        ax_right.set_ylabel("Height z [m]")
        plt.legend()
        plt.tight_layout()
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figures.append(fig)

    # Motor noise
    if "motor_noise" in dist_dict:
        fig, _ = plt.subplots(figsize=(20, 5))
        plt.plot(dist_dict["timestamp"], dist_dict["motor_noise"])
        plt.ylabel("motor_noise [N]")
        plt.xlabel("Time [s]")
        plt.grid()
        plt.tight_layout()
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figures.append(fig)

    if save:
        save_dir = DirectoryConfig.SIMULATION_DIR
        safe_mkdir_recursive(save_dir)
        for i, fig in enumerate(figures):
            fig.savefig(os.path.join(save_dir, f"disturbance_fig{i}.png"), dpi=500, bbox_inches="tight")


def trajectory_tracking_results(
    t_ref, x_ref, x_executed, u_ref, u_executed, title, w_control=None, legend_labels=None, quat_error=True
):
    if legend_labels is None:
        legend_labels = ["reference", "simulated"]

    with_ref = True if x_ref is not None else False

    fig, ax = plt.subplots(3, 4, sharex="all", figsize=(7, 9))

    SMALL_SIZE = 8
    MEDIUM_SIZE = 10
    BIGGER_SIZE = 12

    plt.rc("font", size=SMALL_SIZE)  # controls default text sizes
    plt.rc("axes", titlesize=SMALL_SIZE)  # fontsize of the axes title
    plt.rc("axes", labelsize=MEDIUM_SIZE)  # fontsize of the x and y labels
    plt.rc("xtick", labelsize=SMALL_SIZE)  # fontsize of the tick labels
    plt.rc("ytick", labelsize=SMALL_SIZE)  # fontsize of the tick labels
    plt.rc("legend", fontsize=SMALL_SIZE)  # legend fontsize
    plt.rc("figure", titlesize=BIGGER_SIZE)  # fontsize of the figure title

    labels = ["x", "y", "z"]
    for i in range(3):
        ax[i, 0].plot(t_ref, x_executed[:, i], label=legend_labels[1])
        if with_ref:
            ax[i, 0].plot(t_ref, x_ref[:, i], label=legend_labels[0])
        ax[i, 0].legend()
        ax[i, 0].set_ylabel(labels[i])
    ax[0, 0].set_title(r"$p\:[m]$")
    ax[2, 0].set_xlabel(r"$t [s]$")

    q_euler = np.stack([quaternion_to_euler(x_executed[j, 3:7]) for j in range(x_executed.shape[0])])
    for i in range(3):
        ax[i, 1].plot(t_ref, q_euler[:, i], label=legend_labels[1])
    if with_ref:
        ref_euler = np.stack([quaternion_to_euler(x_ref[j, 3:7]) for j in range(x_ref.shape[0])])
        q_err = []
        for i in range(t_ref.shape[0]):
            q_err.append(q_dot_q(x_executed[i, 3:7], quaternion_inverse(x_ref[i, 3:7])))
        q_err = np.stack(q_err)

        for i in range(3):
            ax[i, 1].plot(t_ref, ref_euler[:, i], label=legend_labels[0])
            if quat_error:
                ax[i, 1].plot(t_ref, q_err[:, i + 1], label="quat error")
    for i in range(3):
        ax[i, 1].legend()
    ax[0, 1].set_title(r"$\theta\:[rad]$")
    ax[2, 1].set_xlabel(r"$t [s]$")

    for i in range(3):
        ax[i, 2].plot(t_ref, x_executed[:, i + 7], label=legend_labels[1])
        if with_ref:
            ax[i, 2].plot(t_ref, x_ref[:, i + 7], label=legend_labels[0])
        ax[i, 2].legend()
    ax[0, 2].set_title(r"$v\:[m/s]$")
    ax[2, 2].set_xlabel(r"$t [s]$")

    for i in range(3):
        ax[i, 3].plot(t_ref, x_executed[:, i + 10], label=legend_labels[1])
        if with_ref:
            ax[i, 3].plot(t_ref, x_ref[:, i + 10], label=legend_labels[0])
        if w_control is not None:
            ax[i, 3].plot(t_ref, w_control[:, i], label="control")
        ax[i, 3].legend()
    ax[0, 3].set_title(r"$\omega\:[rad/s]$")
    ax[2, 3].set_xlabel(r"$t [s]$")

    plt.suptitle(title)

    if u_ref is not None and u_executed is not None:
        ax = plt.subplots(1, 4, sharex="all", sharey="all")[1]
        for i in range(4):
            ax[i].plot(t_ref, u_ref[:, i], label="ref")
            ax[i].plot(t_ref, u_executed[:, i], label="simulated")
            ax[i].set_xlabel(r"$t [s]$")
            tit = "Control %d" % (i + 1)
            ax[i].set_title(tit)
            ax[i].legend()

    dir_path = os.path.dirname(os.path.realpath(__file__))
    img_save_dir = dir_path + "/../../results/images/"
    safe_mkdir_recursive(img_save_dir, overwrite=False)
    fig.savefig(
        img_save_dir + "mse_exp",
        dpi=None,
        facecolor="w",
        edgecolor="w",
        orientation="portrait",
        transparent=False,
        pad_inches=0.1,
    )


def mse_tracking_experiment_plot(
    v_max, mse, traj_type_vec, train_samples_vec, legends, y_labels, t_opt=None, font_size=16
):
    # Check if there is the variants dimension in the data
    if len(mse.shape) == 4:
        variants_dim = mse.shape[3]
    else:
        variants_dim = 1

    fig, axes = plt.subplots(
        variants_dim, len(traj_type_vec), sharex="col", sharey="none", figsize=(17, 2.5 * variants_dim + 2)
    )
    if variants_dim == 1 and len(traj_type_vec) > 1:
        axes = axes[np.newaxis, :]
    elif variants_dim == 1:
        axes = np.expand_dims(axes, 0)
        axes = np.expand_dims(axes, 0)
    elif len(traj_type_vec) == 1:
        axes = axes[:, np.newaxis]

    for seed_id, track_seed in enumerate(traj_type_vec):
        for j in range(variants_dim):
            for i, _ in enumerate(train_samples_vec):
                mse_data = mse[seed_id, :, i, j] if len(mse.shape) == 4 else mse[seed_id, :, i]
                label = legends[i] if seed_id == 0 and j == 0 else None
                if legends[i] == "perfect":
                    axes[j, seed_id].plot(v_max[seed_id, :], mse_data, "--o", linewidth=4, label=label)
                else:
                    axes[j, seed_id].plot(v_max[seed_id, :], mse_data, "--o", label=label)
            if seed_id == 0:
                axes[j, seed_id].set_ylabel(y_labels[j], size=font_size)
            if j == 0:
                axes[j, seed_id].set_title("RMSE [m] | " + str(track_seed), size=font_size + 2)

            axes[j, seed_id].grid()
            axes[j, seed_id].tick_params(labelsize=font_size)

        axes[variants_dim - 1, seed_id].set_xlabel("max vel [m/s]", size=font_size)

    legend_cols = len(train_samples_vec)
    fig.legend(
        loc="upper center", fancybox=True, borderaxespad=0.05, ncol=legend_cols, mode="expand", fontsize=font_size - 4
    )
    plt.tight_layout(h_pad=1.4)
    plt.subplots_adjust(top=0.7 + 0.05 * variants_dim)

    dir_path = os.path.dirname(os.path.realpath(__file__))
    img_save_dir = dir_path + "/../../results/images/"
    safe_mkdir_recursive(img_save_dir, overwrite=False)

    try:
        tikzplotlib.save(img_save_dir + "mse.tex")
    except:
        pass
    fig.savefig(
        img_save_dir + "mse",
        dpi=None,
        facecolor="w",
        edgecolor="w",
        orientation="portrait",
        transparent=False,
        pad_inches=0.1,
    )

    if t_opt is None:
        return

    v = v_max.reshape(-1)
    ind_v = np.argsort(v, axis=0)

    fig = plt.figure(figsize=(17, 4.5))
    for i, n_train in enumerate(train_samples_vec):
        plt.plot(v[ind_v], t_opt.reshape(t_opt.shape[0] * t_opt.shape[1], -1)[ind_v, i], label=legends[i])
    fig.legend(
        loc="upper center", fancybox=True, borderaxespad=0.05, ncol=legend_cols, mode="expand", fontsize=font_size
    )
    plt.ylabel("Mean MPC loop time (s)", fontsize=font_size)
    plt.xlabel("Max vel [m/s]", fontsize=font_size)

    try:
        tikzplotlib.save(img_save_dir + "t_opt.tex")
    except:
        pass
    fig.savefig(
        img_save_dir + "t_opt",
        dpi=None,
        facecolor="w",
        edgecolor="w",
        orientation="portrait",
        transparent=False,
        bbox_inches=None,
        pad_inches=0.1,
    )


# def draw_covariance_ellipsoid(center, covar):
#     """
#     :param center: 3-dimensional array. Center of the ellipsoid
#     :param covar: 3x3 covariance matrix. If the covariance is diagonal, the ellipsoid will have radii equal to the
#     three diagonal axis along axes x, y, z respectively.
#     :return:
#     """

#     # find the rotation matrix and radii of the axes
#     _, radii, rotation = np.linalg.svd(covar)

#     # now carry on with EOL's answer
#     u = np.linspace(0.0, 2.0 * np.pi, 20)
#     v = np.linspace(0.0, np.pi, 20)
#     x = radii[0] * np.outer(np.cos(u), np.sin(v))
#     y = radii[1] * np.outer(np.sin(u), np.sin(v))
#     z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
#     for i in range(len(x)):
#         for j in range(len(x)):
#             [x[i, j], y[i, j], z[i, j]] = np.dot([x[i, j], y[i, j], z[i, j]], rotation) + center

#     x = np.reshape(x, -1)
#     y = np.reshape(y, -1)
#     z = np.reshape(z, -1)
#     return x, y, z


# def visualize_data_distribution(x_data, y_data, clusters, x_pruned, y_pruned):
#     """
#     Visualizes the distribution of the training dataset and the assignation of the GP prediction clusters.
#     :param x_data: numpy array of shape N x 3, where N is the number of training points. Feature variables.
#     :param y_data: numpy array of shape N x 3, where N is the number of training points. Regressed variables.
#     :param x_pruned: numpy array of shape M x 3, where M is the number of pruned training points. Feature variables.
#     :param y_pruned: numpy array of shape M x 3, where M is the number of pruned training points. Regressed variables.
#     :param clusters: A dictionary where each entry is indexed by the cluster number, and contains a list of all the
#     indices of the points in x_pruned belonging to that cluster.
#     """

#     if x_data.shape[1] < 3:
#         return

#     fig = plt.figure()
#     ax = fig.add_subplot(131, projection='3d')
#     c = np.sqrt(np.sum(y_data ** 2, 1))
#     scatter = ax.scatter(x_data[:, 0], x_data[:, 1], x_data[:, 2], c=c, alpha=0.6)
#     ax.set_title('Raw data: Correction magnitude')
#     ax.set_xlabel(r'$v_x\: [m/s]$')
#     ax.set_ylabel(r'$v_y\: [m/s]$')
#     ax.set_zlabel(r'$v_z\: [m/s]$')
#     fig.colorbar(scatter, ax=ax, orientation='vertical', shrink=0.75)

#     ax = fig.add_subplot(132, projection='3d')
#     c = np.sqrt(np.sum(y_pruned ** 2, 1))
#     scatter = ax.scatter(x_pruned[:, 0], x_pruned[:, 1], x_pruned[:, 2], c=c, alpha=0.6)
#     ax.set_title('Pruned data: Correction magnitude')
#     ax.set_xlabel(r'$v_x\: [m/s]$')
#     ax.set_ylabel(r'$v_y\: [m/s]$')
#     ax.set_zlabel(r'$v_z\: [m/s]$')
#     fig.colorbar(scatter, ax=ax, orientation='vertical', shrink=0.75)

#     n_clusters = len(clusters.keys())

#     ax = fig.add_subplot(133, projection='3d')
#     for i in range(int(n_clusters)):
#         ax.scatter(x_pruned[clusters[i], 0], x_pruned[clusters[i], 1], x_pruned[clusters[i], 2], alpha=0.6)
#     ax.set_title('Cluster assignations')
#     ax.set_xlabel(r'$v_x\: [m/s]$')
#     ax.set_ylabel(r'$v_y\: [m/s]$')
#     ax.set_zlabel(r'$v_z\: [m/s]$')

#     plt.show()


# def visualize_gp_inference(x_data, u_data, y_data, gp_ensemble, vis_features_x, y_dims, labels):
#     # WARNING: This function is extremely limited to the case where the regression is performed using just the
#     # velocity state as input features and as output dimensions.

#     predictions = gp_ensemble.predict(x_data.T, u_data.T)
#     predictions = np.atleast_2d(np.atleast_2d(predictions["pred"])[y_dims])

#     if len(vis_features_x) > 1:
#         y_pred = np.sqrt(np.sum(predictions ** 2, 0))
#         y_mse = np.sqrt(np.sum(y_data ** 2, 1))
#     else:
#         y_pred = predictions[0, :]
#         y_mse = y_data[:, 0]

#     v_min = min(np.min(y_pred), np.min(y_mse))
#     v_max = max(np.max(y_pred), np.max(y_mse))

#     fig = plt.figure()

#     font_size = 16

#     if len(vis_features_x) == 1:
#         # Feature dimension is only 1

#         # Compute windowed average
#         n_bins = 20
#         _, b = np.histogram(x_data[:, vis_features_x], bins=n_bins)
#         hist_indices = np.digitize(x_data[:, vis_features_x], b)
#         win_average = np.zeros(n_bins)
#         for i in range(n_bins):
#             win_average[i] = np.mean(y_mse[np.where(hist_indices == i + 1)[0]])
#         bin_midpoints = b[:-1] + np.diff(b)[0] / 2

#         ax = [fig.add_subplot(121), fig.add_subplot(122)]

#         ax[0].scatter(x_data[:, vis_features_x], y_mse)
#         ax[0].set_xlabel(labels[0])
#         ax[0].set_ylabel('RMSE')
#         ax[0].set_title('Post-processed dataset')

#         ax[1].scatter(x_data[:, vis_features_x], y_pred, label='GP')
#         ax[1].plot(bin_midpoints, win_average, label='window average')
#         ax[1].set_xlabel(labels[0])
#         ax[1].set_title('Predictions')
#         ax[1].legend()

#         return

#     elif len(vis_features_x) >= 3:
#         ax = [fig.add_subplot(121, projection='3d'), fig.add_subplot(122, projection='3d')]
#         im = ax[0].scatter(x_data[:, vis_features_x[0]], x_data[:, vis_features_x[1]], x_data[:, vis_features_x[2]], c=y_mse,
#                            cmap='viridis', alpha=0.6, vmin=v_min, vmax=v_max)
#         ax[0].set_xlabel(labels[0], size=font_size - 4, labelpad=10)
#         ax[0].set_ylabel(labels[1], size=font_size - 4, labelpad=10)
#         ax[0].set_zlabel(labels[2], size=font_size - 4, labelpad=10)
#         ax[0].set_title(r'Nominal MPC error $\|\mathbf{a}^e\|$', size=font_size)
#         ax[0].view_init(65, 15)

#         ax[1].scatter(x_data[:, vis_features_x[0]], x_data[:, vis_features_x[1]], x_data[:, vis_features_x[2]], c=y_pred,
#                       cmap='viridis', alpha=0.6, vmin=v_min, vmax=v_max)
#         ax[1].set_xlabel(labels[0], size=font_size - 4, labelpad=10)
#         ax[1].set_ylabel(labels[1], size=font_size - 4, labelpad=10)
#         ax[1].set_zlabel(labels[2], size=font_size - 4, labelpad=10)
#         ax[1].set_title(r'GP prediction mangnitude $\|\tilde{\mathbf{a}}^e\|$', size=font_size)
#         ax[1].view_init(65, 15)

#         plt.tight_layout()
#         fig.subplots_adjust(right=0.85)
#         cbar = fig.add_axes([0.90, 0.05, 0.03, 0.8])
#         fig.colorbar(im, cax=cbar)
#         cbar.get_yaxis().labelpad = 15
#         cbar.set_ylabel(r'$\|\mathbf{a}^e\|\left[\frac{m}{s^2}\right]$', size=font_size, labelpad=20, rotation=270)
#         cbar.tick_params(labelsize=font_size - 4)

#     # Create values for the regressed variables
#     x = np.linspace(min(x_data[:, vis_features_x[0]]), max(x_data[:, vis_features_x[0]]), 100)
#     y = np.linspace(min(x_data[:, vis_features_x[1]]), max(x_data[:, vis_features_x[1]]), 100)
#     # x = np.linspace(-8, 8, 50)
#     # y = np.linspace(-8, 8, 50)
#     x_mesh, y_mesh = np.meshgrid(x, y)
#     x = np.reshape(x_mesh, (-1, 1))
#     y = np.reshape(y_mesh, (-1, 1))
#     z = np.zeros_like(x)
#     x_sample = np.concatenate((x, y, z), 1)

#     # Generate complete mock x features. Only vis_features are non-zero
#     x_mock = np.tile(np.zeros_like(z), (1, x_data.shape[1]))
#     x_mock[:, np.array(vis_features_x)] = x_sample

#     # Also create mock u features
#     u_mock = np.tile(np.zeros_like(z), (1, u_data.shape[1]))

#     if len(vis_features_x) != 3:
#         plt.show()
#         return

#     # Generate animated plot showing prediction of the multiple clusters.
#     # Cluster coloring only possible if all the output dimensions have exactly the same clusters.
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     print("Grid sampling...")
#     outs = gp_ensemble.predict(x_mock.T, u_mock.T, return_gp_id=True, progress_bar=True)
#     y_pred = np.atleast_2d(np.atleast_2d(outs["pred"])[y_dims])
#     gp_ids = outs["gp_id"]
#     y_sample = np.sqrt(np.sum(y_pred ** 2, 0))
#     y_sample = np.reshape(y_sample, x_mesh.shape)

#     gp_ids = np.reshape(gp_ids[next(iter(gp_ids))], x_mesh.shape)

#     def init():
#         # create the new map
#         cmap = cm.get_cmap('jet')
#         cmaplist = [cmap(j) for j in range(cmap.N)]
#         cmap = LinearSegmentedColormap.from_list('Custom cmap', cmaplist, cmap.N)

#         # define the bins and normalize
#         capped_n_clusters = min(np.amax(gp_ids) + 2, 20)
#         bounds = np.linspace(0, np.amax(gp_ids) + 1, capped_n_clusters)
#         norm = BoundaryNorm(bounds, cmap.N)

#         my_col = cm.get_cmap('jet')(gp_ids / (np.amax(gp_ids) + 1))

#         ax.plot_surface(x_mesh, y_mesh, y_sample, facecolors=my_col, linewidth=0, rstride=1, cstride=1,
#                         antialiased=False, alpha=0.7, cmap=cmap, norm=norm)
#         ax2 = fig.add_axes([0.90, 0.2, 0.03, 0.6])
#         ColorbarBase(ax2, cmap=cmap, norm=norm, spacing='proportional', ticks=bounds, boundaries=bounds, format='%1i')
#         ax2.set_ylabel('Cluster assignment ID', size=14)
#         ax2.tick_params(labelsize=16)

#         ax.tick_params(labelsize=14)
#         ax.set_xlabel(labels[0], size=16, labelpad=10)
#         ax.set_ylabel(labels[1], size=16, labelpad=10)
#         ax.set_zlabel(r'$\|\tilde{\mathbf{a}}^e\|\: \left[\frac{m}{s^2}\right]$', size=16, labelpad=10)
#         ax.set_title(r'GP correction. Slice $v_z=0 \:\: \left[\frac{m}{s}\right]$', size=18)
#         return fig,

#     def animate(i):
#         ax.view_init(elev=30., azim=i*3)
#         return fig

#     _ = animation.FuncAnimation(fig, animate, init_func=init, frames=360, interval=20, blit=False)

#     plt.show()

# def load_past_experiments():

#     metadata_file, mse_file, v_file, t_opt_file = get_experiment_files()

#     try:
#         with open(metadata_file) as json_file:
#             metadata = json.load(json_file)
#     except:
#         metadata = None

#     mse = np.load(mse_file)
#     v = np.load(v_file)
#     t_opt = np.load(t_opt_file)

#     return metadata, mse, v, t_opt


# def get_experiment_files():
#     results_path = PathConfig.RESULTS_DIR
#     metadata_file = os.path.join(results_path, 'experiments', 'metadata.json')
#     mse_file = os.path.join(results_path, 'experiments', 'mse.npy')
#     mean_v_file = os.path.join(results_path, 'experiments', 'mean_v.npy')
#     t_opt_file = os.path.join(results_path, 'experiments', 't_opt.npy')

#     if not os.path.exists(metadata_file):
#         safe_mknode_recursive(os.path.join(results_path, 'experiments'), 'metadata.json', overwrite=False)
#     if not os.path.exists(mse_file):
#         safe_mknode_recursive(os.path.join(results_path, 'experiments'), 'mse.npy', overwrite=False)
#     if not os.path.exists(mean_v_file):
#         safe_mknode_recursive(os.path.join(results_path, 'experiments'), 'mean_v.npy', overwrite=False)
#     if not os.path.exists(t_opt_file):
#         safe_mknode_recursive(os.path.join(results_path, 'experiments'), 't_opt.npy', overwrite=False)

#     return metadata_file, mse_file, mean_v_file, t_opt_file

# def angle_to_rot_mat(angle):
#     """
#     Computes the 2x2 rotation matrix from the scalar angle
#     :param angle: scalar angle in radians
#     :return: the corresponding 2x2 rotation matrix
#     """

#     s = np.sin(angle)
#     c = np.cos(angle)
#     return np.array([[c, -s], [s, c]])


# def draw_arrow(x_base, y_base, x_body, y_body):
#     """
#     Returns the coordinates for drawing a 2D arrow given its origin point and its length.
#     :param x_base: x coordinate of the arrow origin
#     :param y_base: y coordinate of the arrow origin
#     :param x_body: x length of the arrow
#     :param y_body: y length of the arrow
#     :return: a tuple of x, y coordinates to plot the arrow
#     """

#     len_arrow = np.sqrt(x_body ** 2 + y_body ** 2)
#     beta = np.arctan2(y_body, x_body)
#     beta_rot = angle_to_rot_mat(beta)
#     lower_arrow = beta_rot.dot(np.array([[-np.cos(np.pi / 6)], [-np.sin(np.pi / 6)]]) * len_arrow / 3)
#     upper_arrow = beta_rot.dot(np.array([[-np.cos(np.pi / 6)], [np.sin(np.pi / 6)]]) * len_arrow / 3)

#     return ([x_base, x_base + x_body, x_base + x_body + lower_arrow[0, 0],
#              x_base + x_body, x_base + x_body + upper_arrow[0, 0]],
#             [y_base, y_base + y_body, y_base + y_body + lower_arrow[1, 0],
#              y_base + y_body, y_base + y_body + upper_arrow[1, 0]])

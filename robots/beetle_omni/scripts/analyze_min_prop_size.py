"""
 Created by li-jinjie on 2026/3/2.
"""
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

# N is 3,4,5,6,..., a int list
N_START = 3
N_END = 12
N_STEP = 1

N = np.linspace(N_START, N_END, int((N_END - N_START) / N_STEP) + 1, dtype=int)

theta = 2 * np.pi / N

# Compute the minimum propeller size (distance from center to rotor) for each N
prop_to_frame_ratio_no_ae = (1 - np.cos(theta)) / 2
prop_to_frame_ratio_no_collision = np.cos((np.pi - theta) / 2)

# Set up the plot style
plt.style.use(["science", "grid"])
plt.rcParams.update({"font.size": 12})
label_size = 13

# plot prop_to_frame_ratio to N
plt.figure(figsize=(8, 3))

# Fill regions with colors
# Light red above prop_to_frame_ratio_no_collision
plt.fill_between(N, prop_to_frame_ratio_no_collision, 1.0, alpha=0.3, color="lightcoral", label="Collision Risk Zone")

# Light blue between the two curves
plt.fill_between(
    N,
    prop_to_frame_ratio_no_ae,
    prop_to_frame_ratio_no_collision,
    alpha=0.3,
    color="lightblue",
    label="Aero-Interference Risk Zone",
)

# Light green below prop_to_frame_ratio_no_ae
plt.fill_between(N, 0, prop_to_frame_ratio_no_ae, alpha=0.3, color="lightgreen", label="Safe Zone")

# Plot the curves
plt.plot(N, prop_to_frame_ratio_no_collision, marker="o", linewidth=2, label="No Collision Boundary")
plt.plot(N, prop_to_frame_ratio_no_ae, marker="x", linewidth=2, label="No Aero-Interference Boundary")

# our design point: N=4, R/l=0.2286/2/0.275
plt.scatter(4, 0.2286 / 2 / 0.275, marker="*", color="red", s=150, label="Ours ($N_p=4$, $R_p/l=0.416$)")

# plt.title("Minimum Propeller Size to Frame Size Ratio vs Number of Rotors")
plt.xlabel("Number of Tiltable-Rotors $N_p$", fontsize=label_size)
plt.ylabel("Propeller Size $R_p$ / Frame Size $l$", fontsize=label_size)

plt.legend(ncol=1, fontsize=label_size - 2, framealpha=0.85)

# # set legend transparency
# legend = plt.gca().get_legend()
# legend.get_frame().set_alpha(0.9)

plt.xticks(N)
plt.grid(alpha=0.5)
plt.xlim(N[0], N[-1])
plt.ylim(0, 1)
plt.show()

import numpy as np
import matplotlib.pyplot as plt
import scienceplots

our_R_div_l = 0.2286 / 2 / 0.275

R_div_l_set = [0.3, 0.5, 0.7, our_R_div_l]

# 0..180 deg
lambdas_deg = np.linspace(0.0, 180.0, 721)
lambdas = np.deg2rad(lambdas_deg)

d_div_l_grow = np.where(lambdas < np.pi / 2, np.sqrt(2) / 2 * np.cos(lambdas), np.sqrt(2) / 2 * np.cos(np.pi - lambdas))
rod_div_l_grow = np.where(
    lambdas < np.pi / 2, np.sqrt(0.5 * (1 + np.sin(lambdas) ** 2)), np.sqrt(0.5 * (1 + np.sin(np.pi - lambdas) ** 2))
)

# ============== Plotting ==============
plt.style.use(["science", "grid"])
plt.rcParams.update({"font.size": 12})
label_size = 13

fig, ax1 = plt.subplots(figsize=(8, 3.7))
for R_div_l in R_div_l_set:
    if not np.isclose(R_div_l, our_R_div_l):
        ax1.plot(lambdas_deg, d_div_l_grow + R_div_l, label=f"$R_p/l$={R_div_l:.1f}")
    else:
        ax1.plot(lambdas_deg, d_div_l_grow + R_div_l, label=f"$R_p/l$={R_div_l:.3f} (Ours)")

ax1.set_xlabel("Installing Angle $\\lambda$ [$^\circ$]", fontsize=label_size)
ax1.set_ylabel("Min. Operation Dist. $d$ / Frame Size $l$", fontsize=label_size)
ax1.grid(True)
plt.legend(title="Left Axis", loc="lower left", fontsize=label_size - 2, framealpha=0.5)

ax2 = ax1.twinx()
for R_div_l in R_div_l_set:
    if not np.isclose(R_div_l, our_R_div_l):
        ax2.plot(lambdas_deg, rod_div_l_grow - R_div_l, label=f"$R_p/l$={R_div_l:.1f}", linestyle="dashed")
    else:
        ax2.plot(lambdas_deg, rod_div_l_grow - R_div_l, label=f"$R_p/l$={R_div_l:.3f}", linestyle="dashed")
ax2.set_ylabel("Max. Rod Radius $R_{\\rm rod}$ / Frame Size $l$", fontsize=label_size)
plt.legend(title="Right Axis", loc="upper right", fontsize=label_size - 2, framealpha=0.5)

# plt.title("X-config: h_min and allowable support radius vs tilt angle (0–180°, R/l=0.5)")

# legend
plt.xlim(0, 180)
# h_line = plt.Line2D([], [], color="blue", label="Minimum operating distance $h_{min}$")
# r_line = plt.Line2D([], [], color="orange", label="Max support cylinder radius $r_{max}$")
# plt.legend(handles=[h_line, r_line], fontsize=label_size - 2, loc="upper right")

plt.tight_layout()
plt.show()

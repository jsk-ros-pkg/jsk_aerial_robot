'''
 Created by li-jinjie on 24-6-15.
 The data is for beetle-art
'''
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

legend_alpha = 0.5

servo_angle = np.array([-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]) * np.pi / 180
kt = np.array(
    [0.1141, 0.1123, 0.1122, 0.1143, 0.1171, 0.1191, 0.1203, 0.1204, 0.1197, 0.1152, 0.1153, 0.1146, 0.1163])
kq_d_kt = np.array(
    [-0.0165, -0.0151, -0.0151, -0.0156, -0.0129, -0.0162, -0.0158, -0.0155, -0.0183, -0.0170, -0.0167, -0.0159,
     -0.0156])

resis_d_thrust = (kt - 0.1203) / 0.1203 * 100

# draw two figures
# plot fz_krpm2_slope with respect to servo_angle
plt.style.use(["science", "grid"])
plt.rcParams.update({'font.size': 11})  # default is 10
label_size = 12

plt.figure(figsize=(5.5, 4.5))
# two subplots sharing the same x-axis

ax1 = plt.subplot(211)
plt.plot(servo_angle * 180 / np.pi, kt, 'o-', label='$k_t$ w/ mounting')
plt.plot(0, 0.1495, 'ro', label='$k_t$ w/o mounting')
# the interval for the x-axis is 15 degrees
# plt.xticks(np.arange(-90, 91, 15))
# plt.xlabel('Servo Angle ($^\circ$)', fontsize=label_size)
# plt.xlim([-90, 90])
plt.ylabel('$k_t$ (N/kRPM$^2$)', fontsize=label_size)
plt.ylim([0.0, 0.16])
plt.legend(framealpha=legend_alpha, loc="center left")
# plt.setp(ax1.get_xticklabels(), visible=False)

# draw kq_d_kt with respect to servo_angle at the same figure
ax = plt.gca()
ax2 = ax.twinx()
ax2.plot(servo_angle * 180 / np.pi, -kq_d_kt, '*-', label='$k_q/k_t$ w/ mounting')
ax2.plot(0, --0.0153, 'r*', label='$k_q/k_t$ w/o mounting')
ax2.set_ylabel('$k_q/k_t$', fontsize=label_size)
ax2.set_ylim([0.0, 0.16])
ax2.legend(framealpha=legend_alpha, loc="center right")

# plot resis_d_thrust with respect to servo_angle
plt.subplot(212, sharex=ax1)
plt.plot(servo_angle * 180 / np.pi, resis_d_thrust, 'o-', label='ground truth')

# use 3rd order polynomial to fit the data
fit = np.polyfit(servo_angle, resis_d_thrust, 3)
fit_fn = np.poly1d(fit)
plt.plot(servo_angle * 180 / np.pi, fit_fn(servo_angle), ':', label='3rd-order poly.')

# use 4th order polynomial to fit the data
fit = np.polyfit(servo_angle, resis_d_thrust, 4)
fit_fn = np.poly1d(fit)
plt.plot(servo_angle * 180 / np.pi, fit_fn(servo_angle), '-.', label='4th-order poly.')

# use 5th order polynomial to fit the data
fit = np.polyfit(servo_angle, resis_d_thrust, 5)
fit_fn = np.poly1d(fit)
plt.plot(servo_angle * 180 / np.pi, fit_fn(servo_angle), '--', label='5th-order poly.')

# the interval for the x-axis is 15 degrees
plt.xticks(np.arange(-90, 91, 15))
plt.xlim([-90, 90])
plt.xlabel('Servo Angle ($^\circ$)', fontsize=label_size)
plt.ylabel('$f_{i,re}/f_i$ (\%)', fontsize=label_size)

plt.legend(fontsize=10, framealpha=legend_alpha, loc="lower center")

plt.tight_layout()

plt.show()

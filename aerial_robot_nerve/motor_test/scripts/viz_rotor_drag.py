'''
 Created by li-jinjie on 24-6-15.
 The data is for beetle-art
'''
import numpy as np
import matplotlib.pyplot as plt
import scienceplots

legend_alpha = 0.5

servo_angle = np.array([-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]) * np.pi / 180

fz = np.zeros([len(servo_angle), 16])
fz[0, :] = np.array(
    [0.1065, 0.4619, 1.0143, 1.5447, 2.2835, 3.0810, 4.1092, 5.0486, 6.1573, 7.5045, 9.0324, 10.3618, 11.6660, 12.9104,
     14.2706, 15.6455])
fz[1, :] = np.array(
    [0.1344, 0.4264, 0.9136, 1.6300, 2.3550, 3.0753, 4.0768, 5.1582, 6.1925, 7.4956, 8.8761, 10.1407, 11.5419, 12.7543,
     14.1690, 15.5922])
fz[2, :] = np.array(
    [0.1767, 0.4836, 0.9218, 1.6407, 2.2553, 3.0047, 4.0327, 5.1743, 6.2267, 7.4353, 8.6992, 10.4299, 11.4561, 12.8858,
     14.0372, 15.4279])
fz[3, :] = np.array(
    [0.1500, 0.5556, 0.9640, 1.6874, 2.2017, 3.1750, 4.0757, 5.1170, 6.2628, 7.7545, 9.0413, 10.3077, 11.5813, 13.0812,
     14.1720, 15.9602])
fz[4, :] = np.array(
    [0.1938, 0.5258, 0.9726, 1.6786, 2.3927, 3.2131, 4.3062, 5.3638, 6.7076, 7.8390, 9.0679, 10.7319, 12.1199, 13.4070,
     14.7612, 16.1906])
fz[5, :] = np.array(
    [0.1253, 0.5087, 0.9658, 1.6234, 2.4528, 3.1856, 4.3385, 5.3359, 6.5216, 8.0621, 9.4246, 10.7791, 12.1477, 13.5716,
     14.9918, 16.3925])
fz[6, :] = np.array(
    [0.1230, 0.5031, 1.0037, 1.7963, 2.3913, 3.2056, 4.3240, 5.3628, 6.5365, 8.0607, 9.4551, 10.9254, 12.1107, 13.7112,
     15.3385, 16.6075])
fz[7, :] = np.array(
    [0.1880, 0.5623, 0.9659, 1.6886, 2.4373, 3.3266, 4.2812, 5.4870, 6.7129, 8.2265, 9.5994, 10.9430, 12.3585, 13.7272,
     15.2585, 16.6534])
fz[8, :] = np.array(
    [0.1695, 0.5615, 1.0080, 1.7692, 2.4756, 3.3525, 4.3695, 5.5268, 6.8614, 8.2323, 9.5095, 10.8827, 12.3028, 13.8343,
     15.2461, 16.4604])
fz[9, :] = np.array(
    [0.1819, 0.4867, 1.0238, 1.6550, 2.3681, 3.1412, 3.9569, 5.1969, 6.3608, 7.5894, 8.9340, 10.3036, 11.8008, 13.1707,
     14.3994, 16.1402])
fz[10, :] = np.array(
    [0.1351, 0.5064, 0.9828, 1.6939, 2.3233, 3.1246, 4.0335, 5.1767, 6.2564, 7.4480, 8.9278, 10.3657, 11.6225, 12.8311,
     14.2612, 16.4650])
fz[11, :] = np.array(
    [0.1806, 0.4212, 0.9472, 1.7084, 2.3726, 3.1107, 4.1890, 5.2293, 6.3734, 7.6650, 8.9826, 10.3064, 11.6084, 13.1588,
     14.3931, 15.8940])
fz[12, :] = np.array(
    [0.1383, 0.4697, 0.9216, 1.6162, 2.3409, 3.1770, 4.2381, 5.1938, 6.5061, 7.8492, 9.1749, 10.5346, 11.9207, 13.1274,
     14.6602, 15.9889])

fz_r = np.zeros([len(servo_angle), 16])
fz_r[0, :] = np.array(
    [0.1904, 0.4763, 0.9933, 1.7194, 2.4517, 3.3293, 4.1953, 5.4315, 6.7285, 8.1034, 9.5070, 10.8310, 12.1489, 13.5539,
     14.9928, 16.2436])
fz_r[1, :] = np.array(
    [0.1263, 0.5142, 1.0404, 1.6685, 2.4718, 3.2658, 4.3704, 5.5122, 6.6144, 8.1109, 9.4115, 10.7517, 12.2067, 13.5206,
     14.6787, 16.1399])
fz_r[2, :] = np.array(
    [0.1401, 0.4742, 1.0089, 1.8847, 2.5112, 3.2547, 4.3831, 5.4146, 6.7644, 8.0596, 9.3511, 10.9017, 12.1758, 13.4776,
     14.7646, 16.2625])
fz_r[3, :] = np.array(
    [0.1634, 0.5335, 1.1043, 1.5340, 2.4922, 3.2938, 4.3304, 5.5321, 6.7701, 8.1338, 9.3919, 10.9181, 12.2461, 13.4673,
     14.8522, 16.2511])
fz_r[4, :] = np.array(
    [0.1391, 0.4922, 1.0923, 1.8799, 2.5876, 3.4190, 4.5196, 5.7548, 6.9478, 8.3725, 9.7701, 11.2292, 12.5955, 14.0452,
     15.5157, 16.8339])
fz_r[5, :] = np.array(
    [0.1641, 0.5359, 1.1503, 1.8179, 2.6081, 3.4285, 4.4768, 5.8662, 6.9551, 8.4486, 9.6794, 11.3162, 12.6672, 14.0506,
     15.5151, 17.0045])
fz_r[6, :] = np.array(
    [0.1458, 0.5096, 1.1393, 1.6901, 2.5680, 3.5531, 4.4621, 5.7831, 7.0414, 8.4606, 9.9400, 11.1986, 12.4997, 14.0011,
     15.6337, 16.8707])
fz_r[7, :] = np.array(
    [0.1958, 0.5362, 1.1302, 1.8964, 2.5866, 3.4409, 4.4385, 5.5781, 7.0661, 8.3642, 9.6693, 11.0314, 12.5256, 13.9410,
     15.1287, 16.5860])
fz_r[8, :] = np.array(
    [0.2042, 0.5927, 1.0475, 1.6828, 2.4551, 3.2655, 4.4265, 5.4694, 6.8486, 8.0551, 9.4342, 10.7657, 12.2879, 13.6193,
     14.9184, 16.3076])
fz_r[9, :] = np.array(
    [0.1446, 0.4621, 0.9274, 1.4385, 2.3643, 3.2212, 4.0311, 5.2580, 6.6137, 7.8684, 9.2791, 10.6831, 11.8723, 13.2455,
     14.4191, 15.9340])
fz_r[10, :] = np.array(
    [0.1872, 0.4884, 0.9211, 1.5863, 2.2948, 3.0596, 4.1190, 5.2522, 6.3480, 7.6831, 8.9163, 10.1417, 11.7157, 13.1003,
     14.3260, 15.8160])
fz_r[11, :] = np.array(
    [0.1969, 0.4673, 0.9227, 1.5121, 2.3735, 3.1901, 4.1569, 5.1573, 6.4640, 7.8788, 8.9584, 10.4246, 11.6508, 13.0575,
     14.2025, 15.7064])
fz_r[12, :] = np.array(
    [0.1913, 0.4757, 1.0287, 1.5233, 2.2942, 3.1966, 4.1866, 5.2930, 6.5271, 7.8746, 8.9816, 10.3634, 11.7313, 13.0596,
     14.3325, 15.6393])

# kt = np.array(
#     [0.1141, 0.1123, 0.1122, 0.1143, 0.1171, 0.1191, 0.1203, 0.1204, 0.1197, 0.1152, 0.1153, 0.1146, 0.1163])
# kq_d_kt = np.array(
#     [-0.0165, -0.0151, -0.0151, -0.0156, -0.0129, -0.0162, -0.0158, -0.0155, -0.0183, -0.0170, -0.0167, -0.0159,
#      -0.0156])
# resis_d_thrust = -(kt - 0.1203) / 0.1203 * 100

# draw two figures
# plot fz_krpm2_slope with respect to servo_angle
plt.style.use(["science", "grid"])
plt.rcParams.update({'font.size': 11})  # default is 10
label_size = 12

# plt.figure(figsize=(5.5, 4.5))
# # two subplots sharing the same x-axis
#
# ax1 = plt.subplot(211)
# plt.plot(servo_angle * 180 / np.pi, kt, 'o-', label='$k_t$ w/ mounting')
# plt.plot(0, 0.1495, 'ro', label='$k_t$ w/o mounting')
# # the interval for the x-axis is 15 degrees
# # plt.xticks(np.arange(-90, 91, 15))
# # plt.xlabel('Servo Angle ($^\circ$)', fontsize=label_size)
# # plt.xlim([-90, 90])
# plt.ylabel('$k_t$ (N/kRPM$^2$)', fontsize=label_size)
# plt.ylim([0.0, 0.16])
# plt.legend(framealpha=legend_alpha, loc="center left")
# # plt.setp(ax1.get_xticklabels(), visible=False)
#
# # draw kq_d_kt with respect to servo_angle at the same figure
# ax = plt.gca()
# ax2 = ax.twinx()
# ax2.plot(servo_angle * 180 / np.pi, -kq_d_kt, '*-', label='$k_q/k_t$ w/ mounting')
# ax2.plot(0, --0.0153, 'r*', label='$k_q/k_t$ w/o mounting')
# ax2.set_ylabel('$k_q/k_t$', fontsize=label_size)
# ax2.set_ylim([0.0, 0.16])
# ax2.legend(framealpha=legend_alpha, loc="center right")
#
# # plot resis_d_thrust with respect to servo_angle
# plt.subplot(212, sharex=ax1)

plt.figure(figsize=(5.5, 2.5))

# plt.plot(servo_angle * 180 / np.pi, resis_d_thrust, 'o-', label='measurement')

# plot fz with respect to servo_angle, the fz should be plotted as violin plot
for i in range(len(servo_angle)):
    plt.violinplot(-(fz[i, 3:] - fz[6, 3:]) / fz[6, 3:] * 100, positions=[servo_angle[i] * 180 / np.pi], widths=10,
                   showmeans=True, showmedians=False, showextrema=False)

# # plot fz_r with respect to servo_angle, the fz_r should be plotted as violin plot
# for i in range(len(servo_angle)):
#     plt.violinplot(-(fz_r[i, 3:] - fz_r[6, 3:]) / fz_r[6, 3:] * 100, positions=[servo_angle[i] * 180 / np.pi], widths=10,
#                    showmeans=True, showmedians=False, showextrema=False)

resis_d_thrust_n = np.mean(-(fz[:, 3:] - fz[6, 3:]) / fz[6, 3:] * 100, axis=1)
resis_d_thrust_r = np.mean(-(fz_r[:, 3:] - fz_r[6, 3:]) / fz_r[6, 3:] * 100, axis=1)

servo_virtual = np.arange(-90, 91, 1) * np.pi / 180

plt.plot(servo_angle * 180 / np.pi, resis_d_thrust_n, 'o-', label='normal prop.')
plt.plot(servo_angle * 180 / np.pi, resis_d_thrust_r, '*-', label='reverse prop.')

# reverse the order of resis_d_thrust_r
resis_d_thrust = (resis_d_thrust_n + resis_d_thrust_r[::-1]) / 2

# use 3rd order polynomial to fit the data
fit, residuals, _, _, _ = np.polyfit(servo_angle, resis_d_thrust, 3, full=True)
fit_fn = np.poly1d(fit)
# plt.plot(servo_virtual * 180 / np.pi, fit_fn(servo_virtual), ':', label='3rd-order poly.')
print("3ord\n", fit_fn)
print(" residuals: ", residuals)

# Calculate R^2
y = resis_d_thrust
y_mean = np.mean(y)
TSS = np.sum((y - y_mean) ** 2)
y_pred = fit_fn(servo_angle)
RSS = np.sum((y - y_pred) ** 2)
r_squared = 1 - (RSS / TSS)
print("R-squared: ", r_squared)

# use 4th order polynomial to fit the data
fit, residuals, _, _, _ = np.polyfit(servo_angle, resis_d_thrust, 4, full=True)
fit_fn = np.poly1d(fit)
plt.plot(servo_virtual * 180 / np.pi, fit_fn(servo_virtual), '--', label='4th-order poly.')
print("4ord\n", fit_fn)
print(" residuals: ", residuals)

# Calculate R^2
y = resis_d_thrust
y_mean = np.mean(y)
TSS = np.sum((y - y_mean) ** 2)
y_pred = fit_fn(servo_angle)
RSS = np.sum((y - y_pred) ** 2)
r_squared = 1 - (RSS / TSS)
print("R-squared: ", r_squared)

# plot 4th order polynomial fit flipped
plt.plot(servo_virtual[::-1] * 180 / np.pi, fit_fn(servo_virtual), '-.', label='4th-order poly. flipped')

# use 5th order polynomial to fit the data
fit, residuals, _, _, _ = np.polyfit(servo_angle, resis_d_thrust, 5, full=True)
fit_fn = np.poly1d(fit)
# plt.plot(servo_virtual * 180 / np.pi, fit_fn(servo_virtual), '--', label='5th-order poly.')
print("5ord\n", fit_fn)
print(" residuals: ", residuals)

# Calculate R^2
y = resis_d_thrust
y_mean = np.mean(y)
TSS = np.sum((y - y_mean) ** 2)
y_pred = fit_fn(servo_angle)
RSS = np.sum((y - y_pred) ** 2)
r_squared = 1 - (RSS / TSS)
print("R-squared: ", r_squared)

# the interval for the x-axis is 15 degrees
plt.xticks(np.arange(-90, 91, 15))
plt.xlim([-90, 90])
plt.xlabel('Servo Angle ($^\circ$)', fontsize=label_size)
plt.ylabel('$f_{i,D}/f_i$ (\%)', fontsize=label_size)

plt.legend(ncol=2, fontsize=10, framealpha=legend_alpha, loc="upper center")

plt.tight_layout()

plt.show()

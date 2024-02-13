clc;
clear;
close all;
input = importdata("inter_real_beetle_2_fly_wo_prop_2024-02-13-15-54-31.bag_gimbals_ctrl.csv");
output = importdata("inter_real_beetle_2_fly_wo_prop_2024-02-13-15-54-31.bag_joint_states.csv");

Options = n4sidOptions;
Options.Display = 'on';
Options.Focus = 'simulation';

% canonical means observability canonical x_d = (u - x) / t_servo,
% which is our need. Ts must be set to 0 for continuous form.
% I find that the Ts should be 0. If the Ts is set to 0.01, the equation
% becomes x_t+1 = 0.95 * x_t + 0.05 u, incorrect.
ss1 = n4sid(input(:, 2), output(:, 2), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0, Options);
ss11 = n4sid(input(:, 2), output(:, 2), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0.01, Options);
(-ss1.A + ss1.B)/2
disp("t_servo: "+ 2/(-ss1.A + ss1.B))

ss2 = n4sid(input(:, 3), output(:, 3), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0, Options);
ss22 = n4sid(input(:, 3), output(:, 3), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0.01, Options);
(-ss2.A + ss2.B)/2
disp("t_servo: "+ 2/(-ss2.A + ss2.B))

ss3 = n4sid(input(:, 4), output(:, 4), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0, Options);
ss33 = n4sid(input(:, 4), output(:, 4), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0.01, Options);
(-ss3.A + ss3.B)/2
disp("t_servo: "+ 2/(-ss3.A + ss3.B))

ss4 = n4sid(input(:, 5), output(:, 5), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0, Options);
ss44 = n4sid(input(:, 5), output(:, 5), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0.01, Options);
(-ss4.A + ss4.B)/2
disp("t_servo: "+ 2/(-ss4.A + ss4.B))

% figure();
% plot(input( :, 1), input( :, 2));
% hold on;
% plot(output( :, 1), output( :, 2));
% legend;

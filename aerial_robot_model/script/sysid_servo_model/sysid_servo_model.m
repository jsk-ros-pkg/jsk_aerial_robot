clc;
clear;
close all;
input = importdata("20240215-210011_inter_gimbals_ctrl.csv");
output = importdata("20240215-210011_inter_joint_states.csv");

Options = n4sidOptions;
Options.Display = 'on';
Options.Focus = 'simulation';

% canonical means observability canonical x_d = (u - x) / t_servo,
% which is our need. Ts must be set to 0 for continuous form.
% I find that the Ts should be 0. If the Ts is set to 0.01, the equation
% becomes x_t+1 = 0.95 * x_t + 0.05 u, incorrect.

% 20240215: find that ssest is better than n4sid
mydata = iddata(output(:, 2), input(:, 2), 0.01);
ss1 = ssest(mydata, 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0, Options);
% ss11 = n4sid(input(:, 2), output(:, 2), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0.01, Options);
(-ss1.A + ss1.B)/2
disp("t_servo: "+ 2/(-ss1.A + ss1.B))

mydata = iddata(output(:, 3), input(:, 3), 0.01);
ss2 = ssest(mydata, 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0, Options);
(-ss2.A + ss2.B)/2
disp("t_servo: "+ 2/(-ss2.A + ss2.B))

mydata = iddata(output(:, 4), input(:, 4), 0.01);
ss3 = ssest(mydata, 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0, Options);
(-ss3.A + ss3.B)/2
disp("t_servo: "+ 2/(-ss3.A + ss3.B))

ss4 = ssest(mydata, 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0, Options);
(-ss4.A + ss4.B)/2
disp("t_servo: "+ 2/(-ss4.A + ss4.B))

disp("t_servo avarage: "+ 2/(-ss1.A + ss1.B + -ss2.A + ss2.B + -ss3.A + ss3.B -ss4.A + ss4.B)*4)

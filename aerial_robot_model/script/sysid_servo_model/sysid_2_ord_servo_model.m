clc;
clear;
close all;
% input = importdata("20240215-210011_inter_gimbals_ctrl.csv");
% output = importdata("20240215-210011_inter_joint_states.csv");
input = importdata("20240216-132439_inter_gimbals_ctrl.csv");
output = importdata("20240216-132439_inter_joint_states.csv");

Options = tfestOptions;
Options.Display = 'on';
Options.EnforceStability = true;

% System Function:
% I * theta^ddot = kp*(u-theta) + kd*(-theta^dot) + miu*theta^dot
% convert to transfer function, no zeros and two poles.

best_dead_time = 0.0;
best_fit_accuracy = 0.0;
t_servo_when_best = 0.0;

% parallel axis theorem
Isxx = 0.00006797 + 0.15429 * (0.0^2 + 0.00007058^2);

mydata = iddata(output(:, 2), input(:, 2), 0.01);
tf1 = tfest(mydata, 2, 0, 'InputDelay', 0.0, Options);
fit_percent_1 = tf1.Report.Fit.FitPercent
kps1 = tf1.Numerator * Isxx
kds1 = tf1.Denominator(2) * Isxx
mus1 = kps1 - tf1.Denominator(3) * Isxx

mydata = iddata(output(:, 3), input(:, 3), 0.01);
tf2 = tfest(mydata, 2, 0, 'InputDelay', 0.0, Options);
fit_percent_2 = tf2.Report.Fit.FitPercent
kps2 = tf2.Numerator * Isxx
kds2 = tf2.Denominator(2) * Isxx
mus2 = kps2 - tf2.Denominator(3) * Isxx

mydata = iddata(output(:, 4), input(:, 4), 0.01);
tf3 = tfest(mydata, 2, 0, 'InputDelay', 0.0, Options);
fit_percent_3 = tf3.Report.Fit.FitPercent
kps3 = tf3.Numerator * Isxx
kds3 = tf3.Denominator(2) * Isxx
mus3 = kps3 - tf3.Denominator(3) * Isxx

mydata = iddata(output(:, 5), input(:, 5), 0.01);
tf4 = tfest(mydata, 2, 0, 'InputDelay', 0.0, Options);
fit_percent_4 = tf4.Report.Fit.FitPercent
kps4 = tf4.Numerator * Isxx
kds4 = tf4.Denominator(2) * Isxx
mus4 = kps4 - tf4.Denominator(3) * Isxx

% for dead_time = 0:0.001:0.02
%     % 20240215: find that ssest is better than n4sid
%     mydata = iddata(output(:, 2), input(:, 2), 0.01);
%     ss1 = ssest(mydata, 1, 'Form', 'canonical', 'InputDelay', dead_time, 'DisturbanceModel', 'none', 'Ts', 0, Options);
%     % ss11 = n4sid(input(:, 2), output(:, 2), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0.01, Options);
%     (-ss1.A + ss1.B)/2;
%     % disp("t_servo: "+ 2/(-ss1.A + ss1.B));
%     fit_percent_1 = ss1.Report.Fit.FitPercent;
% 
%     mydata = iddata(output(:, 3), input(:, 3), 0.01);
%     ss2 = ssest(mydata, 1, 'Form', 'canonical', 'InputDelay', dead_time, 'DisturbanceModel', 'none', 'Ts', 0, Options);
%     (-ss2.A + ss2.B)/2;
%     % disp("t_servo: "+ 2/(-ss2.A + ss2.B));
%     fit_percent_2 = ss2.Report.Fit.FitPercent;
% 
%     mydata = iddata(output(:, 4), input(:, 4), 0.01);
%     ss3 = ssest(mydata, 1, 'Form', 'canonical', 'InputDelay', dead_time, 'DisturbanceModel', 'none', 'Ts', 0, Options);
%     (-ss3.A + ss3.B)/2;
%     % disp("t_servo: "+ 2/(-ss3.A + ss3.B));
%     fit_percent_3 = ss3.Report.Fit.FitPercent;
% 
%     ss4 = ssest(mydata, 1, 'Form', 'canonical', 'InputDelay', dead_time, 'DisturbanceModel', 'none', 'Ts', 0, Options);
%     (-ss4.A + ss4.B)/2;
%     % disp("t_servo: "+ 2/(-ss4.A + ss4.B));
%     fit_percent_4 = ss4.Report.Fit.FitPercent;
% 
%     disp("dead_time: " + dead_time);
%     t_servo_average = 2/(-ss1.A + ss1.B + -ss2.A + ss2.B + -ss3.A + ss3.B -ss4.A + ss4.B)*4;
%     disp("t_servo average: " + t_servo_average);
%     fit_average = (fit_percent_1 + fit_percent_2 + fit_percent_3 + fit_percent_4)/4;
%     disp("fit percentage average: " + fit_average);
%     disp("-------------------------------")
% 
%     if fit_average > best_fit_accuracy
%         best_fit_accuracy = fit_average;
%         best_dead_time = dead_time;
%         t_servo_when_best = t_servo_average;
%     end
% end
% disp("=============================================");
% disp("best_fit_accuracy: " + best_fit_accuracy);
% disp("best_dead_time: " + best_dead_time);
% disp("t_servo_when_best: " + t_servo_when_best);
% disp("=============================================");

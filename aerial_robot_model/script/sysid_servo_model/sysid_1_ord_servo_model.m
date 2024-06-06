clc;
clear;
close all;
% input = importdata("20240215-210011_inter_gimbals_ctrl.csv");
% output = importdata("20240215-210011_inter_joint_states.csv");
input = importdata("20240216-132439_inter_gimbals_ctrl.csv");
output = importdata("20240216-132439_inter_joint_states.csv");

Options = n4sidOptions;
Options.Display = 'on';
Options.Focus = 'simulation';

% canonical means observability canonical x_d = (u - x) / t_servo,
% which is our need. Ts must be set to 0 for continuous form.
% I find that the Ts should be 0. If the Ts is set to 0.01, the equation
% becomes x_t+1 = 0.95 * x_t + 0.05 u, incorrect.

best_dead_time = 0.0;
best_fit_accuracy = 0.0;
t_servo_when_best = 0.0;

for dead_time = 0:0.001:0.02
    % 20240215: find that ssest is better than n4sid
    mydata = iddata(output(:, 2), input(:, 2), 0.01);
    ss1 = ssest(mydata, 1, 'Form', 'canonical', 'InputDelay', dead_time, 'DisturbanceModel', 'none', 'Ts', 0, Options);
    % ss11 = n4sid(input(:, 2), output(:, 2), 1, 'Form', 'canonical', 'DisturbanceModel', 'none', 'Ts', 0.01, Options);
    (-ss1.A + ss1.B)/2;
    % disp("t_servo: "+ 2/(-ss1.A + ss1.B));
    fit_percent_1 = ss1.Report.Fit.FitPercent;
    
    mydata = iddata(output(:, 3), input(:, 3), 0.01);
    ss2 = ssest(mydata, 1, 'Form', 'canonical', 'InputDelay', dead_time, 'DisturbanceModel', 'none', 'Ts', 0, Options);
    (-ss2.A + ss2.B)/2;
    % disp("t_servo: "+ 2/(-ss2.A + ss2.B));
    fit_percent_2 = ss2.Report.Fit.FitPercent;
    
    mydata = iddata(output(:, 4), input(:, 4), 0.01);
    ss3 = ssest(mydata, 1, 'Form', 'canonical', 'InputDelay', dead_time, 'DisturbanceModel', 'none', 'Ts', 0, Options);
    (-ss3.A + ss3.B)/2;
    % disp("t_servo: "+ 2/(-ss3.A + ss3.B));
    fit_percent_3 = ss3.Report.Fit.FitPercent;
    
    ss4 = ssest(mydata, 1, 'Form', 'canonical', 'InputDelay', dead_time, 'DisturbanceModel', 'none', 'Ts', 0, Options);
    (-ss4.A + ss4.B)/2;
    % disp("t_servo: "+ 2/(-ss4.A + ss4.B));
    fit_percent_4 = ss4.Report.Fit.FitPercent;
    
    disp("dead_time: " + dead_time);
    t_servo_average = 2/(-ss1.A + ss1.B + -ss2.A + ss2.B + -ss3.A + ss3.B -ss4.A + ss4.B)*4;
    disp("t_servo average: " + t_servo_average);
    fit_average = (fit_percent_1 + fit_percent_2 + fit_percent_3 + fit_percent_4)/4;
    disp("fit percentage average: " + fit_average);
    disp("-------------------------------")

    if fit_average > best_fit_accuracy
        best_fit_accuracy = fit_average;
        best_dead_time = dead_time;
        t_servo_when_best = t_servo_average;
    end
end
disp("=============================================");
disp("best_fit_accuracy: " + best_fit_accuracy);
disp("best_dead_time: " + best_dead_time);
disp("t_servo_when_best: " + t_servo_when_best);
disp("=============================================");

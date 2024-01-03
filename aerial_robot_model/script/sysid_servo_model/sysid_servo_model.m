clc;
clear;
close all;
input = importdata("inter_2023-12-30-17-56-53_beetle_servo_sysid.bag_gimbals_ctrl.csv");
output = importdata("inter_2023-12-30-17-56-53_beetle_servo_sysid.bag_joint_states.csv");

Options = n4sidOptions;
Options.Display = 'on';
Options.Focus = 'simulation';

% canonical means observability canonical x_d = (u - x) / t_servo,
                                          which is our need.ss1 = n4sid(input(
                                                                            :, 2),
                                                                        output(
                                                                            :, 2),
                                                                        1, 'Form', 'canonical', 'Ts', 0, Options) ss2 =
                                              n4sid(input(
                                                        :, 3),
                                                    output(
                                                        :, 3),
                                                    1, 'Form', 'canonical', 'Ts', 0, Options) ss3 =
                                                  n4sid(input(
                                                            :, 4),
                                                        output(
                                                            :, 4),
                                                        1, 'Form', 'canonical', 'Ts', 0, Options) ss4 =
                                                      n4sid(input(
                                                                :, 5),
                                                            output(
                                                                :, 5),
                                                            1, 'Form', 'canonical', 'Ts', 0, Options)

                                                      % figure();
% plot(input( :, 1), input( :, 2));
% hold on;
% plot(output( :, 1), output( :, 2));
% legend;

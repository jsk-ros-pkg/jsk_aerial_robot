#include <ros/ros.h>
#include <spinal/PwmTest.h>
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

#include <sys/select.h>
#include <unistd.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

Eigen::Vector4d computeAlphaFixedTotal(const Eigen::Vector2d& dir, double total_thrust_c)
{
    Eigen::Vector4d alpha = Eigen::Vector4d::Zero();
    double n = dir.norm();
    if (n < 1e-6) {
        return alpha;
    }


    Eigen::Matrix<double, 2, 4> motor_dirs_base;
    motor_dirs_base <<  1, -1, -1,  1,
                       -1, -1,  1,  1;
    Eigen::Matrix<double, 2, 4> motor_dirs = motor_dirs_base;

    Eigen::Vector2d d = dir / n;

    int best_idx = -1;
    double best_cos = -1.0;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector2d col = motor_dirs.col(i);
        double col_norm = col.norm();
        if (col_norm < 1e-6) continue;

        Eigen::Vector2d col_unit = col / col_norm;
        double cos_angle = col_unit.dot(d);

        if (cos_angle > best_cos) {
            best_cos = cos_angle;
            best_idx = i;
        }
    }

    const double single_rotor_threshold = 0.999; // 角度 ~ 2.6度以内

    if (best_idx >= 0 && best_cos > single_rotor_threshold) {
        alpha[best_idx] = total_thrust_c;
        return alpha;
    }

    Eigen::Matrix<double,3,4> A;
    A.block<2,4>(0,0) = motor_dirs;
    A.block<1,4>(2,0) = Eigen::RowVector4d::Ones();

    Eigen::Matrix3d AAT = A * A.transpose();
    Eigen::Vector3d b;
    b << (total_thrust_c * d.x()), (total_thrust_c * d.y()), total_thrust_c;

    alpha = A.transpose() * AAT.ldlt().solve(b);

    return alpha;
}


double calThrustPower(double alpha, double thrust_strength)
{
    double thrust = thrust_strength * std::abs(alpha);
    double pwm = -0.000679 * thrust * thrust + 0.044878 * thrust + 0.5;
    if (pwm > 0.8) pwm = 0.8;
    if (pwm < 0.5) pwm = 0.5;
    return pwm;
}

bool stdinHasData()
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    return (ret > 0) && FD_ISSET(STDIN_FILENO, &fds);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_haptics_feeling_node");
    ros::NodeHandle nh("~");

    ros::Publisher pwm_pub = nh.advertise<spinal::PwmTest>("/pwm_test", 1);
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float32>("/haptics_thrust", 1);
    ros::Publisher dir_pub = nh.advertise<std_msgs::Int8>("/haptics_direction", 1);
    ros::Publisher alpha_pub = nh.advertise<std_msgs::Float32MultiArray>("/alpha", 1);

    double min_total_thrust, max_total_thrust;
    double inter_trial_interval;
    int rate_hz;
    std::string log_file;

    bool stim_on = true;
    ros::Time toggle_time = ros::Time::now();
    bool pause_toggle, shutdown;
    double total_thrust_ = 1.0;

    nh.param("min_total_thrust", min_total_thrust, 0.5);
    nh.param("max_total_thrust", max_total_thrust, 1.0);
    nh.param("inter_trial_interval", inter_trial_interval, 2.0);
    nh.param("rate_hz", rate_hz, 50);
    nh.param("log_file", log_file, std::string("haptics_threshold_log.csv"));

    std::ofstream ofs(log_file.c_str(), std::ios::app);
    if (!ofs) {
        ROS_WARN("Failed to open log file: %s", log_file.c_str());
    } else {
        ofs << "#dir_idx,angle_deg,total_thrust,response(0/1)\n";
    }

    std::mt19937 rng(static_cast<unsigned int>(
        ros::Time::now().toNSec() & 0xffffffff));

    std::vector<int> dir_indices = {0,1,2,3,4,5,6,7};

    std::vector<int> thrust_set_even = {2,4,6,8};
    std::vector<int> thrust_set_odd  = {1,3,5,7};

    int current_dir_idx_ = 0;
    int repeat_in_dir_ = 0;
    bool use_even_set = true;

    std::vector<int> thrust_seq(4);

    enum State { INTER_TRIAL, STIMULUS };
    State state = INTER_TRIAL;
    ros::Time state_start = ros::Time::now();

    Eigen::Vector2d current_dir(1.0, 0.0);
    double current_total_thrust = 0.0;

    ros::Rate rate(rate_hz);

    while (ros::ok()) {
        ros::spinOnce();
        ros::Time now = ros::Time::now();

        if (state == INTER_TRIAL) {
            spinal::PwmTest neutral;
            neutral.motor_index = {0,1,2,3};
            neutral.pwms = {0.5,0.5,0.5,0.5};
            pwm_pub.publish(neutral);

            if ((now - state_start).toSec() >= inter_trial_interval) {
                int dir_idx = dir_indices[current_dir_idx_];
                double angle = dir_idx * M_PI / 4.0;  // 0,45,...315 [rad]
                std_msgs::Int8 dir_msg;
                dir_msg.data = angle;
                dir_pub.publish(dir_msg);
                current_dir = Eigen::Vector2d(std::cos(angle), std::sin(angle));

                if (repeat_in_dir_ == 0) {
                    const std::vector<int>& base = (use_even_set ? thrust_set_even : thrust_set_odd);
                    thrust_seq = base;
                    std::shuffle(thrust_seq.begin(), thrust_seq.end(), rng);

                    ROS_INFO("Direction %d (%.1f deg): thrust sequence = %d, %d, %d, %d", current_dir_idx_, angle * 180.0 / M_PI, thrust_seq[0], thrust_seq[1], thrust_seq[2], thrust_seq[3]);
                }

                int thrust_raw = thrust_seq[repeat_in_dir_];
                current_total_thrust = thrust_raw;

                std_msgs::Float32 thrust_msg;
                thrust_msg.data = static_cast<float>(current_total_thrust);
                thrust_pub.publish(thrust_msg);

                state = STIMULUS;
                state_start = now;
            }
        }
        else if (state == STIMULUS) {
          if ((now - toggle_time).toSec() >= 0.5){
            stim_on = ! stim_on;
            toggle_time = now;
          }
          if(stim_on){
            Eigen::Vector4d alpha = computeAlphaFixedTotal(current_dir, total_thrust_);

            spinal::PwmTest msg;
            msg.motor_index = {0, 1, 2, 3};
            msg.pwms.resize(4);
            std_msgs::Float32MultiArray alpha_msg;
            alpha_msg.data.resize(4);
            for (int i = 0; i < 4; ++i) {
                msg.pwms[i] = static_cast<float>(calThrustPower(alpha[i], current_total_thrust));
                ROS_INFO("Motor %d: alpha = %.3f, pwm = %.3f", i, alpha[i], msg.pwms[i]);
                alpha_msg.data[i] = alpha[i];
            }
            pwm_pub.publish(msg);
            alpha_pub.publish(alpha_msg);
          }else{
            spinal::PwmTest neutral;
            neutral.motor_index = {0, 1, 2, 3};
            neutral.pwms = {0.5, 0.5, 0.5, 0.5};
            pwm_pub.publish(neutral);
          }

            if (stdinHasData()) {
                std::string line;
                if (!std::getline(std::cin, line)) {
                    ROS_WARN("Failed to read stdin line.");
                } else {
                    int response = -1;
                    if (!line.empty()) {
                        char c = line[0];
                        if (c == '0' || c == '1') {
                            response = (c == '1') ? 1 : 0;
                        } else if (c == 's'){
                          pause_toggle = true;
                        } else if (c == 'h'){
                          shutdown = true;
                        }
                    }

                    if (response == 0 || response == 1) {
                        double angle_deg = current_dir_idx_ * 45.0;
                        ROS_INFO("Response: %d (angle=%.1f deg, total_thrust=%.3f)",
                                 response, angle_deg, current_total_thrust);

                        if (ofs) {
                            ofs << current_dir_idx_ << ","
                                << angle_deg << ","
                                << current_total_thrust << ","
                                << response << "\n";
                            ofs.flush();
                        }

                        spinal::PwmTest neutral;
                        neutral.motor_index = {0, 1, 2, 3};
                        neutral.pwms = {0.5, 0.5, 0.5, 0.5};
                        pwm_pub.publish(neutral);

                        stim_on = true;
                        toggle_time = now;
                        repeat_in_dir_++;
                        if (repeat_in_dir_ >= 4) {
                          repeat_in_dir_ = 0;
                          current_dir_idx_++;
                          use_even_set = !use_even_set;
                        }
                        if (current_dir_idx_ >= 8) {
                          ROS_INFO("All directions completed.");
                          break;
                        }
                        state = INTER_TRIAL;
                        state_start = now;

                    }
                    if(pause_toggle){
                      spinal::PwmTest neutral;
                      neutral.motor_index = {0, 1, 2, 3};
                      neutral.pwms = {0.5, 0.5, 0.5, 0.5};
                      pwm_pub.publish(neutral);
                      stim_on = false;
                      toggle_time = now;
                    }
                    if (shutdown) {
                      ROS_ERROR("== Experiment terminated by user (h pressed) ==");
                      spinal::PwmTest neutral;
                      neutral.motor_index = {0, 1, 2, 3};
                      neutral.pwms = {0.5, 0.5, 0.5, 0.5};
                      pwm_pub.publish(neutral);
                      ros::shutdown();
                      return 0;
                    }
                }
            }
        }
        rate.sleep();
    }
    return 0;
}

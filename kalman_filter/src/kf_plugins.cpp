#include <pluginlib/class_list_macros.h>
#include <kalman_filter/kf_base.h>
#include <kalman_filter/kf_pos_vel_acc_plugin.h>

typedef kf_base::KalmanFilterP<2,1,1> kalmanFilter211;


PLUGINLIB_EXPORT_CLASS(kf_pos_vel_acc_plugins::KalmanFilterPosVelAccP, kalmanFilter211 );







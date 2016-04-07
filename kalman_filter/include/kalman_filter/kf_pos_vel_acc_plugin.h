#ifndef KALMAN_FILTER_POS_VEL_ACC_H_
#define KALMAN_FILTER_POS_VEL_ACC_H_
#include <kalman_filter/kf_base.h>
#include <cmath>

namespace kf_pos_vel_acc_plugins 
{
  class KalmanFilterPosVelAccP : public kf_base::KalmanFilterP<2, 1, 1>
  {
  public:
    KalmanFilterPosVelAccP() {}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nh_private)
    {
      nh_ = nh;
      nhp_ = nh_private;
      kfModelInit();
      test_ = 0;
      std::cout << "estimate_state" << std::endl <<  estimate_state_ << std::endl;

    }

  private:
    double test_;
  };
};
#endif


#ifndef KALMAN_FILTER_POS_VEL_ACC_PLUGIN_H
#define KALMAN_FILTER_POS_VEL_ACC_PLUGIN_H
#include <kalman_filter/kf_base_plugin.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <kalman_filter/KalmanFilterPosVelAccConf.h>
#include <kalman_filter/KalmanFilterPosVelAccConfig.h>
#include <kalman_filter/KalmanFilterPosVelAccBiasConfig.h>

#define CORRECT_POS 0
#define CORRECT_VEL 1
#define CORRECT_POS_VEL 2


namespace kf_pos_vel_acc_plugin
{
  class KalmanFilterPosVelAcc : public kf_base_plugin::KalmanFilter
  {
  public:
    KalmanFilterPosVelAcc() {}
    ~KalmanFilterPosVelAcc() {}

    void initialize(ros::NodeHandle nh, string suffix, int id);

    void setCorrectMode(uint8_t correct_mode);
    void updateModelFromDt(double dt);

  private:
    //dynamic reconfigure
    dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccConfig>* server_;
    dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccConfig>::CallbackType dynamic_reconf_func_;
    void cfgCallback(kalman_filter::KalmanFilterPosVelAccConfig &config, uint32_t level);
  };


  class KalmanFilterPosVelAccBias : public kf_base_plugin::KalmanFilter
  {
  public:
    KalmanFilterPosVelAccBias() {}

    ~KalmanFilterPosVelAccBias() {}

    void initialize(ros::NodeHandle nh, string suffix, int id);

    void setCorrectMode(uint8_t correct_mode);
    void updateModelFromDt(double dt);

  private:
    //dynamic reconfigure
    dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccBiasConfig>* server_;
    dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccBiasConfig>::CallbackType dynamic_reconf_func_;
    void cfgCallback(kalman_filter::KalmanFilterPosVelAccBiasConfig &config, uint32_t level);

  };

};



#endif


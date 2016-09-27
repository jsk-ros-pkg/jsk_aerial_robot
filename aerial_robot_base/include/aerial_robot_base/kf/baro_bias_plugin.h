#ifndef KALMAN_FILTER_Baro_Bias_PLUGIN_H
#define KALMAN_FILTER_Baro_Bias_PLUGIN_H
#include <kalman_filter/kf_base_plugin.h>
//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_base/KalmanFilterBaroBiasConfig.h>

namespace kf_baro_bias_plugin
{
  class KalmanFilterBaroBias : public kf_base_plugin::KalmanFilter
  {
  public:
    KalmanFilterBaroBias() {}
    ~KalmanFilterBaroBias() {}

    void initialize(ros::NodeHandle nh, string suffix, int id);
    void updateModelFromDt(double dt);

    void setCorrectMode(uint8_t correct_mode){};

  private:
    //dynamic reconfigure
    dynamic_reconfigure::Server<aerial_robot_base::KalmanFilterBaroBiasConfig>* server_;
    dynamic_reconfigure::Server<aerial_robot_base::KalmanFilterBaroBiasConfig>::CallbackType dynamic_reconf_func_;
    void cfgCallback(aerial_robot_base::KalmanFilterBaroBiasConfig &config, uint32_t level);
  };
};

#endif


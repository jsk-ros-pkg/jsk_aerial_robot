#ifndef KF_POS_VEL_ACC_H
#define KF_POS_VEL_ACC_H

#include <kalman_filter/kalman_filter.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <kalman_filter/KalmanFilterPosVelAccConf.h>
#include <kalman_filter/KalmanFilterPosVelAccConfig.h>
#include <kalman_filter/KalmanFilterPosVelAccBiasConfig.h>

//using namespace Eigen;
//using namespace std;

#define CORRECT_POS 0
#define CORRECT_VEL 1
#define CORRECT_POS_VEL 2

class KalmanFilterPosVelAcc:  public KalmanFilter<2, 1, 1>
{
 public:
  KalmanFilterPosVelAcc(ros::NodeHandle nh, ros::NodeHandle nh_private, string filter_id = string(""), uint8_t correct_mode = CORRECT_POS);
  ~KalmanFilterPosVelAcc(){}

 void setCorrectMode(uint8_t correct_mode);
 void updateModelFromDt(double dt);

  //dynamic reconfigure
  void cfgCallback(kalman_filter::KalmanFilterPosVelAccConfig &config, uint32_t level);


 private:
  ros::NodeHandle nhp_axis_;

  uint8_t correct_mode_;

  //dynamic reconfigure
  dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccConfig>* server_;
  dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccConfig>::CallbackType dynamic_reconf_func_;

  void rosParamInit();

  string id_;

};

class KalmanFilterPosVelAccBias : public KalmanFilter<3, 2, 1>
{
public:
KalmanFilterPosVelAccBias(ros::NodeHandle nh, ros::NodeHandle nh_private, string filter_id = string(""), uint8_t correct_mode = CORRECT_POS);

~KalmanFilterPosVelAccBias(){}

  //void setInitImuBias(double initBias);
 void setCorrectMode(uint8_t correct_mode);

void cfgCallback(kalman_filter::KalmanFilterPosVelAccBiasConfig &config, uint32_t level);
void setInitImuBias(double init_bias);
 void updateModelFromDt(double dt);


 private:
  ros::NodeHandle nhp_axis_;
  uint8_t correct_mode_;

  string id_;

  //dynamic reconfigure
  dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccBiasConfig>* server_;
  dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccBiasConfig>::CallbackType dynamic_reconf_func_;

  void rosParamInit();
};

#endif

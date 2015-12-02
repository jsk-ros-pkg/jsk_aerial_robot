#ifndef KF_POS_VEL_ACC_H
#define KF_POS_VEL_ACC_H

#include <kalman_filter/kalman_filter.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <kalman_filter/DynamicReconfigureLevels.h>
#include <kalman_filter/KalmanFilterPosVelAccConfig.h>

//using namespace Eigen;
//using namespace std;

#define uint_8 CORRECT_POS = 0;
#define uint_8 CORRECT_VEL = 1;
#define uint_8 CORRECT_POS_VEL = 2;

class KalmanFilterPosVelAcc:  KalmanFilter<2, 1, 1>
{
 public:
  KalmanFilterPosVelAcc(ros::NodeHandle nh, ros::NodeHandle nh_private, string filter_id = string(""), uint8_t correct_mode = CORRECT_POS);
  ~KalmanFilterPosVelAcc(){}

  inline void setCorrectMode(uint8_t correct_mode){correct_mode_ = correct_mode;}

  //dynamic reconfigure
  void cfgCallback(aerial_robot_base::StateKalmanFilterConfig &config, uint32_t level);


 private:
  ros::NodeHandle nhp_axis_;

  uint8_t correct_mode_;

  //dynamic reconfigure
  dynamic_reconfigure::Server<aerial_robot_base::StateKalmanFilterConfig>* server_;
  dynamic_reconfigure::Server<aerial_robot_base::StateKalmanFilterConfig>::CallbackType dynamic_reconf_func_;

  void rosParamInit();
};

class KalmanFilterPosVelAccBias : KalmanFilter<3, 2, 1>
{
 public:
  KalmanFilterPosVelAccBias(ros::NodeHandle nh, ros::NodeHandle nh_private, string filter_id = string(""), , uint8_t correct_mode = CORRECT_POS);
  ~KalmanFilterPosVelAccBias();

  //void setInitImuBias(double initBias);
  inline void setCorrectMode(uint8_t correct_mode){correct_mode_ = correct_mode;}

  void cfgCallback(aerial_robot_base::StateKalmanFilterConfig &config, uint32_t level);

 private:
  ros::NodeHandle nhp_axis_;

  //dynamic reconfigure
  dynamic_reconfigure::Server<aerial_robot_base::StateKalmanFilterConfig>* server_;
  dynamic_reconfigure::Server<aerial_robot_base::StateKalmanFilterConfig>::CallbackType dynamic_reconf_func_;

  void rosParamInit();
};

#endif

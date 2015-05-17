#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

//* ros
#include <ros/ros.h>
#include <jsk_quadcopter/ImuQu.h>

//* for kalman filter
#include <Eigen/Core>
//#include <Eigen/LU>
#include <Eigen/Dense>

#include <iostream>
#include <queue>

#include <jsk_quadcopter/digital_filter.h>
//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <jsk_quadcopter_common/DynamicReconfigureLevels.h>
#include <jsk_quadcopter/StateKalmanFilterConfig.h>


//* for mutex
#include <boost/thread/mutex.hpp>




class KalmanFilterImuLaser : public Filter 
{
 public:
  KalmanFilterImuLaser(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string filterID, bool DynamicReconf = false);
  KalmanFilterImuLaser(double sigmaPredict, double sigmaMeasure, double dtime, std::string filterID); //deprecated
  ~KalmanFilterImuLaser();

  bool prediction(double input, ros::Time timeStamp);
  bool correction(double measurement, ros::Time timeStamp);
  //only velocity
  bool correctionOnlyVelocity(double measurement, ros::Time timeStamp);
  //for time synchronized state
  void imuQuCorrection(ros::Time checkTimeStamp, double measurement, int type = 0);
  void imuQuOnlyPrediction(ros::Time checkTimeStamp); //for bad measurement step
  bool imuQuPrediction(ros::Time checkTimeStamp);
  void imuQuPush(jsk_quadcopter::ImuQuPtr imuQuMsgPtr);


  double getEstimatePos();
  double getEstimateVel();
  double getCorrectPos();
  double getCorrectVel();
  double getPredictPos();
  double getPredictVel();
  void getEstimateCovariance(float* covarianceMatrix);
  void test();
  bool getFilteringStartFlag();
  void setInputStartFlag();
  void setMeasureStartFlag(bool flag);
  void setInitImuBias(double initBias);
  void setInitState(double init_pos, double init_vel);

  //dynamic reconfigure
  void cfgCallback(jsk_quadcopter::StateKalmanFilterConfig &config, uint32_t level);
  
 private:
  ros::NodeHandle kalmanFilterNodeHandle_;
  ros::NodeHandle kalmanFilterNodeHandlePrivate_;
  ros::NodeHandle kalmanFilterNodeHandlePrivateWithAxis_;


  double dt;
  double sigmaAcc;
  double sigmaOpticalFlow;
  double sigmaLaser;
  Eigen::Vector2d estimateState; //0:Position, 1:Velocity
  Eigen::Vector2d correctState;  //0:Position, 1:Velocity
  Eigen::Vector2d predictState; //0:Position, 1:Velocity
  Eigen::Matrix2d predictionNoiseCovariance, estimateCovariance;
  Eigen::Matrix<double, 1, 1> measurementNoiseCovariance;
  Eigen::Matrix<double, 1, 1> measurementOnlyVelocityNoiseCovariance;
  Eigen::Matrix<double, 1, 1> inovationCovariance;
  Eigen::Matrix<double, 2, 1> kalmanGain;
  
  Eigen::Matrix2d stateTransitionModel;
  Eigen::Matrix<double, 1, 2> observationModel;
  Eigen::Matrix<double, 1, 2> observationOnlyVelocityModel;
  Eigen::Matrix<double, 2, 1> controlInputModel;

  //time synchronized state
  std::queue<jsk_quadcopter::ImuQuPtr> imuQu;

  ros::Time kalmanFilterStamp;
  std::string id;

  //filtering start flag
  bool inputStartFlag;
  bool measureStartFlag;

  //bias effect
  double accBias;

  //dynamic reconfigure
  dynamic_reconfigure::Server<jsk_quadcopter::StateKalmanFilterConfig>* server;
  dynamic_reconfigure::Server<jsk_quadcopter::StateKalmanFilterConfig>::CallbackType dynamicReconfFunc;

  //for mutex
  boost::mutex kfMutex;
  boost::mutex queueMutex;
  //boost::mutex kfMutex;

  double inputSigma_;
  double measureSigma_;
  double imuHz_;
  void rosParamInit();
};

class KalmanFilterImuLaserBias : public Filter 
{
 public:
  KalmanFilterImuLaserBias(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string filterID, bool DynamicReconf = false);

  //deprecated
  KalmanFilterImuLaserBias(double sigmaPredictAcc, double sigmaPredictBias,
                           double sigmaMeasure,  double dtime,
                           std::string filterID);

  /*
  KalmanFilterImuLaserBias(double sigmaPredictAcc, double sigmaPredictBias,
                           double sigmaMeasure,
                           double dtime,
                           std::string filterID);
  */

  ~KalmanFilterImuLaserBias();

  bool prediction(double input, ros::Time stamp);
  double correction(double measurement, ros::Time stamp);
  //only velocity
  double correctionOnlyVelocity(double measurement, ros::Time stamp);

  //for time synchronized state
  void imuQuCorrection(ros::Time checkTimeStamp, double measurement, int type = 0);
  void imuQuOnlyPrediction(ros::Time checkTimeStamp); //for bad measurement step
  bool imuQuPrediction(ros::Time checkTimeStamp);
  void imuQuPush(jsk_quadcopter::ImuQuPtr imuQuMsgPtr);



  double getEstimatePos();
  double getEstimateVel();
  double getEstimateBias();
  double getCorrectPos();
  double getCorrectVel();
  double getPredictPos();
  double getPredictVel();
  void getEstimateCovariance(float* covarianceMatrix);
  void test();
  bool getFilteringStartFlag();
  void setInputStartFlag();
  void setMeasureStartFlag(bool flag);
  void setInitImuBias(double initBias);
  void setInitState(double init_pos, double init_vel);
  void reset();

  //dynamic reconfigure
  void cfgCallback(jsk_quadcopter::StateKalmanFilterConfig &config, uint32_t level);

 private:
  ros::NodeHandle kalmanFilterNodeHandle_;
  ros::NodeHandle kalmanFilterNodeHandlePrivate_;
  ros::NodeHandle kalmanFilterNodeHandlePrivateWithAxis_;
  

  double dt;
  double sigmaAcc;
  double sigmaOpticalFlow;
  double sigmaAccBias;
  double sigmaLaser;
  Eigen::Vector3d estimateState; //0:Position, 1:Velocity, 2:Bias
  Eigen::Vector3d correctState;  //0:Position, 1:Velocity, 2:Bias
  Eigen::Vector3d predictState; //0:Position, 1:Velocity, 2:Bias
  Eigen::Matrix3d predictionNoiseCovariance, estimateCovariance;
  Eigen::Matrix<double, 1, 1> measurementNoiseCovariance;
  Eigen::Matrix<double, 1, 1> measurementOnlyVelocityNoiseCovariance;
  Eigen::Matrix<double, 1, 1> inovationCovariance;
  Eigen::Matrix<double, 3, 1> kalmanGain;
  
  Eigen::Matrix3d stateTransitionModel;
  Eigen::Matrix<double, 1, 3> observationModel;
  Eigen::Matrix<double, 1, 3> observationOnlyVelocityModel;
  Eigen::Matrix<double, 3, 1>  controlInputModel;

  //time synchronized state
  std::queue <jsk_quadcopter::ImuQuPtr> imuQu;
  Eigen::Vector2d estimateStateTimeSync; //0:Position, 1:Velocity


  bool inputStartFlag;
  bool measureStartFlag;


  ros::Time kalmanFilterStamp;
  std::string id;
  //dynamic reconfigure
  dynamic_reconfigure::Server<jsk_quadcopter::StateKalmanFilterConfig>* server;
  dynamic_reconfigure::Server<jsk_quadcopter::StateKalmanFilterConfig>::CallbackType dynamicReconfFunc;

  boost::mutex kfMutex;
  boost::mutex queueMutex;

  double inputSigma_;
  double biasSigma_;
  double measureSigma_;
  double imuHz_;
  void rosParamInit();
};

#endif

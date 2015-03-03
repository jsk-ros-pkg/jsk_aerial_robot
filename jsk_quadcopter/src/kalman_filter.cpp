#include "jsk_quadcopter/kalman_filter.h"


KalmanFilterImuLaser::KalmanFilterImuLaser(double sigmaPredict, double sigmaMeasure, 
                                           double dtime, std::string filterID): id(filterID)
{
#if 0
  ROS_WARN("kalman filter: sigmaPredictAcc is %f, sigmaMeasure is %f",
           sigmaPredict,
           sigmaMeasure);
#endif

  //init 
  sigmaAcc = sigmaPredict; sigmaLaser = sigmaMeasure;
  dt = dtime;
  inputStartFlag = false;
  measureStartFlag = false;


  //dynamic reconfigure
  std::string strBegin("~WithoutBias");
  strBegin.append(filterID);
  server = new dynamic_reconfigure::Server<jsk_quadcopter::StateKalmanFilterConfig>(ros::NodeHandle(strBegin));
  dynamicReconfFunc = boost::bind(&KalmanFilterImuLaser::cfgCallback, this, _1, _2);
  server->setCallback(dynamicReconfFunc);


  estimateState(0) = 0; //position initial
  estimateState(1) = 0; //velocity initial

  correctState(0) = 0; //position initial
  correctState(1) = 0; //velocity initial

  predictState(0) = 0; //position initial
  predictState(1) = 0; //velocity initial

  stateTransitionModel << 1, dt, 0, 1;
  controlInputModel << (dt * dt)/2, dt;
  observationModel << 1, 0;
  observationOnlyVelocityModel << 0, 1;

  predictionNoiseCovariance = (sigmaAcc * sigmaAcc) * controlInputModel * controlInputModel.transpose();
  estimateCovariance << 0, 0, 0, 0;
  inovationCovariance << 0;
  measurementNoiseCovariance(0) = sigmaLaser * sigmaLaser;
  //bad !!
  measurementOnlyVelocityNoiseCovariance(0) = sigmaLaser * sigmaLaser;
  kalmanGain << 0, 0;

#if 0
  std::cout << "Estimate State :\n" << estimateState << std::endl;
  std::cout << "State Transition Model :\n" << stateTransitionModel << std::endl;
  std::cout << "Control Input Model :\n" << controlInputModel << std::endl;
  std::cout << "Observation Model :\n" << observationModel << std::endl;
  std::cout << "Prediction Noise Covariance :\n" << predictionNoiseCovariance << std::endl;
  std::cout << "Measurement Noise Covariance :\n" << measurementNoiseCovariance << std::endl;
  std::cout << "Estimate Covariance :\n" << estimateCovariance << std::endl;
  std::cout << "Inovation Covariance :\n" << inovationCovariance << std::endl;
  std::cout << "Kalman Gain :\n" << kalmanGain << std::endl;

#endif
}

KalmanFilterImuLaser::KalmanFilterImuLaser(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string filterID, bool DynamicReconf): kalmanFilterNodeHandle_(nh, "kalman_filter"), kalmanFilterNodeHandlePrivate_(nh_private, "kalman_filter"), kalmanFilterNodeHandlePrivateWithAxis_(nh_private, "kalman_filter/" + filterID), id(filterID)
{
  rosParamInit();

  //init 
  sigmaAcc = inputSigma_; sigmaLaser = measureSigma_;
  dt = 1 / imuHz_;
  inputStartFlag = false;
  measureStartFlag = false;

  //dynamic reconfigure
  //std::string strBegin("~WithoutBias");
  //strBegin.append(filterID);
  if(DynamicReconf)
    {
      server = new dynamic_reconfigure::Server<jsk_quadcopter::StateKalmanFilterConfig>(kalmanFilterNodeHandlePrivateWithAxis_);
      dynamicReconfFunc = boost::bind(&KalmanFilterImuLaser::cfgCallback, this, _1, _2);
      server->setCallback(dynamicReconfFunc);
    }

  estimateState(0) = 0; //position initial
  estimateState(1) = 0; //velocity initial

  correctState(0) = 0; //position initial
  correctState(1) = 0; //velocity initial

  predictState(0) = 0; //position initial
  predictState(1) = 0; //velocity initial

  stateTransitionModel << 1, dt, 0, 1;
  controlInputModel << (dt * dt)/2, dt;
  observationModel << 1, 0;
  observationOnlyVelocityModel << 0, 1;

  predictionNoiseCovariance = (sigmaAcc * sigmaAcc) * controlInputModel * controlInputModel.transpose();
  estimateCovariance << 0, 0, 0, 0;
  inovationCovariance << 0;
  measurementNoiseCovariance(0) = sigmaLaser * sigmaLaser;
  //bad !!
  measurementOnlyVelocityNoiseCovariance(0) = sigmaLaser * sigmaLaser;
  kalmanGain << 0, 0;

}



KalmanFilterImuLaser::~KalmanFilterImuLaser()
{
}

bool KalmanFilterImuLaser::prediction(double input, ros::Time timeStamp)
{
  boost::lock_guard<boost::mutex> lock(kfMutex);

  if(getFilteringStartFlag())
    {
      //ROS_WARN("input");
      //ROS_WARN(" input, %lf", timeStamp.toSec());
      //bais effect
      //float input = rawInput - accBias;

      Eigen::Vector2d estimateHatState
        = stateTransitionModel * estimateState + input * controlInputModel;
      estimateState = estimateHatState;

      Eigen::Matrix2d estimateBarCovariance 
        = stateTransitionModel * estimateCovariance * stateTransitionModel.transpose() 
        + predictionNoiseCovariance;
      estimateCovariance = estimateBarCovariance;

      //debug
      Eigen::Vector2d predictHatState
        = stateTransitionModel * predictState + input * controlInputModel;
      predictState = predictHatState;

      return true;
    }
  else 
    return false;
}

bool KalmanFilterImuLaser::correction(double measurement, ros::Time timeStamp)
{
  boost::lock_guard<boost::mutex> lock(kfMutex);
  if(getFilteringStartFlag())
    {

      //ROS_ERROR("measure is %f", measurement);

      inovationCovariance = observationModel * estimateCovariance * observationModel.transpose()
        + measurementNoiseCovariance;
  
      //std::cout << "Inovation Covariance :\n" << inovationCovariance << std::endl;
      //std::cout << "Estimate Covariance :\n" << estimateCovariance << std::endl;

      kalmanGain = estimateCovariance * observationModel.transpose() * inovationCovariance.inverse();

      //std::cout << "Kalman Gain :\n" << kalmanGain << std::endl;

      Eigen::Vector2d estimateStateTmp 
        = estimateState + kalmanGain * (measurement - observationModel * estimateState);
      estimateState = estimateStateTmp;
      correctState = estimateStateTmp;

      //std::cout << "correctState :\n" << correctState << std::endl;


      Eigen::Matrix2d I; I.setIdentity();
      Eigen::Matrix2d estimateCovarianceTmp;
      estimateCovarianceTmp = (I - kalmanGain * observationModel) * estimateCovariance;
      estimateCovariance = estimateCovarianceTmp;

      return true;
    }
  else 
    return false;
}

bool KalmanFilterImuLaser::correctionOnlyVelocity(double measurement, ros::Time timeStamp)
{
  boost::lock_guard<boost::mutex> lock(kfMutex);
  if(getFilteringStartFlag())
    {

      //ROS_ERROR("measure is %lf", timeStamp.toSec());

      inovationCovariance = observationOnlyVelocityModel * estimateCovariance * observationOnlyVelocityModel.transpose()
        + measurementOnlyVelocityNoiseCovariance;
      // std::cout << "observationOnlyVelocityModel :\n" << observationOnlyVelocityModel << std::endl;
      // std::cout << "measurementOnlyVelocityNoiseCovariance :\n" << measurementOnlyVelocityNoiseCovariance << std::endl;
      // std::cout << "Inovation Covariance :\n" << inovationCovariance << std::endl;
      // std::cout << "Estimate Covariance :\n" << estimateCovariance << std::endl;

      kalmanGain = estimateCovariance * observationOnlyVelocityModel.transpose() * inovationCovariance.inverse();

      //std::cout << "Kalman Gain :\n" << kalmanGain << std::endl;

      Eigen::Vector2d estimateStateTmp 
        = estimateState + kalmanGain * (measurement - observationOnlyVelocityModel * estimateState);
      estimateState = estimateStateTmp;
      correctState = estimateStateTmp;

      //std::cout << "correctState :\n" << correctState << std::endl;


      Eigen::Matrix2d I; I.setIdentity();
      Eigen::Matrix2d estimateCovarianceTmp;
      estimateCovarianceTmp = (I - kalmanGain * observationOnlyVelocityModel) * estimateCovariance;
      estimateCovariance = estimateCovarianceTmp;

      return true;
    }
  else 
    {
      //ROS_WARN("now permission");
      return false;
    }
}



void KalmanFilterImuLaser::imuQuPush(jsk_quadcopter::ImuQuPtr imuQuMsgPtr)
{
  boost::lock_guard<boost::mutex> lock(queueMutex);
  if(getFilteringStartFlag())
    {
      // ROS_ERROR(" push the imu data");
      imuQu.push(imuQuMsgPtr);
    }
  return;
}

bool KalmanFilterImuLaser::imuQuPrediction(ros::Time checkTimeStamp)
{
  boost::lock_guard<boost::mutex> lock(queueMutex);


  if(imuQu.empty())
    {
      //ROS_INFO("no imu data ");
      return false;
    }

  if(checkTimeStamp.toSec() > (imuQu.front()->stamp.toSec()))
    { 
      //ROS_ERROR(" pop the imu data");
      prediction((double)imuQu.front()->acc, imuQu.front()->stamp);
      imuQu.pop();

      return true;
    }
  else
    return false;
}


//for bad measurement step
void KalmanFilterImuLaser::imuQuOnlyPrediction(ros::Time checkTimeStamp)
{
  if(getFilteringStartFlag())
    {
      while(1)
        {
          if(!imuQuPrediction(checkTimeStamp))
            break;
        }
    }
  return;
}



//for time synchronized state
void KalmanFilterImuLaser::imuQuCorrection(ros::Time checkTimeStamp, double measurement, int type)
{
  if(getFilteringStartFlag())
    {


      while(1)
        {
          if(!imuQuPrediction(checkTimeStamp))
            break;
        }

      if(type == 0) // position measurement
        correction(measurement, checkTimeStamp);
      if(type == 1) // position measurement
        correctionOnlyVelocity(measurement, checkTimeStamp);
      //ROS_WARN("correct the data");
    }
  return;
}


void KalmanFilterImuLaser::cfgCallback(jsk_quadcopter::StateKalmanFilterConfig &config, uint32_t level)
{

  if(config.kalmanFilterFlag == true)
    {
      printf("%s axis kf without bias, ", id.c_str());

      switch(level)
        {
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_INPUT_SIGMA:
          sigmaAcc = config.inputSigma;
          //+*+*+*+*+ confirm?? 
          predictionNoiseCovariance 
            = (sigmaAcc * sigmaAcc) * controlInputModel * controlInputModel.transpose();
          printf("change the input sigma\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_MEASURE_SIGMA:
          sigmaLaser   = config.measureSigma;
          measurementNoiseCovariance(0) = sigmaLaser * sigmaLaser;
          printf("change the measure sigma\n");
          break;
        default :
          printf("\n");
          break;
        }
    }

}


double KalmanFilterImuLaser::getEstimatePos()
{
  return estimateState(0);
}

double KalmanFilterImuLaser::getEstimateVel()
{
  return estimateState(1);
}

double KalmanFilterImuLaser::getPredictPos()
{
  return predictState(0);
}

double KalmanFilterImuLaser::getPredictVel()
{
  return predictState(1);
}


double KalmanFilterImuLaser::getCorrectPos()
{
  return correctState(0);
}

double KalmanFilterImuLaser::getCorrectVel()
{
  return correctState(1);
}

void KalmanFilterImuLaser::test()
{
  ROS_WARN("Test OK!");
}

void KalmanFilterImuLaser::getEstimateCovariance(float* covarianceMatrix)
{
  covarianceMatrix[0] = estimateCovariance(0,0);
  covarianceMatrix[1] = estimateCovariance(0,1);
  covarianceMatrix[2] = estimateCovariance(1,0);
  covarianceMatrix[3] = estimateCovariance(1,1);
}


void KalmanFilterImuLaser::setInitImuBias(double initBias)
{
  //set bias
  accBias = initBias;
  // start filtering . danger!
  setInputStartFlag();
}

void KalmanFilterImuLaser::setInputStartFlag()
{
  boost::lock_guard<boost::mutex> lock(kfMutex);
  inputStartFlag = true;
}

void KalmanFilterImuLaser::setMeasureStartFlag(bool flag)
{
  boost::lock_guard<boost::mutex> lock(kfMutex);
  measureStartFlag = flag;
}


bool KalmanFilterImuLaser::getFilteringStartFlag()
{
  if(inputStartFlag && measureStartFlag)
    return true;
  else 
    return false;
}

void KalmanFilterImuLaser::setInitState(double init_pos, double init_vel)
{
  estimateState(0) = init_pos; //position initial
  estimateState(1) = init_vel; //position initial

  correctState(0) = init_pos; //position initial
  correctState(1) = init_vel; //position initial

  predictState(0) = init_pos; //position initial
  predictState(1) = init_vel; //position initial


}


void KalmanFilterImuLaser::rosParamInit()
{
  std::string ns = kalmanFilterNodeHandlePrivateWithAxis_.getNamespace();
  ns.append(" without bias");
  if (!kalmanFilterNodeHandlePrivateWithAxis_.getParam ("inputSigma", inputSigma_))
    inputSigma_ = 0.0;
  printf("%s: inputSigma_ is %.4f\n", ns.c_str(), inputSigma_);
  if (!kalmanFilterNodeHandlePrivateWithAxis_.getParam ("measureSigma", measureSigma_))
    measureSigma_ = 0.0;
  printf("%s: measureSigma_ is %.4f\n", ns.c_str(), measureSigma_);
  if (!kalmanFilterNodeHandlePrivate_.getParam ("imuHz", imuHz_))
    imuHz_ = 100;
  printf("%s: imuHz_ is %.4f\n",kalmanFilterNodeHandlePrivate_.getNamespace().c_str(), imuHz_);
}



KalmanFilterImuLaserBias::KalmanFilterImuLaserBias(ros::NodeHandle nh, 
                                                   ros::NodeHandle nh_private, 
                                                   std::string filterID,
                                                   bool DynamicReconf):kalmanFilterNodeHandle_(nh, "kalman_filter"), kalmanFilterNodeHandlePrivate_(nh_private, "kalman_filter"), kalmanFilterNodeHandlePrivateWithAxis_(nh_private, "kalman_filter/" + filterID), id(filterID) 
{
  rosParamInit();

  //init
  sigmaAcc = inputSigma_;
  sigmaAccBias = biasSigma_;
  sigmaLaser = measureSigma_;
  dt = 1 / imuHz_;

  inputStartFlag = false;
  measureStartFlag = false;


  //dynamic reconfigure
  if(DynamicReconf)
    {
      server = new dynamic_reconfigure::Server<jsk_quadcopter::StateKalmanFilterConfig>(kalmanFilterNodeHandlePrivateWithAxis_);
      dynamicReconfFunc = boost::bind(&KalmanFilterImuLaserBias::cfgCallback, this, _1, _2);
      server->setCallback(dynamicReconfFunc);
    }


  estimateState(0) = 0; //position initial
  estimateState(1) = 0; //position initial
  estimateState(2) = 0; //bias inital

  correctState(0) = 0; //position initial
  correctState(1) = 0; //velocity initial
  correctState(2) = 0; //bias initial

  predictState(0) = 0; //position initial
  predictState(1) = 0; //velocity initial
  predictState(2) = 0; //bias initial

  stateTransitionModel << 1, dt, -dt*dt/2, 0, 1, -dt, 0, 0, 1;
  controlInputModel << (dt * dt)/2, dt, 0;
  observationModel << 1, 0, 0;
  observationOnlyVelocityModel << 0, 1, 0;

  predictionNoiseCovariance = (sigmaAcc * sigmaAcc) * controlInputModel * controlInputModel.transpose();

  predictionNoiseCovariance(2,2) = sigmaAccBias * sigmaAccBias;

  estimateCovariance << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  inovationCovariance << 0;
  measurementNoiseCovariance(0) = sigmaLaser * sigmaLaser;
  //bad !!
  measurementOnlyVelocityNoiseCovariance(0) = sigmaLaser * sigmaLaser;

  kalmanGain << 0, 0, 0;

  kalmanFilterStamp = ros::Time::now();


}


KalmanFilterImuLaserBias::KalmanFilterImuLaserBias(double sigmaPredictAcc,
                                                   double sigmaPredictBias,
                                                   double sigmaMeasure,
                                                   double dtime,
                                                   std::string filterID): id(filterID) 
{

#if 0
  ROS_WARN("kalman filter bias: sigmaPredictAcc is %f, sigmaBiasAcc is %f, sigmaMeasure is %f",
           sigmaPredictAcc,
           sigmaPredictBias,
           sigmaMeasure);

#endif
  //init 
  sigmaAcc = sigmaPredictAcc;
  sigmaAccBias = sigmaPredictBias;
  sigmaLaser = sigmaMeasure;
  dt = dtime;

  inputStartFlag = false;
  measureStartFlag = false;

  //dynamic reconfigure
  std::string strBegin("~WithBias");
  strBegin.append(filterID);
  server = new dynamic_reconfigure::Server<jsk_quadcopter::StateKalmanFilterConfig>(ros::NodeHandle(strBegin));
  dynamicReconfFunc = boost::bind(&KalmanFilterImuLaserBias::cfgCallback, this, _1, _2);
  server->setCallback(dynamicReconfFunc);



  estimateState(0) = 0; //position initial
  estimateState(1) = 0; //position initial
  estimateState(2) = 0; //bias inital

  correctState(0) = 0; //position initial
  correctState(1) = 0; //velocity initial
  correctState(2) = 0; //bias initial

  predictState(0) = 0; //position initial
  predictState(1) = 0; //velocity initial
  predictState(2) = 0; //bias initial

  stateTransitionModel << 1, dt, -dt*dt/2, 0, 1, -dt, 0, 0, 1;
  controlInputModel << (dt * dt)/2, dt, 0;
  observationModel << 1, 0, 0;
  observationOnlyVelocityModel << 0, 1, 0;

  predictionNoiseCovariance = (sigmaAcc * sigmaAcc) * controlInputModel * controlInputModel.transpose();

  predictionNoiseCovariance(2,2) = sigmaAccBias * sigmaAccBias;

  estimateCovariance << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  inovationCovariance << 0;
  measurementNoiseCovariance(0) = sigmaLaser * sigmaLaser;
  //bad !!
  measurementOnlyVelocityNoiseCovariance(0) = sigmaLaser * sigmaLaser;

  kalmanGain << 0, 0, 0;

  kalmanFilterStamp = ros::Time::now();

#if 0
  std::cout << "Estimate State :\n" << estimateState << std::endl;
  std::cout << "State Transition Model :\n" << stateTransitionModel << std::endl;
  std::cout << "Control Input Model :\n" << controlInputModel << std::endl;
  std::cout << "Observation Model :\n" << observationModel << std::endl;
  std::cout << "Prediction Noise Covariance :\n" << predictionNoiseCovariance << std::endl;
  std::cout << "Measurement Noise Covariance :\n" << measurementNoiseCovariance << std::endl;
  std::cout << "Estimate Covariance :\n" << estimateCovariance << std::endl;
  std::cout << "Inovation Covariance :\n" << inovationCovariance << std::endl;
  std::cout << "Kalman Gain :\n" << kalmanGain << std::endl;

#endif

}

KalmanFilterImuLaserBias::~KalmanFilterImuLaserBias()
{
}

bool KalmanFilterImuLaserBias::prediction(double input, ros::Time stamp)
{
  boost::lock_guard<boost::mutex> lock(kfMutex);
  if(getFilteringStartFlag())
    {
      //ROS_WARN("input: %lf", stamp.toSec());

      Eigen::Vector3d estimateHatState
        = stateTransitionModel * estimateState + input * controlInputModel;
      estimateState = estimateHatState;

      Eigen::Matrix3d estimateBarCovariance 
        = stateTransitionModel * estimateCovariance * stateTransitionModel.transpose() 
        + predictionNoiseCovariance;
      estimateCovariance = estimateBarCovariance;

      //debug
      Eigen::Vector3d predictHatState
        = stateTransitionModel * predictState + input * controlInputModel;
      predictState = predictHatState;

      kalmanFilterStamp = stamp;

      return true;
    }
  else 
    return false;
}

//for bad measurement step
void KalmanFilterImuLaserBias::imuQuOnlyPrediction(ros::Time checkTimeStamp)
{
  if(getFilteringStartFlag())
    {
      while(1)
        {
          if(!imuQuPrediction(checkTimeStamp))
            break;
        }
    }
  return;
}



double KalmanFilterImuLaserBias::correction(double measurement, ros::Time stamp)
{
  boost::lock_guard<boost::mutex> lock(kfMutex);
  if(getFilteringStartFlag())
    {
      //ROS_ERROR("measure: %lf", stamp.toSec());

      inovationCovariance = observationModel * estimateCovariance * observationModel.transpose()
        + measurementNoiseCovariance;
  
      //std::cout << "Inovation Covariance :\n" << inovationCovariance << std::endl;
      //std::cout << "Estimate Covariance :\n" << estimateCovariance << std::endl;

      kalmanGain = estimateCovariance * observationModel.transpose() * inovationCovariance.inverse();

      //std::cout << "Kalman Gain :\n" << kalmanGain << std::endl;

      Eigen::Vector3d estimateStateTmp 
        = estimateState + kalmanGain * (measurement - observationModel * estimateState);
      estimateState = estimateStateTmp;
      correctState = estimateStateTmp;

      Eigen::Matrix3d I; I.setIdentity();
      Eigen::Matrix3d estimateCovarianceTmp;
      estimateCovarianceTmp = (I - kalmanGain * observationModel) * estimateCovariance;
      estimateCovariance = estimateCovarianceTmp;

      // if(kalmanFilterStamp.toSec() > stamp.toSec()) 
      //   ROS_WARN("%s Axis Kalman Filter: wrong time stamp, %.3f!!", id.c_str(), kalmanFilterStamp.toSec() - stamp.toSec());

      return (kalmanFilterStamp.toSec() - stamp.toSec());
    }
  else 
    {
      //ROS_WARN("now permission");
      return 0;
    }
}

double KalmanFilterImuLaserBias::correctionOnlyVelocity(double measurement, ros::Time stamp)
{
  boost::lock_guard<boost::mutex> lock(kfMutex);
  if(getFilteringStartFlag())
    {
      //ROS_WARN("only velocity");

      inovationCovariance = observationOnlyVelocityModel * estimateCovariance * observationOnlyVelocityModel.transpose()
        + measurementOnlyVelocityNoiseCovariance;
  

      //std::cout << "Inovation Covariance :\n" << inovationCovariance << std::endl;
      //std::cout << "Estimate Covariance :\n" << estimateCovariance << std::endl;

      kalmanGain = estimateCovariance * observationOnlyVelocityModel.transpose() * inovationCovariance.inverse();

      //std::cout << "Kalman Gain :\n" << kalmanGain << std::endl;

      Eigen::Vector3d estimateStateTmp 
        = estimateState + kalmanGain * (measurement - observationOnlyVelocityModel * estimateState);
      estimateState = estimateStateTmp;
      correctState = estimateStateTmp;

      Eigen::Matrix3d I; I.setIdentity();
      Eigen::Matrix3d estimateCovarianceTmp;
      estimateCovarianceTmp = (I - kalmanGain * observationOnlyVelocityModel) * estimateCovariance;
      estimateCovariance = estimateCovarianceTmp;

      // if(kalmanFilterStamp.toSec() > stamp.toSec()) 
      //   ROS_WARN("%s Axis Kalman Filter: wrong time stamp, %.3f!!", id.c_str(), kalmanFilterStamp.toSec() - stamp.toSec());

      return (kalmanFilterStamp.toSec() - stamp.toSec());
    }
  else 
    {
      //ROS_WARN("now permission");
      return 0;
    }
}
  


void KalmanFilterImuLaserBias::imuQuPush(jsk_quadcopter::ImuQuPtr imuQuMsgPtr)
{
  boost::lock_guard<boost::mutex> lock(queueMutex);
  if(getFilteringStartFlag())
    {
      //ROS_ERROR(" push the imu data");
      imuQu.push(imuQuMsgPtr);
    }
  return;
}

bool KalmanFilterImuLaserBias::imuQuPrediction(ros::Time checkTimeStamp)
{
  boost::lock_guard<boost::mutex> lock(queueMutex);
  if(imuQu.empty())
    {
      //ROS_INFO("               no imu data ");
      return false;
    }

  if(checkTimeStamp.toSec() > (imuQu.front()->stamp.toSec()))
    { 
      //ROS_ERROR(" pop the imu data");
      prediction((double)imuQu.front()->acc, imuQu.front()->stamp);
      imuQu.pop();

      return true;
    }
  else
    return false;
}

//for time synchronized state
void KalmanFilterImuLaserBias::imuQuCorrection(ros::Time checkTimeStamp, double measurement, int type)
{
  if(getFilteringStartFlag())
    {
      while(1)
        {
          if(!imuQuPrediction(checkTimeStamp))
            break;
        }

      if(type == 0) // position measurement
        correction(measurement, checkTimeStamp);
      if(type == 1) // position measurement
        correctionOnlyVelocity(measurement, checkTimeStamp);
      //ROS_WARN("correct the data");
    }
  return;
}

void KalmanFilterImuLaserBias::cfgCallback(jsk_quadcopter::StateKalmanFilterConfig &config, uint32_t level)
{

  if(config.kalmanFilterFlag == true)
    {
      printf("%s axis kf with bias, ", id.c_str());
      switch(level)
        {
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_INPUT_SIGMA:
          sigmaAcc = config.inputSigma;
          //+*+*+*+*+ confirm?? 
          predictionNoiseCovariance 
            = (sigmaAcc * sigmaAcc) * controlInputModel * controlInputModel.transpose();
          predictionNoiseCovariance(2,2) = sigmaAccBias * sigmaAccBias;
          printf("change the input sigma\n");
          std::cout << "Prediction Noise Covariance :\n" << predictionNoiseCovariance << std::endl;
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_BIAS_SIGMA:
          sigmaAccBias = config.biasSigma;
          predictionNoiseCovariance 
            = (sigmaAcc * sigmaAcc) * controlInputModel * controlInputModel.transpose();
          predictionNoiseCovariance(2,2) = sigmaAccBias * sigmaAccBias;
          printf("change the bias sigma\n");
          std::cout << "Prediction Noise Covariance :\n" << predictionNoiseCovariance << std::endl;
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_MEASURE_SIGMA:
          sigmaLaser   = config.measureSigma;
          measurementNoiseCovariance(0) = sigmaLaser * sigmaLaser;
          printf("change the measure sigma\n");
          break;
        default :
          printf("\n");
          break;
        }
    }


}



double KalmanFilterImuLaserBias::getEstimatePos()
{
  return estimateState(0);
}

double KalmanFilterImuLaserBias::getEstimateVel()
{
  return estimateState(1);
}

double KalmanFilterImuLaserBias::getEstimateBias()
{
  return estimateState(2);
}


double KalmanFilterImuLaserBias::getPredictPos()
{
  return predictState(0);
}

double KalmanFilterImuLaserBias::getPredictVel()
{
  return predictState(1);
}


double KalmanFilterImuLaserBias::getCorrectPos()
{
  return correctState(0);
}

double KalmanFilterImuLaserBias::getCorrectVel()
{
  return correctState(1);
}


void KalmanFilterImuLaserBias::getEstimateCovariance(float* covarianceMatrix)
{
  covarianceMatrix[0] = estimateCovariance(0,0);
  covarianceMatrix[1] = estimateCovariance(0,1);
  covarianceMatrix[2] = estimateCovariance(0,2);
  covarianceMatrix[3] = estimateCovariance(1,0);
  covarianceMatrix[4] = estimateCovariance(1,1);
  covarianceMatrix[5] = estimateCovariance(1,2);
  covarianceMatrix[6] = estimateCovariance(2,0);
  covarianceMatrix[7] = estimateCovariance(2,1);
  covarianceMatrix[8] = estimateCovariance(2,2);
}

void KalmanFilterImuLaserBias::setInitImuBias(double initBias)
{
  //set bias
  estimateState(2) = initBias; //bias inital
  correctState(2) = initBias; //bias initial
  predictState(2) = initBias; //bias initial

  // start filtering . danger!
  setInputStartFlag();
}

void KalmanFilterImuLaserBias::setInputStartFlag()
{
  inputStartFlag = true;
}

void KalmanFilterImuLaserBias::setMeasureStartFlag(bool flag)
{
  measureStartFlag = flag;
}

bool KalmanFilterImuLaserBias::getFilteringStartFlag()
{
  if(inputStartFlag && measureStartFlag)
    return true;
  else 
    return false;
}

void KalmanFilterImuLaserBias::setInitState(double init_pos, double init_vel)
{
  estimateState(0) = init_pos; //position initial
  estimateState(1) = init_vel; //position initial

  correctState(0) = init_pos; //position initial
  correctState(1) = init_vel; //position initial

  predictState(0) = init_pos; //position initial
  predictState(1) = init_vel; //position initial

}

void KalmanFilterImuLaserBias::rosParamInit()
{
  std::string ns = kalmanFilterNodeHandlePrivateWithAxis_.getNamespace();
  ns.append(" with bias");
  if (!kalmanFilterNodeHandlePrivateWithAxis_.getParam ("inputSigma", inputSigma_))
    inputSigma_ = 0.0;
  printf("%s: inputSigma_ is %.4f\n", ns.c_str(), inputSigma_);
  if (!kalmanFilterNodeHandlePrivateWithAxis_.getParam ("biasSigma", biasSigma_))
    biasSigma_ = 0.0;
  printf("%s: biasSigma_ is %.4f\n", ns.c_str(), biasSigma_);
  if (!kalmanFilterNodeHandlePrivateWithAxis_.getParam ("measureSigma", measureSigma_))
    measureSigma_ = 0.0;
  printf("%s: measureSigma_ is %.4f\n", ns.c_str(), measureSigma_);
  if (!kalmanFilterNodeHandlePrivate_.getParam ("imuHz", imuHz_))
    imuHz_ = 100;
  printf("%s: imuHz_ is %.4f\n",kalmanFilterNodeHandlePrivate_.getNamespace().c_str(), imuHz_);

}


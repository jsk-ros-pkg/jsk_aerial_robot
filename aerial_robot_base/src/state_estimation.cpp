/*
  2015 05 16 
  1. the prioripty of the sensor => state

  +*+*+*+* 最重要事項：関数のstatic変数はすべて同じポインタに保存されるので、同じ関数を複数呼ぶ場合、中身のstatic変数はお互い
>  
  1. kalman filters of optical flow(vel_x, vel_y, pos_z) do not use accurate timestamp => for throwing?
   on the other hand, the kalman filters for laser-imu do use accurate timestamp => use queue
  ==> the timestamp do not have any important effect on the accuracy of kalman filter, in the case of 100Hz   

  old1. X/Y axis の問題: フィルタ遅延によるもの？ --実は係数をあげればよい(Z axisとX/Y axixの周波数応答が違う)
  old2. the input of slam data should be after the input imu data?

  
*/

#include "aerial_robot_base/state_estimation.h"

RigidEstimator::RigidEstimator(ros::NodeHandle nh,
                       ros::NodeHandle nh_private,
                       bool simulation_flag) : Estimator()
{
  estimatorNodeHandle_ = ros::NodeHandle(nh, "estimator"); 
  estimatorNodeHandlePrivate_ = ros::NodeHandle(nh_private, "estimator");

  rosParamInit();

  //iir filter
  filterAccX_ = new FirFilter(128);
  filterAccY_ = new FirFilter(128);
  filterAccZ_ = new FirFilter(32);

  //kalman filter
  if (kalmanFilterFlag_)
    {
      std::string x("X");
      std::string y("Y");
      std::string z("Z");
      std::string xOpt("XOpt");
      std::string yOpt("YOpt");
      std::string zOpt("ZOpt");

      std::string debug1("Debug1");
      std::string debug2("Debug2");

      bool use_dynamic_reconfigure = true;
      kfX_ = new KalmanFilterImuLaser(nh, nh_private, x);
      kfY_ = new KalmanFilterImuLaser(nh, nh_private, y);
      kfZ_ = new KalmanFilterImuLaser(nh, nh_private, z, use_dynamic_reconfigure);
      kfbX_ = new KalmanFilterImuLaserBias(nh, nh_private, x, use_dynamic_reconfigure); 
      kfbY_ = new KalmanFilterImuLaserBias(nh, nh_private, y, use_dynamic_reconfigure);
      kfbZ_ = new KalmanFilterImuLaserBias(nh, nh_private, z);

      // for optical flow
      kfXForOpt_ = new KalmanFilterImuLaser(nh, nh_private,xOpt);
      kfYForOpt_ = new KalmanFilterImuLaser(nh, nh_private,yOpt);
      kfZForOpt_ = new KalmanFilterImuLaser(nh, nh_private,zOpt);
      kfbXForOpt_ = new KalmanFilterImuLaserBias(nh, nh_private, xOpt, use_dynamic_reconfigure); 
      kfbYForOpt_ = new KalmanFilterImuLaserBias(nh, nh_private, yOpt, use_dynamic_reconfigure);
      kfbZForOpt_ = new KalmanFilterImuLaserBias(nh, nh_private, zOpt, use_dynamic_reconfigure);

      if (kalmanFilterDebug_)
        {
          kf1_ = new KalmanFilterImuLaserBias(nh, nh_private,debug1, use_dynamic_reconfigure);
          kf2_ = new KalmanFilterImuLaserBias(nh, nh_private,debug2, use_dynamic_reconfigure);
        }
      else
        {
          kf1_ = NULL; kf2_ = NULL;
        }
    }
  else
    {
      kfX_ = NULL; kfY_ = NULL; kfZ_ = NULL; kfbX_ = NULL; kfbY_ = NULL; kfbZ_ = NULL;
      kfXForOpt_ = NULL; kfYForOpt_ = NULL; kfZForOpt_ = NULL; 
      kfbXForOpt_ = NULL; kfbYForOpt_ = NULL; kfbZForOpt_ = NULL;
      kf1_ = NULL; kf2_ = NULL;
    }

  tfB_          = new tf::TransformBroadcaster();

  imuData_      = new ImuData(nh, 
                              nh_private, 
                              this,
                              kalmanFilterFlag_,
                              kfX_, kfY_, kfZ_,
                              kfbX_, kfbY_, kfbZ_,
                              kfXForOpt_,
                              kfYForOpt_,
                              kfZForOpt_,
                              kfbXForOpt_,
                              kfbYForOpt_,
                              kfbZForOpt_,
                              kalmanFilterDebug_,
                              kalmanFilterAxis_,
                              kf1_, kf2_,
                              filterAccX_,
                              filterAccY_,
                              filterAccZ_,
                              simulation_flag);

  slamData_     = new SlamData(nh,
                               nh_private,
                               this,
                               kalmanFilterFlag_,
                               kalmanFilterDebug_,
                               kalmanFilterAxis_,
                               kfX_, kfY_, kfZ_,
                               kfbX_, kfbY_, kfbZ_,
                               kf1_, kf2_); 

  opticalFlowData_   = new OpticalFlowData(nh,
                                           nh_private,
                                           this,
                                           kalmanFilterFlag_,
                                           kfXForOpt_, kfYForOpt_, kfZForOpt_,
                                           kfbXForOpt_, kfbYForOpt_, kfbZForOpt_);

  mirrorModule_ = new MirrorModule(nh, nh_private,
                                   this,
                                   kalmanFilterFlag_, kalmanFilterDebug_,
                                   kfZ_, kfbZ_);

  if(mocapFlag_)
      mocapData_ = new MocapData(nh, nh_private, this);
  
  simulationFlag = simulation_flag;
}

RigidEstimator::~RigidEstimator()
{
  if(kalmanFilterFlag)
    {
      delete kfX_;
      delete kfY_;
      delete kfZ_;
      delete kfbX_;
      delete kfbY_;
      delete kfbZ_;
      delete kfXForOpt_;
      delete kfYForOpt_;
      delete kfZForOpt_;
      delete kfbXForOpt_;
      delete kfbYForOpt_;
      delete kfbZForOpt_;
    }
  if(kalmanFilterDebug)
    {
      delete kf1_;
      delete kf2_;
    }


  delete tfB_;
  delete imuData_;
  delete opticalFlowData_;
  delete slamData_;
  delete mirrorModule_;

  delete filterAccX_;
  delete filterAccY_;
  delete filterAccZ_;

  printf("   deleted tfB_, imuData_, slamData_, mirrorModule_, kalmanFilters from estimator1");
}

//TODO: unify those two func
float RigidEstimator::getStatePosX()
{
  if(useOuterPoseEstimate & X_AXIS)
    return  outerEstimatePosX;
  else
    return  kfbX_->getEstimatePos();
}

float RigidEstimator::getStatePosXc()
{
  if(useOuterPoseEstimate & X_AXIS)
    {
      float state_pos_xc
        = outerEstimatePosX * cos(outerEstimatePsiBody) + outerEstimatePosY * sin(outerEstimatePsiBody);
      return  state_pos_xc;
    }
  else
    return  kfbXForOpt_->getEstimatePos();
}

float RigidEstimator::getStateVelX()
{
  if(useOuterVelEstimate & X_AXIS)
      return  outerEstimateVelX;
  else
    return  kfbX_->getEstimateVel();
}

float RigidEstimator::getStateVelXc()
{
  if(useOuterVelEstimate & X_AXIS)
    {
      float state_vel_xc
        = outerEstimateVelX * cos(outerEstimatePsiBody) + outerEstimateVelY * sin(outerEstimatePsiBody);
      return  state_vel_xc;
    }
  else
    return  kfbXForOpt_->getEstimateVel();
}

float RigidEstimator::getStateAccXb()
{
  return imuData_->getAccXbValue();
}

float RigidEstimator::getStatePosY()
{
 if(useOuterPoseEstimate & Y_AXIS)
      return  outerEstimatePosY;
  else
    return  kfbY_->getEstimatePos();
}

float RigidEstimator::getStatePosYc()
{
 if(useOuterPoseEstimate & Y_AXIS)
   {
      float state_pos_yc
        = -outerEstimatePosX * sin(outerEstimatePsiBody) + outerEstimatePosY * cos(outerEstimatePsiBody);
      return  state_pos_yc;
   }
  else
  return  kfbYForOpt_->getEstimatePos();
}

float RigidEstimator::getStateVelY()
{
 if(useOuterVelEstimate & Y_AXIS)
     return  outerEstimateVelY;
  else
    return  kfbY_->getEstimateVel();
}

float RigidEstimator::getStateVelYc()
{
 if(useOuterVelEstimate & Y_AXIS)
   {
      float state_vel_yc
        = -outerEstimateVelX * sin(outerEstimatePsiBody) + outerEstimateVelY * cos(outerEstimatePsiBody);
      return  state_vel_yc;
   }
  else
    return  kfbYForOpt_->getEstimateVel();
}

float RigidEstimator::getStateAccYb()
{
  return imuData_->getAccYbValue();
}

float RigidEstimator::getStatePosZ()
{
  if(useOuterPoseEstimate & Z_AXIS)
      return  outerEstimatePosZ;
  else
    {
      if(altitudeControlMode_ == LASER_MIRROR)
        return kfZ_->getEstimatePos();
      else if(altitudeControlMode_ == SONAR)
        return kfZForOpt_->getEstimatePos();
      else
        return 0;
    }
}
float RigidEstimator::getStateVelZ()
{
  if(useOuterVelEstimate & Z_AXIS)
      return  outerEstimateVelZ;
  else
    {
      if(altitudeControlMode_ == LASER_MIRROR)
        return kfZ_->getEstimateVel();
      else if(altitudeControlMode_ == SONAR)
        return kfZForOpt_->getEstimateVel();
      else
        return 0;
    }
}

float RigidEstimator::getStateAccZb()
{
  return imuData_->getAccZbValue();
}

float RigidEstimator::getStateTheta()
{
  if(useOuterPoseEstimate & PITCH_AXIS)
    return  outerEstimateTheta;
  else
    return imuData_->getPitchValue();
}
float RigidEstimator::getStatePhy()
{
  if(useOuterPoseEstimate & ROLL_AXIS)
    return  outerEstimatePhy;
  else
    return imuData_->getRollValue();
}

float RigidEstimator::getStatePsiCog()
{
  if(useOuterPoseEstimate & YAW_AXIS)
    {      
      return  outerEstimatePsiCog;
    }
  else
    {
      if(useOuterYawEst_)
        return slamData_->getPsiSlamValue();
      else
        return 0;  //+*+*+ fixed point
    }
}
float RigidEstimator::getStateVelPsiCog()
{
  if(useOuterVelEstimate & YAW_AXIS)
    {      
      return  outerEstimateVelPsiCog;
    }
  else
    {
      if(useOuterYawEst_)
        return   slamData_->getVelPsiSlamValue();
      else 
        return 0;   //+*+*+ fixed point
    }
}

float RigidEstimator::getStatePsiBody()
{
  if(useOuterPoseEstimate & YAW_AXIS)
    {      
      return  outerEstimatePsiBody;
    }
  else
    {
      if(useOuterYawEst_)
        return slamData_->getPsiSlamValue();
      else
        return 0;  //+*+*+ fixed point
    }
}
float RigidEstimator::getStateVelPsiBody()
{
  if(useOuterVelEstimate & YAW_AXIS)
    {      
      return  outerEstimateVelPsiBody;
    }
  else
    {
      if(useOuterYawEst_)
        return   slamData_->getVelPsiSlamValue();
      else 
        return 0;   //+*+*+ fixed point
    }
}


float RigidEstimator::getStateVelXOpt()
{
  return opticalFlowData_->getRawVelX();
}
float RigidEstimator::getStateVelYOpt()
{
  return opticalFlowData_->getRawVelY();
}


bool RigidEstimator::getRocketStartFlag()
{
   if(altitudeControlMode_ == SONAR)
      return opticalFlowData_->getRocketStartFlag();
  else
      return false;
}

void RigidEstimator::setRocketStartFlag()
{
  if(altitudeControlMode_ == SONAR)
    opticalFlowData_->setRocketStartFlag();
}

void RigidEstimator::tfPublish()
{

  //TODO mutex
  if(simulationFlag)
    tfStamp_ = mirrorModule_->getScanStamp();
  else
    tfStamp_ = ros::Time::now();



  tf::Transform laser_to_baselink_;
  tf::Transform footprint_to_laser_;
  tf::Transform laser_to_camera_;
  tf::Quaternion tmp_;

  //send the laser -> quadcopter_base
  laser_to_baselink_.setOrigin(tf::Vector3(0.0, 0.0, laserToBaselinkDistance_));
  tmp_.setRPY(0.0 , 0.0 , 0.0);
  laser_to_baselink_.setRotation(tmp_);
  tfB_->sendTransform(tf::StampedTransform(laser_to_baselink_, tfStamp_, laserFrame_,
        				   baselinkFrame_));

  //send the laser -> camera
  laser_to_camera_.setOrigin(tf::Vector3(0.02, 0.0, -0.04));
  tmp_.setRPY(0.0 , 0.0 , 0.0);
  laser_to_camera_.setRotation(tmp_);
  tfB_->sendTransform(tf::StampedTransform(laser_to_camera_, tfStamp_, laserFrame_,
        				   cameraFrame_));

  tmp_.setRPY((getStatePhy()), getStateTheta(), 0); 

  footprint_to_laser_.setRotation(tmp_);

  footprint_to_laser_.setOrigin(tf::Vector3(0.0, 0.0, getStatePosZ() + getPosZOffset() - mirrorModuleArmLength_));

  tfB_->sendTransform(tf::StampedTransform(footprint_to_laser_, tfStamp_,
  					   baseFootprintFrame_, laserFrame_));

}

float RigidEstimator::getLaserToImuDistance()
{
  return laserToBaselinkDistance_;
}

void RigidEstimator::setKalmanFilterBoardToAsctec()
{
  imuData_->setKalmanFilterBoardToAsctec();
}

void RigidEstimator::rosParamInit()
{
  std::string ns = estimatorNodeHandlePrivate_.getNamespace();

  if (!estimatorNodeHandlePrivate_.getParam ("altitudeControlMode", altitudeControlMode_))
    altitudeControlMode_ = 0;
  printf("%s: altitudeControlMode_ is %d\n", ns.c_str(), altitudeControlMode_);

  if (!estimatorNodeHandlePrivate_.getParam ("useOuterYawEst", useOuterYawEst_))
    useOuterYawEst_ = false;
  printf("%s: useOuterYawEst is %s\n", ns.c_str(), useOuterYawEst_ ? ("true") : ("false"));

  if (!estimatorNodeHandlePrivate_.getParam ("baselinkFrame", baselinkFrame_))
    baselinkFrame_ = "unknown";
  printf("%s: baselinkFrame_ is %s\n", ns.c_str(), baselinkFrame_.c_str());

  if (!estimatorNodeHandlePrivate_.getParam ("baseFootprintFrame", baseFootprintFrame_))
    baseFootprintFrame_ = "unknown";
  printf("%s: baseFootprintFrame_ is %s\n", ns.c_str(), baseFootprintFrame_.c_str());

  if (!estimatorNodeHandlePrivate_.getParam ("laserFrame", laserFrame_))
    laserFrame_ = "unknown";
  printf("%s: laserFrame_ is %s\n", ns.c_str(), laserFrame_.c_str());

  if (!estimatorNodeHandlePrivate_.getParam ("cameraFrame", cameraFrame_))
    laserFrame_ = "unknown";
  printf("%s: cameraFrame_ is %s\n", ns.c_str(), cameraFrame_.c_str());

  if (!estimatorNodeHandlePrivate_.getParam ("laserToBaselinkDistance", laserToBaselinkDistance_))
    laserToBaselinkDistance_ = 0;
  printf("%s: laserToBaselinkDistance_ is %.3f\n", ns.c_str(), laserToBaselinkDistance_);

  if (!estimatorNodeHandlePrivate_.getParam ("mirrorModuleArmLength", mirrorModuleArmLength_))
    mirrorModuleArmLength_ = 0;
  printf("%s: mirrorModuleArmLength_ is %.3f\n", ns.c_str(), mirrorModuleArmLength_);

  //*** kalman filter
  if (!estimatorNodeHandlePrivate_.getParam ("kalmanFilterFlag", kalmanFilterFlag_))
    kalmanFilterFlag_ = false;
  printf("%s: kalmanFilterFlag is %s\n", ns.c_str(), kalmanFilterFlag_ ? ("true") : ("false"));

  //*** kalman filter debug
  if (!estimatorNodeHandlePrivate_.getParam ("kalmanFilterDebug", kalmanFilterDebug_))
    kalmanFilterDebug_ = false;
  printf("%s: kalmanFilterDebug is %s\n", ns.c_str(), kalmanFilterDebug_ ? ("true") : ("false"));

  if (!estimatorNodeHandlePrivate_.getParam ("kalmanFilterAxis", kalmanFilterAxis_))
    kalmanFilterAxis_ = 0;
  printf("%s: kalmanFilterAxis_ is %d\n", ns.c_str(), kalmanFilterAxis_);

  //*** mocap 
  if (!estimatorNodeHandlePrivate_.getParam ("mocapFlag", mocapFlag_))
    mocapFlag_ = false;
  printf("%s: mocapFlag is %s\n", ns.c_str(), mocapFlag_ ? ("true") : ("false"));


}

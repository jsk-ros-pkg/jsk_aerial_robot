#ifndef SLAM_DATA_H
#define SLAM_DATA_H

//* ros
#include <ros/ros.h>
#include <jsk_quadcopter/SlamDebug.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_quadcopter/state_estimation.h>
#include <jsk_quadcopter/kalman_filter.h>
#include <jsk_quadcopter/digital_filter.h>
#include <tf/transform_broadcaster.h>

class SlamData
{
 public:
 SlamData(ros::NodeHandle nh,
          ros::NodeHandle nh_private,
          Estimator* state_estimator,
          bool kalman_filter_flag,
          bool kalman_filter_debug,
          int kalman_filter_axis,
          KalmanFilterImuLaser *kf_x, 
          KalmanFilterImuLaser *kf_y,
          KalmanFilterImuLaser *kf_z,
          KalmanFilterImuLaserBias *kfb_x, 
          KalmanFilterImuLaserBias *kfb_y,
          KalmanFilterImuLaserBias *kfb_z,
          KalmanFilterImuLaserBias *kf1,
          KalmanFilterImuLaserBias *kf2)
   : slamDataNodeHandle_(nh, "slam"),
    slamDataNodeHandlePrivate_(nh_private, "slam")
      {
        rosParamInit();

        slamPub_ = slamDataNodeHandle_.advertise<jsk_quadcopter::SlamDebug>("debug", 10);
        slamSub_ = slamDataNodeHandle_.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 5, boost::bind(&SlamData::poseStampedCallback, this, _1, state_estimator));

        posX = 0;
        posY = 0;
        posZSlam = 0;
        psiSlam = 0; 

        rawPosX = 0;
        rawPosY = 0;
        rawPosZSlam = 0;
        rawPsiSlam = 0;

        velX = 0;
        velY = 0;
        velZSlam = 0;
        velPsiSlam = 0; 

        rawVelX = 0;
        rawVelY = 0;
        rawVelZSlam = 0;
        rawVelPsiSlam = 0;
                                                 
        filterX_ =     IirFilter((float)slamRxFreqX_, 
                                 (float)slamCutoffPosFreqX_, 
                                 (float)slamCutoffVelFreqX_, 
                                 (float)slamFilterVelValThreX_, 
                                 (float)slamFilterVelChangeRateThreX_);

        filterY_ =     IirFilter((float)slamRxFreqY_, 
                                 (float)slamCutoffPosFreqY_, 
                                 (float)slamCutoffVelFreqY_, 
                                 (float)slamFilterVelValThreY_, 
                                 (float)slamFilterVelChangeRateThreY_);

        filterPsi_ =     IirFilter((float)slamRxFreqPsi_, 
                                   (float)slamCutoffPosFreqPsi_, 
                                   (float)slamCutoffVelFreqPsi_, 
                                   (float)slamFilterVelValThrePsi_, 
                                   (float)slamFilterVelChangeRateThrePsi_);


        kalmanFilterFlag = kalman_filter_flag;
        kalmanFilterDebug = kalman_filter_debug;
        kalmanFilterAxis = kalman_filter_axis;

        kfX_ = kf_x; kfY_ = kf_y; kfZ_ = kf_z;
        kfbX_ = kfb_x; kfbY_ = kfb_y; kfbZ_ = kfb_z;
        kf1_ = kf1; kf2_ = kf2; 

      }
  ~SlamData()
    {
    }


  void setPosXValue(float pos_x_value)
  {
    posX = pos_x_value;
  }
  void setPosYValue(float pos_y_value)
  {
    posY = pos_y_value;
  }
  void setPosZSlamValue(float pos_z_value)
  {
    posZSlam = pos_z_value;
  }

  void setPsiSlamValue(float psi_value)
  {
    psiSlam = psi_value;
  }


  float getPosXValue()
  {
    return  posX;
  }

  float getPosYValue()
  {
    return posY;
  }
  float getPosZSlamValue()
  {
    return posZSlam;
  }

  float getPsiSlamValue()
  {
    return psiSlam;
  }

  void setRawPosXValue(float raw_pos_x_value)
  {
    rawPosX = raw_pos_x_value;
  }

  void setRawPosYValue(float raw_pos_y_value)
  {
    rawPosY = raw_pos_y_value;
  }

  void setRawPosZSlamValue(float raw_pos_z_value)
  {
    rawPosZSlam = raw_pos_z_value;
  }

  void setRawPsiSlamValue(float raw_psi_value)
  {
    rawPsiSlam = raw_psi_value;
  }

  float getRawPosXValue()
  {
    return rawPosX;
  }
  float getRawPosYValue()
  {
    return rawPosY;
  }

  float getRawPosZSlamValue()
  {
    return rawPosZSlam;
  }

  float getRawPsiSlamValue()
  {
    return rawPsiSlam;
  }

  void setVelXValue(float vel_x_value)
  {
    velX = vel_x_value;
  }

  void setVelYValue(float vel_y_value)
  {
    velY = vel_y_value;
  }

  void setVelZSlamValue(float vel_z_value)
  {
    velZSlam = vel_z_value;
  }

  void setVelPsiSlamValue(float vel_psi_value)
  {
    velPsiSlam = vel_psi_value;
  }

  float getVelXValue()
  {
    return velX;
  }

  float getVelYValue()
  {
    return velY;
  }

  float getVelZSlamValue()
  {
    return velZSlam;
  }

  float getVelPsiSlamValue()
  {
    return velPsiSlam;
  }

  void setRawVelXValue(float raw_vel_x_value)
  {
    rawVelX = raw_vel_x_value;
  }

  void setRawVelYValue(float raw_vel_y_value)
  {
    rawVelY = raw_vel_y_value;
  }

  void setRawVelZSlamValue(float raw_vel_z_value)
  {
    rawVelZSlam = raw_vel_z_value;
  }

  void setRawVelPsiSlamValue(float raw_vel_psi_value)
  {
    rawVelPsiSlam = raw_vel_psi_value;
  }

  float getRawVelXValue()
  {
    return rawVelX;
  }

  float getRawVelYValue()
  {
    return rawVelY;
  }

  float getRawVelZSlamValue()
  {
    return rawVelZSlam;
  }

  float getRawVelPsiSlamValue()
  {
    return rawVelPsiSlam;
  }

  const static int X = 0;
  const static int Y = 1;
  const static int Z = 2;
  const static int THETA = 3;
  const static int PHY = 4;
  const static int PSI = 5;


 private:
  ros::NodeHandle slamDataNodeHandle_;
  ros::NodeHandle slamDataNodeHandlePrivate_;
  ros::Publisher slamPub_;
  ros::Subscriber slamSub_;
  ros::Time slamStamp_;


  bool kalmanFilterFlag;
  KalmanFilterImuLaser *kfX_;
  KalmanFilterImuLaser *kfY_;
  KalmanFilterImuLaser *kfZ_;

  KalmanFilterImuLaserBias *kfbX_;
  KalmanFilterImuLaserBias *kfbY_;
  KalmanFilterImuLaserBias *kfbZ_;

  bool kalmanFilterDebug;
  int  kalmanFilterAxis;
  KalmanFilterImuLaserBias *kf1_;
  KalmanFilterImuLaserBias *kf2_;


  double posX;
  double posY;
  double posZSlam;
  double psiSlam; //yaw angle
  
  double rawPosX;
  double rawLaserPosX;
  double rawPosY;
  double rawLaserPosY;
  double rawPosZSlam;
  double rawPsiSlam;

  double velX;
  double velY;
  double velZSlam;
  double velPsiSlam; 

  double rawVelX;
  double rawVelY;
  double rawLaserVelX;
  double rawLaserVelY;
  double rawVelZSlam;
  double rawVelPsiSlam;
  //+*+*+*+ filter types
  double slamRxFreqX_;
  double slamRxFreqY_;
  double slamRxFreqZ_;
  double slamRxFreqTheta_;
  double slamRxFreqPhy_;
  double slamRxFreqPsi_;
  double slamCutoffPosFreqX_;
  double slamCutoffPosFreqY_;
  double slamCutoffPosFreqZ_;
  double slamCutoffPosFreqTheta_;
  double slamCutoffPosFreqPhy_;
  double slamCutoffPosFreqPsi_;
  double slamCutoffVelFreqX_;
  double slamCutoffVelFreqY_;
  double slamCutoffVelFreqZ_;
  double slamCutoffVelFreqTheta_;
  double slamCutoffVelFreqPhy_;
  double slamCutoffVelFreqPsi_;
  double slamFilterVelValThreX_;
  double slamFilterVelValThreY_;
  double slamFilterVelValThreZ_;
  double slamFilterVelValThreTheta_;
  double slamFilterVelValThrePhy_;
  double slamFilterVelValThrePsi_;
  double slamFilterVelChangeRateThreX_;
  double slamFilterVelChangeRateThreY_;
  double slamFilterVelChangeRateThreZ_;
  double slamFilterVelChangeRateThreTheta_;
  double slamFilterVelChangeRateThrePhy_;
  double slamFilterVelChangeRateThrePsi_;



  IirFilter filterX_;
  IirFilter filterY_;
  IirFilter filterZ_;
  IirFilter filterTheta_;
  IirFilter filterPhy_;
  IirFilter filterPsi_;


  void rosParamInit()
  {
    std::string ns = slamDataNodeHandlePrivate_.getNamespace();
    if (!slamDataNodeHandlePrivate_.getParam ("slamRxFreqX", slamRxFreqX_))
      slamRxFreqX_ = 0;
    printf("%s: slamRxFreqX_ is %.3f\n", ns.c_str(), slamRxFreqX_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamRxFreqY", slamRxFreqY_))
      slamRxFreqY_ = 0;
    printf("%s: slamRxFreqY_ is %.3f\n", ns.c_str(), slamRxFreqY_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamRxFreqZ", slamRxFreqZ_))
      slamRxFreqZ_ = 0;
    printf("%s: slamRxFreqZ_ is %.3f\n", ns.c_str(), slamRxFreqZ_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamRxFreqTheta", slamRxFreqTheta_))
      slamRxFreqTheta_ = 0;
    printf("%s: slamRxFreqTheta_ is %.3f\n", ns.c_str(), slamRxFreqTheta_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamRxFreqPhy", slamRxFreqPhy_))
      slamRxFreqPhy_ = 0;
    printf("%s: slamRxFreqPhy_ is %.3f\n", ns.c_str(), slamRxFreqPhy_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamRxFreqPsi", slamRxFreqPsi_))
      slamRxFreqPsi_ = 0;
    printf("%s: slamRxFreqPsi_ is %.3f\n", ns.c_str(), slamRxFreqPsi_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffPosFreqX", slamCutoffPosFreqX_))
      slamCutoffPosFreqX_ = 0;
    printf("%s: slamCutoffPosFreqX_ is %.3f\n", ns.c_str(), slamCutoffPosFreqX_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffPosFreqY", slamCutoffPosFreqY_))
      slamCutoffPosFreqY_ = 0;
    printf("%s: slamCutoffPosFreqY_ is %.3f\n", ns.c_str(), slamCutoffPosFreqY_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffPosFreqZ", slamCutoffPosFreqZ_))
      slamCutoffPosFreqZ_ = 0;
    printf("%s: slamCutoffPosFreqZ_ is %.3f\n", ns.c_str(), slamCutoffPosFreqZ_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffPosFreqTheta", slamCutoffPosFreqTheta_))
      slamCutoffPosFreqTheta_ = 0;
    printf("%s: slamCutoffPosFreqTheta_ is %.3f\n", ns.c_str(), slamCutoffPosFreqTheta_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffPosFreqPhy", slamCutoffPosFreqPhy_))
      slamCutoffPosFreqPhy_ = 0;
    printf("%s: slamCutoffPosFreqPhy_ is %.3f\n", ns.c_str(), slamCutoffPosFreqPhy_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffPosFreqPsi", slamCutoffPosFreqPsi_))
      slamCutoffPosFreqPsi_ = 0;
    printf("%s: slamCutoffPosFreqPsi_ is %.3f\n", ns.c_str(), slamCutoffPosFreqPsi_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffVelFreqX", slamCutoffVelFreqX_))
      slamCutoffVelFreqX_ = 0;
    printf("%s: slamCutoffVelFreqX_ is %.3f\n", ns.c_str(), slamCutoffVelFreqX_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffVelFreqY", slamCutoffVelFreqY_))
      slamCutoffVelFreqY_ = 0;
    printf("%s: slamCutoffVelFreqY_ is %.3f\n", ns.c_str(), slamCutoffVelFreqY_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffVelFreqZ", slamCutoffVelFreqZ_))
      slamCutoffVelFreqZ_ = 0;
    printf("%s: slamCutoffVelFreqZ_ is %.3f\n", ns.c_str(), slamCutoffVelFreqZ_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffVelFreqTheta", slamCutoffVelFreqTheta_))
      slamCutoffVelFreqTheta_ = 0;
    printf("%s: slamCutoffVelFreqTheta_ is %.3f\n", ns.c_str(), slamCutoffVelFreqTheta_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffVelFreqPhy", slamCutoffVelFreqPhy_))
      slamCutoffVelFreqPhy_ = 0;
    printf("%s: slamCutoffVelFreqPhy_ is %.3f\n", ns.c_str(), slamCutoffVelFreqPhy_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamCutoffVelFreqPsi", slamCutoffVelFreqPsi_))
      slamCutoffVelFreqPsi_ = 0;
    printf("%s: slamCutoffVelFreqPsi_ is %.3f\n", ns.c_str(), slamCutoffVelFreqPsi_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelValThreX", slamFilterVelValThreX_))
      slamFilterVelValThreX_ = 0;
    printf("%s: slamFilterVelValThreX_ is %.3f\n", ns.c_str(), slamFilterVelValThreX_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelValThreY", slamFilterVelValThreY_))
      slamFilterVelValThreY_ = 0;
    printf("%s: slamFilterVelValThreY_ is %.3f\n", ns.c_str(), slamFilterVelValThreY_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelValThreZ", slamFilterVelValThreZ_))
      slamFilterVelValThreZ_ = 0;
    printf("%s: slamFilterVelValThreZ_ is %.3f\n", ns.c_str(), slamFilterVelValThreZ_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelValThreTheta", slamFilterVelValThreTheta_))
      slamFilterVelValThreTheta_ = 0;
    printf("%s: slamFilterVelValThreTheta_ is %.3f\n", ns.c_str(), slamFilterVelValThreTheta_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelValThrePhy", slamFilterVelValThrePhy_))
      slamFilterVelValThrePhy_ = 0;
    printf("%s: slamFilterVelValThrePhy_ is %.3f\n", ns.c_str(), slamFilterVelValThrePhy_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelValThrePsi", slamFilterVelValThrePsi_))
      slamFilterVelValThrePsi_ = 0;
    printf("%s: slamFilterVelValThrePsi_ is %.3f\n", ns.c_str(), slamFilterVelValThrePsi_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelChangeRateThreX", slamFilterVelChangeRateThreX_))
      slamFilterVelChangeRateThreX_ = 0;
    printf("%s: slamFilterVelChangeRateThreX_ is %.3f\n", ns.c_str(), slamFilterVelChangeRateThreX_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelChangeRateThreY", slamFilterVelChangeRateThreY_))
      slamFilterVelChangeRateThreY_ = 0;
    printf("%s: slamFilterVelChangeRateThreY_ is %.3f\n", ns.c_str(), slamFilterVelChangeRateThreY_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelChangeRateThreZ", slamFilterVelChangeRateThreZ_))
      slamFilterVelChangeRateThreZ_ = 0;
    printf("%s: slamFilterVelChangeRateThreZ_ is %.3f\n", ns.c_str(), slamFilterVelChangeRateThreZ_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelChangeRateThreTheta", slamFilterVelChangeRateThreTheta_))
      slamFilterVelChangeRateThreTheta_ = 0;
    printf("%s: slamFilterVelChangeRateThreTheta_ is %.3f\n", ns.c_str(), slamFilterVelChangeRateThreTheta_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelChangeRateThrePhy", slamFilterVelChangeRateThrePhy_))
      slamFilterVelChangeRateThrePhy_ = 0;
    printf("%s: slamFilterVelChangeRateThrePhy_ is %.3f\n", ns.c_str(), slamFilterVelChangeRateThrePhy_);

    if (!slamDataNodeHandlePrivate_.getParam ("slamFilterVelChangeRateThrePsi", slamFilterVelChangeRateThrePsi_))
      slamFilterVelChangeRateThrePsi_ = 0;
    printf("%s: slamFilterVelChangeRateThrePsi_ is %.3f\n", ns.c_str(), slamFilterVelChangeRateThrePsi_);

  }

  void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg,
                           Estimator* state_estimator)
  {
    static float prev_raw_pos_x, prev_raw_pos_y, prev_raw_pos_psi;
    //static float prev_raw_laser_pos_x, prev_raw_laser_pos_y;
    static bool first_flag = true;    
    static double previous_secs;
    double current_secs = pose_msg->header.stamp.toSec();

    if(first_flag)
      {
        prev_raw_pos_x = 0; prev_raw_pos_y = 0; prev_raw_pos_psi = 0;
        first_flag = false;
       
        bool start_flag = true;
        kfbX_->setMeasureStartFlag(start_flag);
        kfbY_->setMeasureStartFlag(start_flag);
       
        kf1_->setMeasureStartFlag(start_flag);
        kf2_->setMeasureStartFlag(start_flag);
       
        kfX_->setMeasureStartFlag(start_flag);
        kfY_->setMeasureStartFlag(start_flag);
      }
    else
      {

        //**** 位置情報の更新
        //***** laser
#if 0 //laser frame case
        rawLaserPosX = pose_msg->pose.position.x; 
        rawLaserPosY = pose_msg->pose.position.y;    
        rawPsiSlam =  tf::getYaw(pose_msg->pose.orientation); 
        //***** base link( imu board)
        float thetaTmp = state_estimator->getStateTheta();
        float phyTmp   = state_estimator->getStatePhy();
        float distance = state_estimator->getLaserToImuDistance();
        // printf("theta is %f, phy is %f, distance is %f\n", state_estimator->getStateTheta(),
        //        state_estimator->getStatePhy(),state_estimator->getLaserToImuDistance());
        rawPosX = rawLaserPosX + sin(thetaTmp) * cos(phyTmp) * distance;
        rawPosY = rawLaserPosY + (-sin(phyTmp)) * distance;
        //**** 速度情報の更新
        //***** laser
        rawVelX = (rawPosX - prev_raw_pos_x) / (current_secs - previous_secs);
        rawVelY = (rawPosY - prev_raw_pos_y) / (current_secs - previous_secs);
        rawLaserVelX = (rawLaserPosX - prev_raw_laser_pos_x) / (current_secs - previous_secs);
        rawLaserVelY = (rawLaserPosY - prev_raw_laser_pos_y) / (current_secs - previous_secs);

#else //no laser frame case
#if 1 // plus frame
        rawPosX = pose_msg->pose.position.x; 
        rawPosY = pose_msg->pose.position.y;    
#else // x frame
        rawPosX = 0.7071 * (pose_msg->pose.position.x + pose_msg->pose.position.y);
        rawPosY = 0.7071 * (- pose_msg->pose.position.x + pose_msg->pose.position.y);
#endif
        rawPsiSlam =  tf::getYaw(pose_msg->pose.orientation); 
        rawVelX = (rawPosX - prev_raw_pos_x) / (current_secs - previous_secs);
        rawVelY = (rawPosY - prev_raw_pos_y) / (current_secs - previous_secs);
#endif



        //***** base link( imu board)

        if(rawPsiSlam - prev_raw_pos_psi > M_PI)
          rawVelPsiSlam = (- 2 * M_PI + rawPsiSlam - prev_raw_pos_psi)
            / (current_secs - previous_secs);
        else if(rawPsiSlam - prev_raw_pos_psi < -M_PI)
          rawVelPsiSlam = (2 * M_PI + rawPsiSlam - prev_raw_pos_psi)
            / (current_secs - previous_secs);
        else
          rawVelPsiSlam = (rawPsiSlam - prev_raw_pos_psi)
            / (current_secs - previous_secs);


        //IIR filter
        filterX_.filterFunction(rawPosX, posX, rawVelX, velX);
        filterY_.filterFunction(rawPosY, posY, rawVelY, velY);
        filterPsi_.filterFunction(rawPsiSlam, psiSlam, rawVelPsiSlam, velPsiSlam);

        if(kalmanFilterFlag)
          {

            kfX_->imuQuCorrection(pose_msg->header.stamp, rawPosX);
            kfY_->imuQuCorrection(pose_msg->header.stamp, rawPosY);
            kfbX_->imuQuCorrection(pose_msg->header.stamp, rawPosX);
            kfbY_->imuQuCorrection(pose_msg->header.stamp, rawPosY);

          }

        if(kalmanFilterDebug)
          {
            if(kalmanFilterAxis == 0) //X axis
              {
                kf1_->correction(rawPosX,pose_msg->header.stamp);
                kf2_->correction(rawPosX,pose_msg->header.stamp);

              }
            else if(kalmanFilterAxis == 1) //Y axis
              {
                kf1_->correction(rawPosY,pose_msg->header.stamp);
                kf2_->correction(rawPosY,pose_msg->header.stamp);

              }
            else if(kalmanFilterAxis == 3)
              {
                kf1_->imuQuCorrection(pose_msg->header.stamp, rawPosX);
                kf2_->correction(rawPosY,pose_msg->header.stamp);               
              }
          }

        jsk_quadcopter::SlamDebug slamDebug_;
        slamDebug_.header.stamp = pose_msg->header.stamp;
        slamDebug_.posX = posX;
        slamDebug_.emptyX1 = rawLaserPosX;
        slamDebug_.rawPosX = rawPosX;
        slamDebug_.velX = velX;
        slamDebug_.emptyX2 = rawLaserVelX;
        slamDebug_.rawVelX = rawVelX;

        if(kalmanFilterFlag)
          {
            slamDebug_.crrPosX1 = kfX_->getEstimatePos();
            slamDebug_.crrVelX1 = kfX_->getEstimateVel();
            slamDebug_.crrPosX2 = kfbX_->getEstimatePos();
            slamDebug_.crrVelX2 = kfbX_->getEstimateVel();
            slamDebug_.crrXBias = kfbX_->getEstimateBias();
            //slamDebug_.emptyX1  = xAxisDelay;
           
          }
        slamDebug_.rawVelX = rawVelX;
        slamDebug_.posY = posY;
        slamDebug_.emptyY1 = rawLaserPosY;
        slamDebug_.rawPosY = rawPosY;
        slamDebug_.velY = velY;
        slamDebug_.emptyY2 = rawLaserVelY;
        slamDebug_.rawVelY = rawVelY;

        if(kalmanFilterFlag)
          {
            slamDebug_.crrPosY1 = kfY_->getEstimatePos();
            slamDebug_.crrVelY1 = kfY_->getEstimateVel();
            slamDebug_.crrPosY2 = kfbY_->getEstimatePos();
            slamDebug_.crrVelY2 = kfbY_->getEstimateVel();
            slamDebug_.crrYBias = kfbY_->getEstimateBias();
            //slamDebug_.emptyY1  = yAxisDelay;
          }

        if(kalmanFilterDebug)
          {
            slamDebug_.crrPosDebug1 = kf1_->getEstimatePos();
            slamDebug_.crrVelDebug1 = kf1_->getEstimateVel();
            slamDebug_.crrPosDebug2 = kf2_->getEstimatePos();
            slamDebug_.crrVelDebug2 = kf2_->getEstimateVel();
            slamDebug_.posZ = kfZ_->getEstimatePos();
            slamDebug_.velZ = kfZ_->getEstimateVel();
          }

        slamDebug_.psi = psiSlam;
        slamDebug_.rawPsi = rawPsiSlam;
        slamDebug_.velPsi = velPsiSlam;
        slamDebug_.rawVelPsi = rawVelPsiSlam;
        slamPub_.publish(slamDebug_);
      }

    //更新
    previous_secs = current_secs;
    //prev_raw_laser_pos_x = rawLaserPosX;
    //prev_raw_laser_pos_y = rawLaserPosY;
    prev_raw_pos_x = rawPosX;
    prev_raw_pos_y = rawPosY;
    prev_raw_pos_psi =  tf::getYaw(pose_msg->pose.orientation);
  }

};

#endif






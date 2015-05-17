#ifndef OPTICAL_FLOW_MODULE_H
#define OPTICAL_FLOW_MODULE_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <jsk_quadcopter/state_estimation.h>
#include <jsk_quadcopter/OpticalFlowDebug.h>
#include <px_comm/OpticalFlow.h>
//* filter
#include <jsk_quadcopter/kalman_filter.h>
#include <jsk_quadcopter/digital_filter.h>


class OpticalFlowData
{
 public:

 OpticalFlowData(ros::NodeHandle nh,
                 ros::NodeHandle nh_private,
                 Estimator* state_estimator,
                 bool kalman_filter_flag,
                 KalmanFilterImuLaser *kf_x, 
                 KalmanFilterImuLaser *kf_y,
                 KalmanFilterImuLaser *kf_z,
                 KalmanFilterImuLaserBias *kfb_x, 
                 KalmanFilterImuLaserBias *kfb_y,
                 KalmanFilterImuLaserBias *kfb_z)
   : opticalFlowDataNodeHandle_(nh, "opticalFlow"),
    opticalFlowDataNodeHandlePrivate_(nh_private, "opticalFlow")
    {
      opticalFlowPub_ = opticalFlowDataNodeHandle_.advertise<jsk_quadcopter::OpticalFlowDebug>("debug", 10); 
      opticalFlowSub_ = opticalFlowDataNodeHandle_.subscribe<px_comm::OpticalFlow>("opt_flow", 5, boost::bind(&OpticalFlowData::opticalFlowCallback, this, _1, state_estimator));

      rosParamInit();

      kalmanFilterFlag = kalman_filter_flag;
      kfX_ = kf_x; kfY_ = kf_y; kfZ_ = kf_z;
      kfbX_ = kfb_x; kfbY_ = kfb_y; kfbZ_ = kfb_z;

      firFilterVelX_ = FirFilter(16);
      firFilterVelY_ = FirFilter(16);

      iirFilterVelX_ = IirFilter(97, iirFilterAccXCutoffHz_);
      iirFilterVelY_ = IirFilter(97, iirFilterAccYCutoffHz_);
      iirFilterPosZ_ = IirFilter(97, iirFilterAccZCutoffHz_);
      setRocketStartFlag();

      rawPosZ = 0;
      posZ = 0;
      rawVelZ = 0;
      velZ = 0;

      rawVelX = 0;
      velX = 0;
      posX = 0;
      rawVelY = 0;
      velY = 0;
      posY = 0;
    }

  ~OpticalFlowData()
    {
    }


  float getPosX()
  {
    return 0;
  }

  float getPosY()
  {
    return 0;
  }

  float getPosZ()
  {
    //default
    return rawPosZ;
  }

  void setVelX(float vel_x)
  {
    rawVelX= vel_x;
  }

  void setVelY(float vel_y)
  {
    rawVelY= vel_y;
  }

  void setVelZ(float vel_z)
  {
    rawVelZ= vel_z;
  }

  float getVelX()
  {
    return velX;
  }

  float getVelY()
  {
    return velY;
  }

  float getVelZ()
  {
    // default
    return rawVelZ;
  }

  void setRawVelX(float raw_vel_x)
  {
    rawVelX = raw_vel_x;
  }

  void setRawVelY(float raw_vel_y)
  {
    rawVelY = raw_vel_y;
  }

  void setRawVelZ(float raw_vel_z)
  {
    rawVelZ = raw_vel_z;
  }

  float getRawVelX()
  {
    return rawVelX;
  }

  float getRawVelY()
  {
    return rawVelY;
  }

  float getRawVelZ()
  {
    return rawVelZ;
  }

  bool  getRocketStartFlag()
  {
    return rocketStartFlag;
  }

  void  setRocketStartFlag()
  {
    if(useRocketStart_)
      {
        rocketStartFlag = true;
        bool stop_flag = false;
        kfX_->setMeasureStartFlag(stop_flag);
        kfY_->setMeasureStartFlag(stop_flag);
        kfZ_->setMeasureStartFlag(stop_flag);
      
        kfbX_->setMeasureStartFlag(stop_flag);
        kfbY_->setMeasureStartFlag(stop_flag);
        kfbZ_->setMeasureStartFlag(stop_flag);
        ROS_ERROR(" set measure start flag to false");
      }
    else
      {
        rocketStartFlag = false;
      }
  }


 private:
  ros::NodeHandle opticalFlowDataNodeHandle_;
  ros::NodeHandle opticalFlowDataNodeHandlePrivate_;
  ros::Publisher opticalFlowPub_;
  ros::Subscriber opticalFlowSub_;
  ros::Time opticalFlowStamp_;

  bool   useRocketStart_;
  bool   rocketStartFlag;
  double rocketStartUpperThre_;
  double rocketStartLowerThre_;
  double rocketStartVel_;

  double xAxisDirection_;  
  double yAxisDirection_;


  bool kalmanFilterFlag;
  KalmanFilterImuLaser *kfX_;
  KalmanFilterImuLaser *kfY_;
  KalmanFilterImuLaser *kfZ_;

  KalmanFilterImuLaserBias *kfbX_;
  KalmanFilterImuLaserBias *kfbY_;
  KalmanFilterImuLaserBias *kfbZ_;
  
  IirFilter iirFilterVelX_;
  IirFilter iirFilterVelY_;
  IirFilter iirFilterPosZ_;
  double iirFilterAccXCutoffHz_;
  double iirFilterAccYCutoffHz_;
  double iirFilterAccZCutoffHz_;

  FirFilter firFilterVelX_;
  FirFilter firFilterVelY_;

  float rawPosZ;
  float posZ;
  float rawVelZ;
  float velZ;

  float rawVelX;
  float velX;
  float posX;
  float rawVelY;
  float velY;
  float posY;

  void opticalFlowCallback(const px_comm::OpticalFlowConstPtr & optical_flow_msg,
                           Estimator* state_estimator)
  {
    static int cnt = 0;
    static int CNT = 1;
    static float prev_raw_pos_z;
    static bool first_flag = true;
    static double previous_secs;
    double current_secs = optical_flow_msg->header.stamp.toSec();

    //**** 高さ方向情報の更新
    rawPosZ = optical_flow_msg->ground_distance;

    // rocket start mode
    if( optical_flow_msg->ground_distance < rocketStartUpperThre_ &&
        optical_flow_msg->ground_distance > rocketStartLowerThre_ &&
	prev_raw_pos_z < rocketStartLowerThre_ &&
	prev_raw_pos_z > (rocketStartLowerThre_ - 0.1) &&
        rocketStartFlag)
      {//pose init
	//TODO: start flag fresh arm, or use air pressure => refined

	ROS_INFO("prev_raw_pos_z : %f", prev_raw_pos_z);
	ROS_ERROR("start rocket!");

        kfX_->setInitState(0, optical_flow_msg->flow_x /1000.0);
        kfbX_->setInitState(0, optical_flow_msg->flow_x /1000.0);

        kfY_->setInitState(0, -optical_flow_msg->flow_y /1000.0);
        kfbY_->setInitState(0, -optical_flow_msg->flow_y /1000.0);

        kfZ_->setInitState(optical_flow_msg->ground_distance, rocketStartVel_);
        kfbZ_->setInitState(optical_flow_msg->ground_distance, rocketStartVel_);

        bool start_flag = true;

        kfX_->setMeasureStartFlag(start_flag);
        kfY_->setMeasureStartFlag(start_flag);
        kfZ_->setMeasureStartFlag(start_flag);

        kfbX_->setMeasureStartFlag(start_flag);
        kfbY_->setMeasureStartFlag(start_flag);
        kfbZ_->setMeasureStartFlag(start_flag);

        rocketStartFlag = false;
      }


    if(first_flag)
        first_flag = false;
    else
      {
        if(!rocketStartFlag)
          {
            //**** 高さ方向情報の更新
            //rawPosZ = optical_flow_msg->ground_distance;
            rawVelZ = (rawPosZ - prev_raw_pos_z) / (current_secs - previous_secs);

            //**** 速度情報の更新,ボードの向き
            rawVelX = optical_flow_msg->velocity_x; 
            rawVelY = -optical_flow_msg->velocity_y; 

            velX = optical_flow_msg->flow_x /1000.0;
            velY = - optical_flow_msg->flow_y /1000.0;

            if(kalmanFilterFlag)
              {
                cnt++;
                if(cnt == CNT) //50 Hz !!!!!!!!!!!!!!!!!!!!!
                  {
                    cnt = 0; 
#if 0 // optical with accurate timestamp
                    if(optical_flow_msg->quality == 0 || rawVelX == 0 || rawPosZ > 2.5)
                      {// 2.5m is a good threhold
                        kfX_->imuQuCorrection(optical_flow_msg->header.stamp, velX,1);  //velocity
                        kfbX_->imuQuCorrection(optical_flow_msg->header.stamp, velX, 1); //velocity
                      }
                    else
                      {
                        kfX_->imuQuCorrection(optical_flow_msg->header.stamp, rawVelX,1);  //velocity
                        kfbX_->imuQuCorrection(optical_flow_msg->header.stamp, rawVelX, 1); //velocity
                      }
                    if(optical_flow_msg->quality == 0 || rawVelY == 0 || rawPosZ > 2.5)
                      {
                        kfY_->imuQuCorrection(optical_flow_msg->header.stamp, velY, 1); //velocity
                        kfbY_->imuQuCorrection(optical_flow_msg->header.stamp, velY, 1); //velocity                      
                      }
                    else
                      {
                        kfY_->imuQuCorrection(optical_flow_msg->header.stamp, rawVelY, 1); //velocity
                        kfbY_->imuQuCorrection(optical_flow_msg->header.stamp, rawVelY, 1); //velocity
                      }
      
#else // optical without accurate timestamp
                    if(optical_flow_msg->quality == 0 || rawVelX == 0 || rawPosZ > 2.5)
                      { // remove the rawVelX case is not good !!
                        kfX_->correctionOnlyVelocity(velX, optical_flow_msg->header.stamp);  //velocity
                        kfbX_->correctionOnlyVelocity(velX, optical_flow_msg->header.stamp); //velocity
                      }
                    else  
                      {
                        kfX_->correctionOnlyVelocity(rawVelX, optical_flow_msg->header.stamp);  //velocity
                        kfbX_->correctionOnlyVelocity(rawVelX, optical_flow_msg->header.stamp); //velocity
                      }
                    if(optical_flow_msg->quality == 0 || rawVelY == 0 || rawPosZ > 2.5)
                      //if(optical_flow_msg->quality == 0)
                      { // remove the rawVelY case is not good !!
                        kfY_->correctionOnlyVelocity(velY, optical_flow_msg->header.stamp); //velocity
                        kfbY_->correctionOnlyVelocity(velY, optical_flow_msg->header.stamp); //velocity
                      }
                    else
                      {
                        kfY_->correctionOnlyVelocity(rawVelY, optical_flow_msg->header.stamp); //velocity
                        kfbY_->correctionOnlyVelocity(rawVelY, optical_flow_msg->header.stamp); //velocity
                      }
#endif

                  }
                if(rawPosZ != prev_raw_pos_z && rawPosZ < 2.5) //100Hz
                  {
#if 0 //time stamp
                    kfZ_->imuQuCorrection(optical_flow_msg->header.stamp, rawPosZ);
                    kfbZ_->imuQuCorrection(optical_flow_msg->header.stamp, rawPosZ);
#else
                    kfbZ_->correction(rawPosZ, optical_flow_msg->header.stamp);
                    kfZ_->correction(rawPosZ, optical_flow_msg->header.stamp);
#endif
                  }
              }
            jsk_quadcopter::OpticalFlowDebug opticalFlowDebug_;
            opticalFlowDebug_.header.stamp = optical_flow_msg->header.stamp;
            opticalFlowDebug_.rawVelX = rawVelX;
            opticalFlowDebug_.velX = velX;
            opticalFlowDebug_.rawVelY = rawVelY;
            opticalFlowDebug_.velY = velY;
            opticalFlowDebug_.rawPosZ = rawPosZ;
            opticalFlowDebug_.rawVelZ = rawVelZ;

            if(kalmanFilterFlag)
              {
                opticalFlowDebug_.crrPosXNonBias = kfX_->getEstimatePos();
                opticalFlowDebug_.crrVelXNonBias = kfX_->getEstimateVel();
                opticalFlowDebug_.crrPosXBias    = kfbX_->getEstimatePos();
                opticalFlowDebug_.crrVelXBias    = kfbX_->getEstimateVel();
                opticalFlowDebug_.crrXBias       = kfbX_->getEstimateBias();

                opticalFlowDebug_.crrPosYNonBias = kfY_->getEstimatePos();
                opticalFlowDebug_.crrVelYNonBias = kfY_->getEstimateVel();
                opticalFlowDebug_.crrPosYBias    = kfbY_->getEstimatePos();
                opticalFlowDebug_.crrVelYBias    = kfbY_->getEstimateVel();
                opticalFlowDebug_.crrYBias       = kfbY_->getEstimateBias();

                opticalFlowDebug_.crrPosZNonBias = kfZ_->getEstimatePos();
                opticalFlowDebug_.crrVelZNonBias = kfZ_->getEstimateVel();
                opticalFlowDebug_.crrPosZBias    = kfbZ_->getEstimatePos();
                opticalFlowDebug_.crrVelZBias    = kfbZ_->getEstimateVel();
                opticalFlowDebug_.crrZBias       = kfbZ_->getEstimateBias();

              }
            opticalFlowPub_.publish(opticalFlowDebug_);
          }
      }

    //更新
    previous_secs = current_secs;
    prev_raw_pos_z = rawPosZ;
  }


  void rosParamInit()
  {
    //name space in /quadcopter/navigator/...
    ros::NodeHandle navigatorNodeHandle("~navigator");
    if (!navigatorNodeHandle.getParam ("useRocketStart", useRocketStart_))
      useRocketStart_ = false;
    printf("%s: useRocketStart is %s\n", navigatorNodeHandle.getNamespace().c_str(), useRocketStart_ ? ("true") : ("false"));
    if (!navigatorNodeHandle.getParam ("rocketStartUpperThre", rocketStartUpperThre_))
      rocketStartUpperThre_ = 0;
    printf("%s: rocketStartUpperThre_ is %.3f\n", navigatorNodeHandle.getNamespace().c_str(), rocketStartUpperThre_);
    if (!navigatorNodeHandle.getParam ("rocketStartLowerThre", rocketStartLowerThre_))
      rocketStartLowerThre_ = 0;
    printf("%s: rocketStartLowerThre_ is %.3f\n", navigatorNodeHandle.getNamespace().c_str(), rocketStartLowerThre_);
    if (!navigatorNodeHandle.getParam ("rocketStartVel", rocketStartVel_))
      rocketStartVel_ = 0;
    printf("%s: rocketStartVel_ is %.3f\n", navigatorNodeHandle.getNamespace().c_str(), rocketStartVel_);

    std::string ns = opticalFlowDataNodeHandlePrivate_.getNamespace();
    if (!opticalFlowDataNodeHandlePrivate_.getParam ("xAxisDirection", xAxisDirection_))
      xAxisDirection_ = 1.0;
    printf("%s: xAxisDirection_ is %.3f\n", ns.c_str(), xAxisDirection_);

    if (!opticalFlowDataNodeHandlePrivate_.getParam ("yAxisDirection", yAxisDirection_))
      yAxisDirection_ = 1.0;
    printf("%s: yAxisDirection_ is %.3f\n", ns.c_str(), yAxisDirection_);

    if (!opticalFlowDataNodeHandlePrivate_.getParam ("iirFilterAccXCutoffHz", iirFilterAccXCutoffHz_))
      iirFilterAccXCutoffHz_ = 0;
    printf("%s: iirFilterAccXCutoffHz_ is %.3f\n", ns.c_str(), iirFilterAccXCutoffHz_);

    if (!opticalFlowDataNodeHandlePrivate_.getParam ("iirFilterAccYCutoffHz", iirFilterAccYCutoffHz_))
      iirFilterAccYCutoffHz_ = 0;
    printf("%s: iirFilterAccYCutoffHz_ is %.3f\n", ns.c_str(), iirFilterAccYCutoffHz_);

    if (!opticalFlowDataNodeHandlePrivate_.getParam ("iirFilterAccZCutoffHz", iirFilterAccZCutoffHz_))
      iirFilterAccZCutoffHz_ = 0;
    printf("%s: iirFilterAccZCutoffHz_ is %.3f\n", ns.c_str(), iirFilterAccZCutoffHz_);
  }
};


#endif




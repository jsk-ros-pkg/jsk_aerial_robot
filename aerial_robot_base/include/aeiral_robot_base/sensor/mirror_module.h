#ifndef MIRROR_MODULE_H
#define MIRROR_MODULE_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <laser_geometry/laser_geometry.h>
#include <jsk_quadcopter/MirrorModuleDebug.h>

//* filter
#include <jsk_quadcopter/kalman_filter.h>
#include <jsk_quadcopter/digital_filter.h>


class MirrorModule
{
 public:
  MirrorModule(ros::NodeHandle nh,
               ros::NodeHandle nh_private,
               Estimator* state_estimator,
               bool kalman_filter_flag,
               bool kalman_filter_debug,
               KalmanFilterImuLaser *kf_z,
               KalmanFilterImuLaserBias *kfb_z)
    : mirrorModuleNodeHandle_(nh, "mirror"), 
    mirrorModuleNodeHandlePrivate_(nh_private, "mirror")
    {
      rosParamInit();
      stateEstimator_ = state_estimator;

      mirrorModulePub_ =
        mirrorModuleNodeHandle_.advertise<jsk_quadcopter::MirrorModuleDebug>("debug", 10);

      mirrorModuleLaserBoundaryPub_ =
        mirrorModuleNodeHandle_.advertise<std_msgs::Int32>("laser_boundary_offset", 1); 


      mirrorModuleLaserPointsPub_ =
        mirrorModuleNodeHandle_.advertise<std_msgs::Float32MultiArray>("laser_reflcted_points", 1); 


      mirrorModuleSub_ = 
        mirrorModuleNodeHandle_.subscribe("scan", 5, &MirrorModule::scanCallbackWithZCalc, this, ros::TransportHints().tcpNoDelay());
      zOffsetUpperLimit = mirrorZOffsetUpperLimit_;
      zOffsetLowerLimit = mirrorZOffsetLowerLimit_;
      filterZ_ =  IirFilter( (float)mirrorRxFreq_,
                             (float)mirrorCutoffPosFreq_,
                             (float)mirrorCutoffVelFreq_,
                             (float)mirrorFilterVelValThre_,
                             (float)mirrorFilterVelChangeRateThre_);

      kalmanFilterFlag = kalman_filter_flag;
      kalmanFilterDebug = kalman_filter_debug;
      kfZ_ = kf_z;
      kfbZ_ = kfb_z;

      scanStamp_ = ros::Time::now();
      posZMirrorOffset = 0; 
      laserBoundary = 0;
      laserReflected = 0;
    }
  
  ~MirrorModule()
    {
    }


  const static int Z_CALC_COUNT = 10;
  const static float Max_Diff_Height = 0.05; //gap is 5cm
  const static float LASER_TO_BASELINK = -0.18; // int the case of model of general quadcopter
  const static float MIRROR_MODULE_ARM_LENGTH = 0.114;


  void setPosZMirrorValue(float pos_z_value)
  {
    posZMirror = pos_z_value;
  }

  float getPosZMirrorValue()
  {
    return posZMirror;
  }

  void setRawPosZMirrorValue(float raw_pos_z_value)
  {
    rawPosZMirror = raw_pos_z_value;
  }

  float getRawPosZMirrorValue()
  {
    return rawPosZMirror;
  }

  void setVelZMirrorValue(float vel_z_value)
  {
    velZMirror = vel_z_value;
  }

  float getVelZMirrorValue()
  {
    return velZMirror;
  }

  void setRawVelZMirrorValue(float raw_vel_z_value)
  {
    rawVelZMirror = raw_vel_z_value;
  }

  float getRawVelZMirrorValue()
  {
    return rawVelZMirror;
  }

  void setPosZMirrorOffset(float pos_z_offset)
  {
    posZMirrorOffset = pos_z_offset;
  }

  float getPosZMirrorOffset()
  {
    return (float)posZMirrorOffset;
  }

  int  getLaserBoundary()
  {
    return laserBoundary;
  }

  int  getLaserReflected()
  {
    return laserReflected;
  }




  void  setScanStamp(ros::Time tm)
  {
    scanStamp_ = tm;
  }

  ros::Time getScanStamp()
    {
      return scanStamp_;
    }


  void scanCallbackWithZCalc(const sensor_msgs::LaserScan::ConstPtr & scan)
  {
    static float prev_raw_pos_z;
    static double previous_secs;
    double current_secs = scan->header.stamp.toSec();
    static bool first_time = true;
    setScanStamp(scan->header.stamp);

    static int calibrate_count = 0;
    int count1= 0;
    float raw_pos_z= 0;


    if(first_time)
      prev_raw_pos_z= 0;

    //キャリブレーションタイム
    if(calibrate_count < Z_CALC_COUNT && scan->ranges[0] != 0){
      ROS_INFO("CALIBRATING");
      while (1){
        count1++;
        if((scan->ranges[count1] - scan->ranges[count1-1]) > - Max_Diff_Height && 
           (scan->ranges[count1] - scan->ranges[count1-1]) < Max_Diff_Height && 
           count1 <= 10)  // set param count1 < (int)scan->ranges.size() previous
          {
            posZMirrorOffset += scan->ranges[count1-1];
            laserReflected ++;
          }
        else
          {
            break;
          }
      }
      calibrate_count++;

    }else if(calibrate_count == Z_CALC_COUNT){

      //+*+* calculate the offset
      posZMirrorOffset /= laserReflected;
      laserReflected /= calibrate_count;

      //+*+* calculate the laser boundary
      laserBoundary = 0;
      while (1){
        laserBoundary ++;
        if((scan->ranges[laserBoundary] - scan->ranges[laserBoundary-1]) > - Max_Diff_Height && 
           (scan->ranges[laserBoundary] - scan->ranges[laserBoundary-1]) < Max_Diff_Height && 
           laserBoundary < (int)scan->ranges.size())
          {
          }
        else  break;
      }


      if(posZMirrorOffset > zOffsetUpperLimit || 
         posZMirrorOffset < zOffsetLowerLimit)
        {
          ROS_WARN("Bad Calib : the height offset is %f", posZMirrorOffset);
          posZMirrorOffset = 0;
          laserBoundary = 0;
          laserReflected = 0;
          calibrate_count = 0;
        }
      else
        {
          ROS_INFO("CALIBRATION OVER. z offset : %f, laser boundary : %d, laser reflected : %d", 
                   posZMirrorOffset, laserBoundary, laserReflected);


          //set the offset
          stateEstimator_->setPosZOffset(posZMirrorOffset);

          std_msgs::Int32 msg;
          msg.data = laserBoundary * 2; //set the parameter
          mirrorModuleLaserBoundaryPub_.publish(msg);

          bool start_flag = true;
          kfbZ_->setMeasureStartFlag(start_flag);
          kfZ_->setMeasureStartFlag(start_flag);

          calibrate_count++;

        }
    }else if(calibrate_count >  Z_CALC_COUNT){

      std_msgs::Float32MultiArray reflected_points;

      for(int i =0; i< laserReflected; i++){
        raw_pos_z += scan->ranges[i];
        reflected_points.data.push_back(scan->ranges[i]);
      }

      raw_pos_z /= laserReflected;
      rawPosZMirror = raw_pos_z;

      reflected_points.data.push_back(raw_pos_z);

      if(!first_time)
        {
          rawVelZMirror = (rawPosZMirror - prev_raw_pos_z) /
            (current_secs - previous_secs);
           
          filterZ_.filterFunction(rawPosZMirror, posZMirror, 
                                  rawVelZMirror, velZMirror);


          if(kalmanFilterFlag)
            { //no rocket start mode
#if 1 //true
              kfbZ_->imuQuCorrection(scan->header.stamp, (double)(rawPosZMirror-posZMirrorOffset));
              kfZ_->imuQuCorrection(scan->header.stamp, (double)(rawPosZMirror-posZMirrorOffset));
#else //no time stamp
              kfbZ_->correction((double)(rawPosZMirror-posZMirrorOffset), scan->header.stamp);
              kfZ_->correction((double)(rawPosZMirror-posZMirrorOffset), scan->header.stamp);
#endif
            }
        }

      //filterZ_->filterFunction(rawPosZMirror, posZMirror, rawVelZMirror, velZMirror);
      jsk_quadcopter::MirrorModuleDebug mirrorModuleDebug_;
      mirrorModuleDebug_.header.stamp = scan->header.stamp;
      mirrorModuleDebug_.posZ = posZMirror; //debug
      //mirrorModuleDebug_.posZ = scan->ranges[0] - posZMirrorOffset;
      mirrorModuleDebug_.rawPosZ = rawPosZMirror - posZMirrorOffset;
      mirrorModuleDebug_.velZ = velZMirror;
      mirrorModuleDebug_.rawVelZ = rawVelZMirror;
      if(kalmanFilterFlag)
        {
          mirrorModuleDebug_.crrPosZ1 = kfZ_->getEstimatePos();
          mirrorModuleDebug_.crrVelZ1 = kfZ_->getEstimateVel();
          mirrorModuleDebug_.crrPosZ2 = kfbZ_->getEstimatePos();
          mirrorModuleDebug_.crrVelZ2 = kfbZ_->getEstimateVel();
          mirrorModuleDebug_.crrBias  = kfbZ_->getEstimateBias();
        }

       

      mirrorModulePub_.publish( mirrorModuleDebug_ );
      mirrorModuleLaserPointsPub_.publish(reflected_points);

      //debug 
      //printf("raw_pos_z is %f\n", raw_pos_z);

      // 更新
      first_time = false;
      previous_secs = current_secs;
      prev_raw_pos_z = rawPosZMirror; 
    }
  }

  bool kalmanFilterFlag;
  bool kalmanFilterDebug;
  KalmanFilterImuLaser *kfZ_;
  KalmanFilterImuLaserBias *kfbZ_;


 private:
  ros::NodeHandle mirrorModuleNodeHandle_;
  ros::NodeHandle mirrorModuleNodeHandlePrivate_;
  ros::Publisher  mirrorModulePub_;
  ros::Publisher  mirrorModuleLaserBoundaryPub_;
  ros::Publisher  mirrorModuleLaserPointsPub_;
  ros::Subscriber  mirrorModuleSub_;
  ros::Time scanStamp_;
  Estimator* stateEstimator_;


  double mirrorRxFreq_;
  double mirrorCutoffPosFreq_;
  double mirrorCutoffVelFreq_;
  double mirrorFilterVelValThre_;
  double mirrorFilterVelChangeRateThre_;
  double mirrorZOffsetUpperLimit_;
  double mirrorZOffsetLowerLimit_;

  double posZMirror;
  double rawPosZMirror;
  double velZMirror;
  double rawVelZMirror;

  double posZMirrorOffset;
  int   laserBoundary;
  int   laserReflected;
  double zOffsetUpperLimit;
  double zOffsetLowerLimit;

  IirFilter filterZ_;


  void rosParamInit()
  {
    std::string ns = mirrorModuleNodeHandlePrivate_.getNamespace();
    if (!mirrorModuleNodeHandlePrivate_.getParam ("mirrorRxFreq", mirrorRxFreq_))
      mirrorRxFreq_ = 0;
    printf("%s: mirrorRxFreq_ is %.3f\n", ns.c_str(), mirrorRxFreq_);

    if (!mirrorModuleNodeHandlePrivate_.getParam ("mirrorCutoffPosFreq", mirrorCutoffPosFreq_))
      mirrorCutoffPosFreq_ = 0;
    printf("%s: mirrorCutoffPosFreq_ is %.3f\n", ns.c_str(), mirrorCutoffPosFreq_);

    if (!mirrorModuleNodeHandlePrivate_.getParam ("mirrorCutoffVelFreq", mirrorCutoffVelFreq_))
      mirrorCutoffVelFreq_ = 0;
    printf("%s: mirrorCutoffVelFreq_ is %.3f\n", ns.c_str(), mirrorCutoffVelFreq_);

    if (!mirrorModuleNodeHandlePrivate_.getParam ("mirrorFilterVelValThre", mirrorFilterVelValThre_))
      mirrorFilterVelValThre_ = 0;
    printf("%s: mirrorFilterVelValThre_ is %.3f\n", ns.c_str(), mirrorFilterVelValThre_);

    if (!mirrorModuleNodeHandlePrivate_.getParam ("mirrorFilterVelChangeRateThre", mirrorFilterVelChangeRateThre_))
      mirrorFilterVelChangeRateThre_ = 0;
    printf("%s: mirrorFilterVelChangeRateThre_ is %.3f\n", ns.c_str(), mirrorFilterVelChangeRateThre_);

    if (!mirrorModuleNodeHandlePrivate_.getParam ("mirrorZOffsetUpperLimit", mirrorZOffsetUpperLimit_))
      mirrorZOffsetUpperLimit_ = 0;
    printf("%s: mirrorZOffsetUpperLimit_ is %.3f\n", ns.c_str(), mirrorZOffsetUpperLimit_);

    if (!mirrorModuleNodeHandlePrivate_.getParam ("mirrorZOffsetLowerLimit", mirrorZOffsetLowerLimit_))
      mirrorZOffsetLowerLimit_ = 0;
    printf("%s: mirrorZOffsetLowerLimit_ is %.3f\n", ns.c_str(), mirrorZOffsetLowerLimit_);

  }

};

#endif

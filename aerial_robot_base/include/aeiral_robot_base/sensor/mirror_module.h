#ifndef MIRROR_MODULE_H
#define MIRROR_MODULE_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <laser_geometry/laser_geometry.h>
#include <aerial_robot_base/MirrorModuleData.h>

//* filter
#include <aerial_robot_base/kalman_filter.h>
#include <aerial_robot_base/digital_filter.h>


class MirrorModule
{
 public:
  MirrorModule(ros::NodeHandle nh,
               ros::NodeHandle nh_private,
               Estimator* estimator,
               bool kalman_filter_flag,
               bool kalman_filter_debug,
               KalmanFilterImuLaser *kf_z,
               KalmanFilterImuLaserBias *kfb_z)
    : nh_(nh, "mirror"), nhp_(nh_private, "mirror")
    {
      rosParamInit(nhp_);
      estimator_ = estimator;

      module_pub_ = nh_.advertise<aerial_robot_base::MirrorModuleDebug>("data", 10);
      module_laser_boundary_pub_ = nh_.advertise<std_msgs::Int32>("laser_boundary_offset", 1); 
      module_laser_points_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("laser_reflcted_points", 1); 
      module_sub_ = 
        nh_.subscribe("scan", 5, &MirrorModule::scanCallback, this, ros::TransportHints().tcpNoDelay());

      lpf_z_ =  Iir_filter( (float)rx_freq_,
                             (float)cutoffPos_freq_,
                             (float)cutoffVel_freq_,
                             (float)lpf_vel_val_thre_,
                             (float)lpf_vel_change_rate_thre_);

      kalman_filter_flag_ = kalman_filter_flag;
      kalman_filter_debug_ = kalman_filter_debug;
      kf_z_ = kf_z;
      kfb_z_ = kfb_z;

      scan_stamp_ = ros::Time::now();
      pos_z_mirror_offset_ = 0; 
      laser_boundary_ = 0;
      laser_reflected_ = 0;
    }
  
  ~MirrorModule()
    {
    }


  const static int Z_CALC_COUNT = 10;

  inline void setPosZ(float pos_z_value) {  pos_z_ = pos_z_value; }
  inline float getPosZ()  {    return pos_z_;  }
  inline void setRawPosZ(float raw_pos_z_value)  {  raw_pos_z_ = raw_pos_z_value;  }
  inline float getRawPosZ()  {    return raw_pos_z_;  }
  inline void setVelZ(float vel_z_value)  {    vel_z_ = vel_z_value;  }
  inline float getVelZ()  {    return vel_z_;  }
  inline void setRawVelZ(float raw_vel_z_value)  { raw_vel_z_ = raw_vel_z_value;  }
  inline float getRawVelZ()  {    return raw_vel_z_;  }
  inline void setPosZMirrorOffset(float pos_z_offset)  {    pos_z_mirror_offset_ = pos_z_offset;  }
  inline float getPosZMirrorOffset()  {    return (float)pos_z_mirror_offset_;  }
  inline int  getLaserBoundary()  {    return laser_boundary_; }
  inline int  getLaserReflected() {   return laser_reflected_; }
  inline void  setScanStamp(ros::Time tm)  {    scan_stamp_ = tm;  }
  inline ros::Time getScanStamp()    {      return scan_stamp_;    }


  void scanCallback(const sensor_msgs::LaserScan::ConstPtr & scan)
  {
    static float prev_raw_pos_z;
    static double previous_secs;
    double current_secs = scan->header.stamp.toSec();
    static bool first_time = true;
    setScanStamp(scan->header.stamp);

    static int calibrate_count = 0;
    int count1= 0;
    float raw_pos_z= 0;

    if(first_time)  prev_raw_pos_z= 0;

    //キャリブレーションタイム
    if(calibrate_count < Z_CALC_COUNT && scan->ranges[0] != 0){
      ROS_INFO("CALIBRATING");
      while (1){
        count1++;
        if((scan->ranges[count1] - scan->ranges[count1-1]) > - max_diff_height_ && 
           (scan->ranges[count1] - scan->ranges[count1-1]) < max_diff_height_ && 
           count1 <= 10)  // set param count1 < (int)scan->ranges.size() previous
          {
            pos_z_mirror_offset_ += scan->ranges[count1-1];
            laser_reflected_ ++;
          }
        else
          {
            break;
          }
      }
      calibrate_count++;

    }else if(calibrate_count == Z_CALC_COUNT){

      //+*+* calculate the offset
      pos_z_mirror_offset_ /= laser_reflected_;
      laser_reflected_ /= calibrate_count;

      //+*+* calculate the laser boundary
      laser_boundary_ = 0;
      while (1){
        laser_boundary_ ++;
        if((scan->ranges[laser_boundary_] - scan->ranges[laser_boundary_-1]) > - Max_Diff_Height && 
           (scan->ranges[laser_boundary_] - scan->ranges[laser_boundary_-1]) < Max_Diff_Height && 
           laser_boundary_ < (int)scan->ranges.size())
          {
          }
        else  break;
      }


      if(pos_z_mirror_offset_ > z_offset_ipper_limit_ || 
         pos_z_mirror_offset_ < z_offset_lower_limit_)
        {
          ROS_WARN("Bad Calib : the height offset is %f", pos_z_mirror_offset_);
          pos_z_mirror_offset_ = 0;
          laser_boundary_ = 0;
          laser_reflected_ = 0;
          calibrate_count = 0;
        }
      else
        {
          ROS_INFO("CALIBRATION OVER. z offset : %f, laser boundary : %d, laser reflected : %d",  pos_z_mirror_offset_, laser_boundary_, laser_reflected_);

          //set the offset
          estimator_->setPosZOffset(pos_z_mirror_offset_);

          std_msgs::Int32 msg;
          msg.data = laser_boundary_ * 2; //set the parameter
          mirror_module_laser_boundary_pub_.publish(msg);

          bool start_flag = true;
          kfb_z_->setMeasureStartFlag(start_flag);
          kf_z_->setMeasureStartFlag(start_flag);

          calibrate_count++;

        }
    }else if(calibrate_count >  Z_CALC_COUNT){

      std_msgs::Float32MultiArray reflected_points;

      for(int i =0; i< laser_reflected_; i++){
        raw_pos_z += scan->ranges[i];
        reflected_points.data.push_back(scan->ranges[i]);
      }

      raw_pos_z /= laserReflected;
      raw_pos_z_ = raw_pos_z;

      if(!first_time)
        {
          raw_vel_z_ = (raw_pos_z_ - prev_raw_pos_z) /
            (current_secs - previous_secs);
           
          lpf_z_.filterFunction(raw_pos_z_, pos_z_, 
                                  raw_vel_z_, vel_z_);


          if(kalmanFilterFlag)
            { //no rocket start mode
#if 1 //true
              kfb_z_->imuQuCorrection(scan->header.stamp, (double)(raw_pos_z_ - pos_z_offset_));
              kf_z_->imuQuCorrection(scan->header.stamp, (double)(raw_pos_z_- pos_z_offset_));
#else //no time stamp
              kfb_z_->correction((double)(raw_pos_z_ - pos_z_offset_), scan->header.stamp);
              kf_z_->correction((double)(raw_pos_z_ - pos_z_offset_), scan->header.stamp);
#endif
            }
        }

      aerial_robot_base::MirrorModuleDebug mirror_data;
      mirror_data.header.stamp = scan->header.stamp;
      mirror_data.posZ = pos_z_; //debug
      mirror_data.rawPosZ = raw_pos_z_ - pos_z_offset_;
      mirror_data.velZ = vel_z_;
      mirror_data.rawVelZ = raw_vel_z_;

      if(kalmanFilterFlag)
        {
          mirror_data.crrPosZ1 = kf_z_->getEstimatePos();
          mirror_data.crrVelZ1 = kf_z_->getEstimateVel();
          mirror_data.crrPosZ2 = kfb_z_->getEstimatePos();
          mirror_data.crrVelZ2 = kfb_z_->getEstimateVel();
          mirror_data.crrBias  = kfb_z_->getEstimateBias();
        }

      mirror_module_pub_.publish( mirror_data );
      mirror_module_laser_points_pub_.publish(reflected_points);

      // 更新
      first_time = false;
      previous_secs = current_secs;
      prev_raw_pos_z = raw_pos_z_; 
    }
  }



 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_;
  ros::Publisher  mirror_module_pub_;
  ros::Publisher  mirror_module_laser_boundary_pub_;
  ros::Publisher  mirror_module_laser_points_pub_;
  ros::Subscriber  mirror_module_sub_;
  ros::Time scan_stamp_;
  Estimator* estimator_;


  KalmanFilterImuLaser *kf_z_;
  KalmanFilterImuLaserBias *kfb_z_;

  bool kalman_filter_flag_;
  bool kalman_filter_debug_;



  double rx_freq_;
  double cutoff_pos_freq_;
  double cutoff_vel_freq_;
  double lpf_vel_val_thre_;
  double lpf_vel_change_rate_thre_;
  double z_offset_upper_limit_;
  double z_offset_lower_limit_;

  double pos_z;
  double raw_pos_z;
  double vel_z;
  double raw_vel_z;

  double pos_z_mirror_offset;
  int   laser_boundary;
  int   laser_reflected;

  IirFilter lpf_z_;


  void rosParamInit(ros::NodeHandle nh)
  {
    std::string ns = nh.getNamespace();
    if (!nh.getParam ("mirrorRxFreq", rx_freq_))
      rx_freq_ = 0;
    printf("%s: mirrorRxFreq_ is %.3f\n", ns.c_str(), rx_freq_);

    if (!nh.getParam ("mirrorCutoffPosFreq", cutoff_pos_freq_))
      cutoff_pos_freq_ = 0;
    printf("%s: cutoff_pos_freq_ is %.3f\n", ns.c_str(), cutoff_pos_freq_);

    if (!nh.getParam ("mirrorutoffVelFreq", cutoff_vel_freq_))
      cutoff_vel_freq_ = 0;
    printf("%s: cutoff_vel_freq_ is %.3f\n", ns.c_str(), cutoff_vel_freq_);

    if (!nh.getParam ("filterVelValThre", lpf_vel_val_thre_))
      lpf_vel_val_thre_ = 0;
    printf("%s: lpf_vel_valThre_ is %.3f\n", ns.c_str(), lpf_vel_val_thre_);

    if (!nh.getParam ("filterVelChangeRateThre", lpf_vel_change_rateThre_))
      lpf_vel_change_rate_thre_ = 0;
    printf("%s: lpf_vel_change_rateThre_ is %.3f\n", ns.c_str(), lpf_vel_change_rateThre_);

    if (!nh.getParam ("zOffsetUpperLimit", z_offset_upper_limit_))
      z_offset_upper_limit_ = 0;
    printf("%s: z_offset_upper_limit_ is %.3f\n", ns.c_str(), z_offset_upper_limit_);

    if (!nh.getParam ("zOffsetLowerLimit", z_offset_lower_limit_))
      z_offset_lower_limit_ = 0;
    printf("%s: z_offset_lower_limit_ is %.3f\n", ns.c_str(), z_offset_lower_limit_);

    if (!nh.getParam ("maxDiffHeight", max_diff_height_))
      max_diff_height_ = 0.05;
    printf("%s: max_diff_height_ is %.3f\n", ns.c_str(), max_diff_height_);

  }

};

#endif

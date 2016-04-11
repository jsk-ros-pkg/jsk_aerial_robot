#ifndef MIRROR_MODULE_H
#define MIRROR_MODULE_H

#include <ros/ros.h>

#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <laser_geometry/laser_geometry.h>
#include <aerial_robot_base/States.h>


namespace sensor_plugin
{

  class Mirror :public sensor_base_plugin::SensorBase
  {
  public:
 
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator)
    {
      nh_ = ros::NodeHandle(nh, "mirror");
      nhp_ = ros::NodeHandle(nhp, "mirror");
      estimator_ = estimator;

      baseRosParamInit();
      rosParamInit();

      state_pub_ = nh_.advertise<aerial_robot_base::States>("data", 10);
      module_laser_boundary_pub_ = nh_.advertise<std_msgs::Int32>("laser_boundary_offset", 1); 
      module_laser_points_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("laser_reflcted_points", 1); 
      module_sub_ = 
        nh_.subscribe("scan", 5, &Mirror::scanCallback, this, ros::TransportHints().tcpNoDelay());

      lpf_z_ =  IirFilter( (float)rx_freq_,
                           (float)cutoff_pos_freq_,
                           (float)cutoff_vel_freq_,
                           (float)lpf_vel_val_thre_,
                           (float)lpf_vel_change_rate_thre_);


      scan_stamp_ = ros::Time::now();
      pos_z_mirror_offset_ = 0; 
      laser_boundary_ = 0;
      laser_reflected_ = 0;
    }

    ~Mirror() { }
    Mirror() { }

    const static int Z_CALC_COUNT = 10;

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
          if((scan->ranges[laser_boundary_] - scan->ranges[laser_boundary_-1]) > - max_diff_height_ && 
             (scan->ranges[laser_boundary_] - scan->ranges[laser_boundary_-1]) < max_diff_height_ && 
             laser_boundary_ < (int)scan->ranges.size())
            {
            }
          else  break;
        }


        if(pos_z_mirror_offset_ > z_offset_upper_limit_ || 
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

            std_msgs::Int32 msg;
            msg.data = laser_boundary_ * 2; //set the parameter
            module_laser_boundary_pub_.publish(msg);

            Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 

            if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
              {
                for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                  {
                    if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                      {
                        temp(0,0) = pos_noise_sigma_;
                        estimator_->getFuserEgomotion(i)->setMeasureSigma(temp);
                        estimator_->getFuserEgomotion(i)->setMeasureFlag();
                      }
                  }
              }

            if(estimate_mode_ & (1 << EXPERIMENT_MODE))
              {
                for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                  {
                    if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))
                      {
                        temp(0,0) = pos_noise_sigma_;
                        estimator_->getFuserExperiment(i)->setMeasureSigma(temp);
                        estimator_->getFuserExperiment(i)->setMeasureFlag();
                      }
                  }
              }

            calibrate_count++;

          }
      }else if(calibrate_count >  Z_CALC_COUNT){

        std_msgs::Float32MultiArray reflected_points;

        for(int i =0; i< laser_reflected_; i++){
          raw_pos_z += scan->ranges[i];
          reflected_points.data.push_back(scan->ranges[i]);
        }

        raw_pos_z_ /= laser_reflected_;
        raw_pos_z_ = raw_pos_z;

        aerial_robot_base::States state;
        state.header.stamp = scan->header.stamp;

        aerial_robot_base::State z_state;


        if(!first_time)
          {

            raw_vel_z_ = (raw_pos_z_ - prev_raw_pos_z) /
              (current_secs - previous_secs);

            lpf_z_.filterFunction(raw_pos_z_, pos_z_, raw_vel_z_, vel_z_);

            Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);
            temp(0, 0) = (double)(raw_pos_z_ - pos_z_mirror_offset_);
            Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1);
 
            if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
              {
                for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                  {
                    if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                      {
                        sigma_temp(0,0) = pos_noise_sigma_;
                        estimator_->getFuserEgomotion(i)->setMeasureSigma(sigma_temp);
                        estimator_->getFuserExperiment(i)->correction(temp);

                        if(estimator_->getFuserEgomotionName(i) == "kalman_filter/kf_pose_vel_acc")

                          {//temporary
                            Eigen::Matrix<double,2,1> kf_z_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                            estimator_->setEEState(BasicEstimator::Z_W, 0, kf_z_state(0,0));
                            estimator_->setEEState(BasicEstimator::Z_W, 1, kf_z_state(1,0));
                            z_state.kf_pos = kf_z_state(0, 0);
                            z_state.kf_vel = kf_z_state(1, 0);
                          }
                        if(estimator_->getFuserEgomotionName(i) == "kalman_filter/kf_pose_vel_acc_bias") 
                          {//temporary
                            Eigen::Matrix<double,3,1> kfb_z_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                            z_state.kfb_pos = kfb_z_state(0, 0);
                            z_state.kfb_vel = kfb_z_state(1, 0);
                            z_state.kfb_bias= kfb_z_state(2, 0);
                          }
                      }
                  }
              }

            if(estimate_mode_ & (1 << EXPERIMENT_MODE))
              {
                for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                  {
                    if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))
                      {
                        sigma_temp(0,0) = pos_noise_sigma_;
                        estimator_->getFuserExperiment(i)->setMeasureSigma(sigma_temp);
                        estimator_->getFuserExperiment(i)->correction(temp);

                        if(estimator_->getFuserExperimentName(i) == "kalman_filter/kf_pose_vel_acc")
                          {//temporary
                            Eigen::Matrix<double,2,1> kf_z_state = estimator_->getFuserExperiment(i)->getEstimateState();
                            estimator_->setEXState(BasicEstimator::Z_W, 0, kf_z_state(0,0));
                            estimator_->setEXState(BasicEstimator::Z_W, 1, kf_z_state(1,0));
                            z_state.reserves.push_back(kf_z_state(0, 0));
                            z_state.reserves.push_back(kf_z_state(1, 0));
                          }
                        if(estimator_->getFuserExperimentName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                          {//temporary
                            Eigen::Matrix<double,3,1> kfb_z_state = estimator_->getFuserExperiment(i)->getEstimateState();
                            z_state.reserves.push_back(kfb_z_state(0, 0));
                            z_state.reserves.push_back(kfb_z_state(1, 0));
                            z_state.reserves.push_back(kfb_z_state(2, 0));
                          }
                      }
                  }
              }

          }

        z_state.id = "z";
        z_state.pos = pos_z_  - pos_z_mirror_offset_ ; //debug
        z_state.raw_pos = raw_pos_z_ - pos_z_mirror_offset_;
        z_state.vel = vel_z_;
        z_state.raw_vel = raw_vel_z_;

        state.states.push_back(z_state);
        state_pub_.publish( state );
        module_laser_points_pub_.publish(reflected_points);

        // 更新
        first_time = false;
        previous_secs = current_secs;
        prev_raw_pos_z = raw_pos_z_; 
      }
    }



  private:
    ros::Publisher  state_pub_;
    ros::Publisher  module_laser_boundary_pub_;
    ros::Publisher  module_laser_points_pub_;
    ros::Subscriber module_sub_;
    ros::Time scan_stamp_;
    BasicEstimator* estimator_;

    double rx_freq_;
    double cutoff_pos_freq_;
    double cutoff_vel_freq_;
    double lpf_vel_val_thre_;
    double lpf_vel_change_rate_thre_;
    double z_offset_upper_limit_;
    double z_offset_lower_limit_;

    double pos_noise_sigma_;
    double pos_z_;
    double raw_pos_z_;
    double vel_z_;
    double raw_vel_z_;

    double pos_z_mirror_offset_;
    int   laser_boundary_;
    int   laser_reflected_;

    double max_diff_height_;

    IirFilter lpf_z_;



    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();

      nhp_.param("pos_noise_sigma", pos_noise_sigma_, 0.001 );
      printf("pos noise sigma  is %f\n", pos_noise_sigma_);

      if (!nhp_.getParam ("rx_freq", rx_freq_))
        rx_freq_ = 0;
      printf("%s: rx_freq_ is %.3f\n", ns.c_str(), rx_freq_);

      if (!nhp_.getParam ("cutoff_pos_freq", cutoff_pos_freq_))
        cutoff_pos_freq_ = 0;
      printf("%s: cutoff_pos_freq_ is %.3f\n", ns.c_str(), cutoff_pos_freq_);

      if (!nhp_.getParam ("cutoff_vel_freq", cutoff_vel_freq_))
        cutoff_vel_freq_ = 0;
      printf("%s: cutoff_vel_freq_ is %.3f\n", ns.c_str(), cutoff_vel_freq_);

      if (!nhp_.getParam ("lpf_vel_val_thre", lpf_vel_val_thre_))
        lpf_vel_val_thre_ = 0;
      printf("%s: lpf_vel_val_thre_ is %.3f\n", ns.c_str(), lpf_vel_val_thre_);

      if (!nhp_.getParam ("lpf_vel_change_rate_thre", lpf_vel_change_rate_thre_))
        lpf_vel_change_rate_thre_ = 0;
      printf("%s: lpf_vel_change_rate_thre_ is %.3f\n", ns.c_str(), lpf_vel_change_rate_thre_);

      if (!nhp_.getParam ("z_offset_upper_limit", z_offset_upper_limit_))
        z_offset_upper_limit_ = 0;
      printf("%s: z_offset_upper_limit_ is %.3f\n", ns.c_str(), z_offset_upper_limit_);

      if (!nhp_.getParam ("z_offset_lower_limit", z_offset_lower_limit_))
        z_offset_lower_limit_ = 0;
      printf("%s: z_offset_lower_limit_ is %.3f\n", ns.c_str(), z_offset_lower_limit_);

      if (!nhp_.getParam ("max_diff_height", max_diff_height_))
        max_diff_height_ = 0.05;
      printf("%s: max_diff_height_ is %.3f\n", ns.c_str(), max_diff_height_);

    }

  };
};

#endif

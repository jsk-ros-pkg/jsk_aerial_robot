#ifndef OPTICAL_FLOW_MODULE_H
#define OPTICAL_FLOW_MODULE_H

#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
//* filter
#include <kalman_filter/kf_pos_vel_acc.h>
#include <kalman_filter/digital_filter.h>

#include <aerial_robot_base/States.h>
#include <aerial_robot_base/OpticalFlow.h>
#include <std_msgs/Int32.h>

snamespace sensor_plugin
{
  class OpticalFLow :public sensor_base_plugin::SensorBase
  {
  public:
      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* state_estimator)
      {
        nh_ = ros::NodeHandle(nh, "optical_flow");
        nhp_ = ros::NodeHandle(nhp, "optical_flow");
        state_estimator_ = state_estimator;

        baseRosParamInit();
        rosParamInit();

        optical_flow_pub_ = nh_.advertise<aerial_robot_base::States>("data",10);
        optical_flow_sub_ = nh_.subscribe<aerial_robot_base::OpticalFlow>("opt_flow", 1, &OpticalFlowData::opticalFlowCallback, this, ros::TransportHints().tcpNoDelay());

        raw_pos_z_ = 0;
        pos_z_ = 0;
        raw_vel_z_ = 0;
        vel_z_ = 0;

        raw_vel_x_ = 0;
        filtered_vel_x_ = 0;

        raw_vel_y_ = 0;
        filtered_vel_y_ = 0;

        //debug, but can be constant;
        sensor_fusion_flag_ = false; 
        //ROS_ERROR("kalman_filter_flag_: %s", kalman_filter_flag_?("true"):("false"));
        }

    ~OpticalFlowData()
      {
      }

    void setSensorFusionFlag(bool flag)
    {
      kf_correct_flag_ = flag;

      if(kf_correct_flag_)
        {//the should init phase
          //reset!
          bool stop_flag = false;
          kf_x_->setMeasureFlag(stop_flag);
          kf_y_->setMeasureFlag(stop_flag);
          kf_z_->setMeasureFlag(stop_flag);

          kfb_x_->setMeasureFlag(stop_flag);
          kfb_y_->setMeasureFlag(stop_flag);
        }
      else
        {//debug
          //ROS_WARN("px4flow: kf correct flag is false");
        }
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher optical_flow_pub_;
    ros::Subscriber optical_flow_sub_;
    ros::Time optical_flow_stamp_;

    BasicEstimator* state_estimator_;

    double start_upper_thre_;
    double start_lower_thre_;
    double start_vel_;

    double x_axis_direction_;
    double y_axis_direction_;

    float raw_pos_z_;
    float pos_z_;
    float raw_vel_z_;
    float vel_z_;

    float raw_vel_x_;
    float filtered_vel_x_;

    float raw_vel_y_;
    float filtered_vel_y_;

    bool sensor_fusion_flag_;

    void opticalFlowCallback(const aerial_robot_base::OpticalFlowConstPtr & optical_flow_msg)
    {
      static int cnt = 0;
      static int CNT = 0;
      static float prev_raw_pos_z;
      static bool start_flag = false;
      static double previous_secs;
      static double special_increment = 0;
      double current_secs = optical_flow_msg->header.stamp.toSec();

      //**** Global Sensor Fusion Flag Check
      if(!estimator_->getSensorFusionFlag()) 
        {
          if((kf_z_->getEstimateState())[0] <= 0)
            {
              bool stop_flag = false;
              kf_x_->setMeasureFlag(stop_flag);
              kf_y_->setMeasureFlag(stop_flag);
              kf_z_->setMeasureFlag(stop_flag);

              kfb_x_->setMeasureFlag(stop_flag);
              kfb_y_->setMeasureFlag(stop_flag);

              Eigen::Matrix<double, 3, 1> init_state3 = Eigen::MatrixXd::Zero(3, 1); 

              //kf_x_->setInitState(0, 0);
              kfb_x_->setInitState(init_state3);

              //kf_y_->setInitState(0, 0);
              kfb_y_->setInitState(init_state3);

              special_increment += 0.005;
              Eigen::Matrix<double, 2, 1> init_state2 = Eigen::MatrixXd::Zero(2, 1); 
              init_state2(1,0) = special_increment;
              kf_z_->setInitState(init_state2);
            }
          return;
        }
      //**** 高さ方向情報の更新
      raw_pos_z_ = optical_flow_msg->ground_distance;

      //  start sensor fusion condition
      if(!sensor_fusion_flag_)
        {
          if(raw_pos_z_ < start_upper_thre_ && raw_pos_z_ > start_lower_thre_ &&
             prev_raw_pos_z < start_lower_thre_ && 
             prev_raw_pos_z > (start_lower_thre_ - 0.1) && !start_flag)
            {//pose init
              //TODO: start flag fresh arm, or use air pressure => refined
              ROS_ERROR("optical flow: start sensor fusion, prev_raw_pos_z : %f", prev_raw_pos_z);

              kfb_x_->setInitState(x_axis_direction_ * optical_flow_msg->flow_x /1000.0, 1);
              kfb_y_->setInitState(y_axis_direction_ * optical_flow_msg->flow_y /1000.0 ,1);

              Eigen::Matrix<double, 2, 1> init_state2 = Eigen::MatrixXd::Zero(2, 1); 
              init_state2(0,0) = optical_flow_msg->ground_distance;
              init_state2(1,0) = start_vel_;
              kf_z_->setInitState(init_state2);

              kf_z_->setMeasureFlag();

              kfb_x_->setMeasureFlag();
              kfb_y_->setMeasureFlag();

              sensor_fusion_flag_ = true;

              //special 
              special_increment = 0;
            }
        }
      else //special process for landing
        {
          if(prev_raw_pos_z < start_upper_thre_ &&
             prev_raw_pos_z > start_lower_thre_ &&
             raw_pos_z_ < start_lower_thre_ && raw_pos_z_ > (start_lower_thre_ - 0.1))
            {
              sensor_fusion_flag_ = false;
              ROS_ERROR("optical flow sensor: stop sensor fusion");
            }
        }

      if(start_flag)
        {

          //**** 高さ方向情報の更新
          raw_vel_z_ = (raw_pos_z_ - prev_raw_pos_z) / (current_secs - previous_secs);

          //**** 速度情報の更新,ボードの向き
          raw_vel_x_ = x_axis_direction_ * optical_flow_msg->velocity_x; 
          raw_vel_y_ = y_axis_direction_ * optical_flow_msg->velocity_y; 

          filtered_vel_x_ = x_axis_direction_ * optical_flow_msg->flow_x /1000.0;
          filtered_vel_y_ = y_axis_direction_ * optical_flow_msg->flow_y /1000.0;


          if(kalman_filter_flag_)
            {

              Eigen::MatrixXd temp(1,1); 

              // optical without accurate timestamp
              if(optical_flow_msg->quality == 0 || raw_vel_x_ == 0 || raw_pos_z_ > 2.5)
                { // remove the raw_vel_x case is not good !!
                  temp(0, 0) = filtered_vel_x_;
                  //kf_x_->correction(temp);
                  kfb_x_->correction(temp); 
                }
              else  
                {
                  temp(0, 0) = raw_vel_x_;
                  //kf_x_->correction(temp);
                  kfb_x_->correction(temp); 
                }
              if(optical_flow_msg->quality == 0 || raw_vel_y_ == 0 || raw_pos_z_ > 2.5)
                { // remove the raw_vel_y case is not good !!
                  temp(0, 0) = filtered_vel_y_;
                  //kf_y_->correction(temp);
                  kfb_y_->correction(temp); 
                }
              else
                {
                  temp(0, 0) = raw_vel_y_;
                  //kf_y_->correction(temp);
                  kfb_y_->correction(temp); 
                }

              if(raw_pos_z_ != prev_raw_pos_z && raw_pos_z_ < 2.5) //100Hz
                {
                  temp(0, 0) = raw_pos_z_;
                  kf_z_->correction(temp);
                }
            }

          //publish
          aerial_robot_base::States opt_data;
          opt_data.header.stamp = optical_flow_msg->header.stamp;

          aerial_robot_base::State x_state;
          x_state.id = "x";
          x_state.raw_vel = raw_vel_x_;
          x_state.vel = filtered_vel_x_;

          aerial_robot_base::State y_state;
          y_state.id = "y";
          y_state.raw_vel = raw_vel_y_;
          y_state.vel = filtered_vel_y_;

          aerial_robot_base::State z_state;
          z_state.id = "z";
          z_state.raw_pos = raw_pos_z_;
          z_state.raw_vel = raw_vel_z_;

          if(kalman_filter_flag_)
            {
              Eigen::Matrix<double,2,1> kf_x_state = kf_x_->getEstimateState();
              Eigen::Matrix<double,2,1> kf_y_state = kf_y_->getEstimateState();
              Eigen::Matrix<double,2,1> kf_z_state = kf_z_->getEstimateState();
              Eigen::Matrix<double,3,1> kfb_x_state = kfb_x_->getEstimateState();
              Eigen::Matrix<double,3,1> kfb_y_state = kfb_y_->getEstimateState();

              x_state.kf_pos = kf_x_state(0, 0);
              x_state.kf_vel = kf_x_state(1, 0);
              x_state.kfb_pos = kfb_x_state(0, 0);
              x_state.kfb_vel = kfb_x_state(1, 0);
              x_state.kfb_bias = kfb_x_state(2, 0);

              y_state.kf_pos = kf_y_state(0, 0);
              y_state.kf_vel = kf_y_state(1, 0);
              y_state.kfb_pos = kfb_y_state(0, 0);
              y_state.kfb_vel = kfb_y_state(1, 0);
              y_state.kfb_bias = kfb_y_state(2, 0);

              z_state.kf_pos = kf_z_state(0, 0);
              z_state.kf_vel = kf_z_state(1, 0);
            }

          opt_data.states.push_back(x_state);
          opt_data.states.push_back(y_state);
          opt_data.states.push_back(z_state);

          optical_flow_pub_.publish(opt_data);

        }

      if(!start_flag && !kf_correct_flag_)
        {//special process
        }

      //更新
      previous_secs = current_secs;
      prev_raw_pos_z = raw_pos_z_;
    }

    void rosParamInit()
    {

      nh_.param("level_vel_noise_sigma", level_vel_noise_sigma_, 0.01 );
      printf("level vel noise sigma  is %f\n", level_vel_noise_sigma_);

      nh_.param("sonar_noise_sigma", sonar_noise_sigma_, 0.01 );
      printf("level vel noise sigma  is %f\n", sonar_noise_sigma_);

      if (!nhp_.getParam ("start_upper_thre", start_upper_thre_))
        start_upper_thre_ = 0;
      printf("%s: start_upper_thre_ is %.3f\n", nhp_.getNamespace().c_str(), start_upper_thre_);
      if (!nhp_.getParam ("start_lower_thre", start_lower_thre_))
        start_lower_thre_ = 0;
      printf("%s: start_lower_thre_ is %.3f\n", nhp_.getNamespace().c_str(), start_lower_thre_);
      if (!nhp_.getParam ("start_vel", start_vel_))
        start_vel_ = 0;
      printf("%s: start_vel_ is %.3f\n", nhp_.getNamespace().c_str(), start_vel_);

      std::string ns = nhp_.getNamespace();
      if (!nhp_.getParam ("x_axis_direction", x_axis_direction_))
        x_axis_direction_ = 1.0;
      printf("%s: x_axis_direction_ is %.3f\n", ns.c_str(), x_axis_direction_);

      if (!nhp_.getParam ("y_axis_direction", y_axis_direction_))
        y_axis_direction_ = -1.0; //-1 is default
      printf("%s: y_axisDirection_ is %.3f\n", ns.c_str(), y_axis_direction_);
    }
  };

};
#endif




#ifndef RTK_GPS_H
#define RTK_GPS_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>
#include <aerial_robot_base/States.h>
#include <nav_msgs/Odometry.h>

namespace sensor_plugin
{
  
  class RtkGps :public sensor_base_plugin::SensorBase
    {
    public:
      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, std::vector< boost::shared_ptr<sensor_base_plugin::SensorBase> > sensors, std::vector<std::string> sensor_names, int sensor_index)
      {
        baseParamInit(nh, nhp, estimator, sensor_names[sensor_index], sensor_index);
        rosParamInit();

        rtk_gps_pub_ = nh_.advertise<aerial_robot_base::States>("state", 10);
        rtk_gps_sub_ = nh_.subscribe<nav_msgs::Odometry>(rtk_gps_sub_name_, 5, &RtkGps::rtkFixCallback, this, ros::TransportHints().tcpNoDelay());

        pos_x_ = 0; pos_y_ = 0;
        prev_raw_pos_x_ = 0, prev_raw_pos_y_ = 0;
        raw_pos_x_ = 0; raw_pos_y_ = 0;
        vel_x_ = 0; vel_y_ = 0;
        raw_vel_x_ = 0;raw_vel_y_ = 0;

      }
      ~RtkGps() {}
      RtkGps() {}

    private:
      ros::Publisher rtk_gps_pub_;
      ros::Subscriber rtk_gps_sub_;

      std::string rtk_gps_sub_name_;

      aerial_robot_base::States rtk_gps_state_;
      aerial_robot_base::State x_state_, y_state_;

      double pos_x_;
      double pos_y_;
      double raw_pos_x_;
      double raw_pos_y_;
      double prev_raw_pos_x_;
      double prev_raw_pos_y_;
      double vel_x_;
      double vel_y_;
      double raw_vel_x_;
      double raw_vel_y_;

      double pos_noise_sigma_;

      void rosParamInit()
      {
        std::string ns = nhp_.getNamespace();

        nhp_.param("rtk_gps_sub_name", rtk_gps_sub_name_, std::string("/rtk_gps"));
	std::cout << "rtk gps sub_name is:" << rtk_gps_sub_name_ << std::endl;

        nhp_.param("pos_noise_sigma", pos_noise_sigma_, 0.001 );
        printf("pos noise sigma  is %f\n", pos_noise_sigma_);

      }

      void rtkFixCallback(const nav_msgs::OdometryConstPtr & rtk_gps_msg)
      {
        static bool first_flag = true;
        static double previous_secs;
        double current_secs = rtk_gps_msg->header.stamp.toSec();

        Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);

        if(first_flag)
          {
            prev_raw_pos_x_ = 0; prev_raw_pos_y_ = 0;
            first_flag = false;

            if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
              {
                for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                  {
                    if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W)))
                      {
                        temp(0,0) = pos_noise_sigma_;
                        estimator_->getFuserEgomotion(i)->setMeasureSigma(temp);
                        if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W))
                          estimator_->getFuserEgomotion(i)->setInitState(rtk_gps_msg->pose.pose.position.y, 0);
                        if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W))
                            estimator_->getFuserEgomotion(i)->setInitState(-rtk_gps_msg->pose.pose.position.x, 0);
                        estimator_->getFuserEgomotion(i)->setMeasureFlag();
                        estimate_indices_.push_back(i);
                      }
                  }
              }

            if(estimate_mode_ & (1 << EXPERIMENT_MODE))
              {
                for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                  {
                    if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W)))
                      {
                        temp(0,0) = pos_noise_sigma_;
                        estimator_->getFuserExperiment(i)->setMeasureSigma(temp);
                        if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W))
                          estimator_->getFuserExperiment(i)->setInitState(rtk_gps_msg->pose.pose.position.y, 0);
                        if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W))
                          estimator_->getFuserExperiment(i)->setInitState(-rtk_gps_msg->pose.pose.position.x, 0);

                        estimator_->getFuserExperiment(i)->setMeasureFlag();
                        experiment_indices_.push_back(i);
                      }
                  }
              }
          }
        else
          {
            //**** update 
            raw_pos_x_ = rtk_gps_msg->pose.pose.position.y;
            raw_pos_y_ = -rtk_gps_msg->pose.pose.position.x;
            raw_vel_x_ = rtk_gps_msg->twist.twist.linear.y;
            raw_vel_y_ = -rtk_gps_msg->twist.twist.linear.x;

            rtk_gps_state_.header.stamp = rtk_gps_msg->header.stamp;

            x_state_.id = "x";
            x_state_.pos = pos_x_;
            x_state_.raw_pos = raw_pos_x_;
            x_state_.vel = vel_x_;
            x_state_.raw_vel = raw_vel_x_;

            y_state_.id = "y";
            y_state_.pos = pos_y_;
            y_state_.raw_pos = raw_pos_y_;
            y_state_.vel = vel_y_;
            y_state_.raw_vel = raw_vel_y_;

            estimateProcess();

            rtk_gps_state_.states.resize(0);
            rtk_gps_state_.states.push_back(x_state_);
            rtk_gps_state_.states.push_back(y_state_);

            rtk_gps_pub_.publish(rtk_gps_state_);
          }

        //更新
        previous_secs = current_secs;
        prev_raw_pos_x_ = raw_pos_x_;
        prev_raw_pos_y_ = raw_pos_y_;
        updateHealthStamp(current_secs);
      }

      void estimateProcess()
      {
        if(!estimate_flag_) return;

        Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);
        Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1);

        for(vector<int>::iterator  it = estimate_indices_.begin(); it != estimate_indices_.end(); ++it )
          {
            if((estimator_->getFuserEgomotionId(*it) & (1 << BasicEstimator::X_W)))
              {
                sigma_temp(0,0) = pos_noise_sigma_;
                estimator_->getFuserEgomotion(*it)->setMeasureSigma(sigma_temp);

                temp(0, 0) = raw_pos_x_;
                estimator_->getFuserEgomotion(*it)->correction(temp);

                if(estimator_->getFuserEgomotionPluginName(*it) == "kalman_filter/kf_pose_vel_acc")
                  {//temporary
                    Eigen::MatrixXd state = estimator_->getFuserEgomotion(*it)->getEstimateState();
                    x_state_.kf_pos = state(0, 0);
                    x_state_.kf_vel = state(1, 0);
                  }
                if(estimator_->getFuserEgomotionPluginName(*it) == "kalman_filter/kf_pose_vel_acc_bias")
                  {//temporary
                    Eigen::MatrixXd state = estimator_->getFuserEgomotion(*it)->getEstimateState();
                    estimator_->setEEState(BasicEstimator::X_W, 0, state(0,0));
                    estimator_->setEEState(BasicEstimator::X_W, 1, state(1,0));
                    x_state_.kfb_pos = state(0, 0);
                    x_state_.kfb_vel = state(1, 0);
                    x_state_.kfb_bias = state(2, 0);
                  }
              }
            else if((estimator_->getFuserEgomotionId(*it) & (1 << BasicEstimator::Y_W)))
              {
                sigma_temp(0,0) = pos_noise_sigma_;
                estimator_->getFuserEgomotion(*it)->setMeasureSigma(sigma_temp);

                temp(0, 0) = raw_pos_y_;
                estimator_->getFuserEgomotion(*it)->correction(temp);

                if(estimator_->getFuserEgomotionPluginName(*it) == "kalman_filter/kf_pose_vel_acc")
                  {//temporary
                    Eigen::MatrixXd state = estimator_->getFuserEgomotion(*it)->getEstimateState();
                    y_state_.kf_pos = state(0, 0);
                    y_state_.kf_vel = state(1, 0);
                  }
                if(estimator_->getFuserEgomotionPluginName(*it) == "kalman_filter/kf_pose_vel_acc_bias")
                  {//temporary
                    Eigen::MatrixXd state = estimator_->getFuserEgomotion(*it)->getEstimateState();
                    estimator_->setEEState(BasicEstimator::Y_W, 0, state(0,0));
                    estimator_->setEEState(BasicEstimator::Y_W, 1, state(1,0));
                    y_state_.kfb_pos = state(0, 0);
                    y_state_.kfb_vel = state(1, 0);
                    y_state_.kfb_bias = state(2, 0);
                  }
              }
          }

        for(vector<int>::iterator  it = experiment_indices_.begin(); it != experiment_indices_.end(); ++it )
          {
            if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::X_W)))
              {
                sigma_temp(0,0) = pos_noise_sigma_;
                estimator_->getFuserExperiment(*it)->setMeasureSigma(sigma_temp);

                temp(0, 0) = raw_pos_x_;
                estimator_->getFuserExperiment(*it)->correction(temp);

                if(estimator_->getFuserExperimentPluginName(*it) == "kalman_filter/kf_pose_vel_acc")
                  {//temporary

                    Eigen::MatrixXd state = estimator_->getFuserExperiment(*it)->getEstimateState();
                    x_state_.reserves.push_back(state(0, 0));
                    x_state_.reserves.push_back(state(1, 0));
                  }
                if(estimator_->getFuserExperimentPluginName(*it) == "kalman_filter/kf_pose_vel_acc_bias")
                  {//temporary
                    Eigen::MatrixXd state = estimator_->getFuserExperiment(*it)->getEstimateState();
                    estimator_->setEXState(BasicEstimator::X_W, 0, state(0,0));
                    estimator_->setEXState(BasicEstimator::X_W, 1, state(1,0));
                    x_state_.reserves.push_back(state(0, 0));
                    x_state_.reserves.push_back(state(1, 0));
                    x_state_.reserves.push_back(state(2, 0));
                  }
              }
            else if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::Y_W)))
              {
                sigma_temp(0,0) = pos_noise_sigma_;
                estimator_->getFuserExperiment(*it)->setMeasureSigma(sigma_temp);

                temp(0, 0) = raw_pos_y_;
                estimator_->getFuserExperiment(*it)->correction(temp);

                if(estimator_->getFuserExperimentPluginName(*it) == "kalman_filter/kf_pose_vel_acc")
                  {//temporary
                    Eigen::MatrixXd state = estimator_->getFuserExperiment(*it)->getEstimateState();
                    y_state_.reserves.push_back(state(0, 0));
                    y_state_.reserves.push_back(state(1, 0));
                  }
                if(estimator_->getFuserExperimentPluginName(*it) == "kalman_filter/kf_pose_vel_acc_bias")
                  {//temporary
                    Eigen::MatrixXd state = estimator_->getFuserExperiment(*it)->getEstimateState();
                    estimator_->setEXState(BasicEstimator::Y_W, 0, state(0,0));
                    estimator_->setEXState(BasicEstimator::Y_W, 1, state(1,0));
                    y_state_.reserves.push_back(state(0, 0));
                    y_state_.reserves.push_back(state(1, 0));
                    y_state_.reserves.push_back(state(2, 0));
                  }
              }
          }
      }
    };
};
#endif






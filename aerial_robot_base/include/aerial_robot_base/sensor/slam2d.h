#ifndef LASER_2DSLAM_H
#define LASER_2DSLAM_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>

#include <std_msgs/Float32.h>
#include <kalman_filter/digital_filter.h>
#include <tf/transform_broadcaster.h>

#include <aerial_robot_base/States.h>
#include <geometry_msgs/PoseStamped.h>

namespace sensor_plugin
{
  
  class Slam2D :public sensor_base_plugin::SensorBase
    {

    public:
      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator)
      {
        nh_ = ros::NodeHandle(nh, "slam2d");
        nhp_ = ros::NodeHandle(nhp, "slam2d");
        estimator_ = estimator;

        baseRosParamInit();
        rosParamInit();

        slam_pub_ = nh_.advertise<aerial_robot_base::States>("state", 10);
        slam_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 5, &Slam2D::poseStampedCallback, this, ros::TransportHints().tcpNoDelay()); 
        cog_offset_sub_ = nh_.subscribe<std_msgs::Float32>(cog_rotate_sub_name_, 5, &Slam2D::cogOffsetCallback, this, ros::TransportHints().tcpNoDelay()); 


        pos_x_ = 0; pos_y_ = 0; psi_ = 0; 
        raw_pos_x_ = 0; raw_pos_y_ = 0; raw_psi_ = 0;
        vel_x_ = 0; vel_y_ = 0; vel_psi_ = 0; 
        raw_vel_x_ = 0;raw_vel_y_ = 0; raw_vel_psi_ = 0;

        cog_offset_angle_ = 0;

        filter_x_ =     IirFilter((float)rx_freq_x_, 
                                  (float)cutoff_pos_freq_x_, 
                                  (float)cutoff_vel_freq_x_);

        filter_y_ =     IirFilter((float)rx_freq_y_, 
                                  (float)cutoff_pos_freq_y_, 
                                  (float)cutoff_vel_freq_y_);

        filter_psi_ =     IirFilter((float)rx_freq_psi_, 
                                    (float)cutoff_pos_freq_psi_, 
                                    (float)cutoff_vel_freq_psi_);

      }
      ~Slam2D() {}
      Slam2D() {}

    private:
      ros::Publisher slam_pub_;
    ros::Subscriber cog_offset_sub_;
      ros::Subscriber slam_sub_;
      ros::Time stamp_;

    std::string cog_rotate_sub_name_;

      double pos_x_;
      double pos_y_;
      double psi_; //yaw angle
  
      double raw_pos_x_;
      double raw_pos_y_;
      double raw_psi_;

      double vel_x_;
      double vel_y_;
      double vel_psi_; 

      double raw_vel_x_;
      double raw_vel_y_;
      double raw_vel_psi_;

      double pos_noise_sigma_, angle_noise_sigma_;
      double cog_offset_angle_;

      double rx_freq_x_;
      double rx_freq_y_;
      double rx_freq_psi_;
      double cutoff_pos_freq_x_;
      double cutoff_pos_freq_y_;
      double cutoff_pos_freq_psi_;
      double cutoff_vel_freq_x_;
      double cutoff_vel_freq_y_;
      double cutoff_vel_freq_psi_;

      IirFilter filter_x_;
      IirFilter filter_y_;
      IirFilter filter_psi_;

      void rosParamInit()
      {
        std::string ns = nhp_.getNamespace();

        nhp_.param("cog_rotate_sub_name", cog_rotate_sub_name_, std::string("/cog_rotate"));

        nhp_.param("pos_noise_sigma", pos_noise_sigma_, 0.001 );
        printf("pos noise sigma  is %f\n", pos_noise_sigma_);

        nhp_.param("angle_noise_sigma", angle_noise_sigma_, 0.001 );
        printf("angle noise sigma  is %f\n", angle_noise_sigma_);

        if (!nhp_.getParam ("rx_freq_x", rx_freq_x_))
          rx_freq_x_ = 0;
        printf("%s: rx_freq_x_ is %.3f\n", ns.c_str(), rx_freq_x_);

        if (!nhp_.getParam ("rx_freq_y", rx_freq_y_))
          rx_freq_y_ = 0;
        printf("%s: rx_freq_y_ is %.3f\n", ns.c_str(), rx_freq_y_);

        if (!nhp_.getParam ("rx_freq_psi", rx_freq_psi_))
          rx_freq_psi_ = 0;
        printf("%s: rx_freq_psi_ is %.3f\n", ns.c_str(), rx_freq_psi_);

        if (!nhp_.getParam ("cutoff_pos_freq_x", cutoff_pos_freq_x_))
          cutoff_pos_freq_x_ = 0;
        printf("%s: cutoff_pos_freq_x_ is %.3f\n", ns.c_str(), cutoff_pos_freq_x_);

        if (!nhp_.getParam ("cutoff_pos_freq_y", cutoff_pos_freq_y_))
          cutoff_pos_freq_y_ = 0;
        printf("%s: cutoff_pos_freq_y_ is %.3f\n", ns.c_str(), cutoff_pos_freq_y_);

        if (!nhp_.getParam ("cutoff_pos_freq_psi", cutoff_pos_freq_psi_))
          cutoff_pos_freq_psi_ = 0;
        printf("%s: cutoff_pos_freq_psi_ is %.3f\n", ns.c_str(), cutoff_pos_freq_psi_);

        if (!nhp_.getParam ("cutoff_vel_freq_x", cutoff_vel_freq_x_))
          cutoff_vel_freq_x_ = 0;
        printf("%s: cutoff_vel_freq_x_ is %.3f\n", ns.c_str(), cutoff_vel_freq_x_);

        if (!nhp_.getParam ("cutoff_vel_freq_y", cutoff_vel_freq_y_))
          cutoff_vel_freq_y_ = 0;
        printf("%s: cutoff_vel_freq_y_ is %.3f\n", ns.c_str(), cutoff_vel_freq_y_);

        if (!nhp_.getParam ("cutoff_vel_freq_psi", cutoff_vel_freq_psi_))
          cutoff_vel_freq_psi_ = 0;
        printf("%s: cutoff_vel_freq_psi_ is %.3f\n", ns.c_str(), cutoff_vel_freq_psi_);
      }

      void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
      {
        static bool first_flag = true;    
        static float prev_raw_pos_x, prev_raw_pos_y, prev_raw_pos_psi;
        static double previous_secs;
        double current_secs = pose_msg->header.stamp.toSec();
        Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);
        Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1);

        if(first_flag)
          {
            prev_raw_pos_x = 0; prev_raw_pos_y = 0; prev_raw_pos_psi = 0;
            first_flag = false;
       

            if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
              {
                for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                  {
                    if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W)))
                      {
                        temp(0,0) = pos_noise_sigma_;
                        estimator_->getFuserEgomotion(i)->setMeasureSigma(temp);
                        estimator_->getFuserEgomotion(i)->setMeasureFlag();
                      }
                    if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::YAW_W_COG)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::YAW_W_B)))
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
                    if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W)))
                      {
                        temp(0,0) = pos_noise_sigma_;
                        estimator_->getFuserExperiment(i)->setMeasureSigma(temp);
                        estimator_->getFuserExperiment(i)->setMeasureFlag();
                      }
                    if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::YAW_W_COG)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::YAW_W_B)))
                      {
                        temp(0,0) = pos_noise_sigma_;
                        estimator_->getFuserExperiment(i)->setMeasureSigma(temp);
                        estimator_->getFuserExperiment(i)->setMeasureFlag();
                      }
                  }
              }
          }
        else
          {
            //**** 位置情報の更新
            raw_pos_x_ = pose_msg->pose.position.x; 
            raw_pos_y_ = pose_msg->pose.position.y;    
            raw_psi_ =  tf::getYaw(pose_msg->pose.orientation); 
            raw_vel_x_ = (raw_pos_x_ - prev_raw_pos_x) / (current_secs - previous_secs);
            raw_vel_y_ = (raw_pos_y_ - prev_raw_pos_y) / (current_secs - previous_secs);

            if(raw_psi_ - prev_raw_pos_psi > M_PI)
              raw_vel_psi_ = (- 2 * M_PI + raw_psi_ - prev_raw_pos_psi)
                / (current_secs - previous_secs);
            else if(raw_psi_ - prev_raw_pos_psi < -M_PI)
              raw_vel_psi_ = (2 * M_PI + raw_psi_ - prev_raw_pos_psi)
                / (current_secs - previous_secs);
            else
              raw_vel_psi_ = (raw_psi_ - prev_raw_pos_psi)
                / (current_secs - previous_secs);


            //IIR filter
            filter_x_.filterFunction(raw_pos_x_, pos_x_, raw_vel_x_, vel_x_);
            filter_y_.filterFunction(raw_pos_y_, pos_y_, raw_vel_y_, vel_y_);
            filter_psi_.filterFunction(raw_psi_, psi_, raw_vel_psi_, vel_psi_);

            aerial_robot_base::States three_axis_state;
            three_axis_state.header.stamp = pose_msg->header.stamp;

            aerial_robot_base::State x_state;
            x_state.id = "x";
            x_state.pos = pos_x_;
            x_state.raw_pos = raw_pos_x_;
            x_state.vel = vel_x_;
            x_state.raw_vel = raw_vel_x_;


            aerial_robot_base::State y_state;
            y_state.id = "y";
            y_state.pos = pos_y_;
            y_state.raw_pos = raw_pos_y_;
            y_state.vel = vel_y_;
            y_state.raw_vel = raw_vel_y_;

            aerial_robot_base::State yaw_state;
            yaw_state.id = "yaw";
            yaw_state.pos = psi_;
            yaw_state.raw_pos = raw_psi_;
            yaw_state.vel = vel_psi_;
            yaw_state.raw_vel = raw_vel_psi_;

            if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
            {
              for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                {
                  if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W)))
                    {
                      sigma_temp(0,0) = pos_noise_sigma_;
                      estimator_->getFuserEgomotion(i)->setMeasureSigma(sigma_temp);

                      temp(0, 0) = raw_pos_x_;
                      estimator_->getFuserEgomotion(i)->correction(temp);

                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          x_state.kf_pos = state(0, 0);
                          x_state.kf_vel = state(1, 0);
                        }
                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::X_W, 0, state(0,0));
                          estimator_->setEEState(BasicEstimator::X_W, 1, state(1,0));
                          x_state.kfb_pos = state(0, 0);
                          x_state.kfb_vel = state(1, 0);
                          x_state.kfb_bias = state(2, 0);
                        }
                    }
                  else if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W)))
                    {
                      sigma_temp(0,0) = pos_noise_sigma_;
                      estimator_->getFuserEgomotion(i)->setMeasureSigma(sigma_temp);

                      temp(0, 0) = raw_pos_y_;
                      estimator_->getFuserEgomotion(i)->correction(temp);

                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          y_state.kf_pos = state(0, 0);
                          y_state.kf_vel = state(1, 0);
                        }
                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::Y_W, 0, state(0,0));
                          estimator_->setEEState(BasicEstimator::Y_W, 1, state(1,0));
                          y_state.kfb_pos = state(0, 0);
                          y_state.kfb_vel = state(1, 0);
                          y_state.kfb_bias = state(2, 0);
                        }
                    }
                  else if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::YAW_W_COG)))
                    {
                      sigma_temp(0,0) = angle_noise_sigma_;
                      estimator_->getFuserEgomotion(i)->setMeasureSigma(sigma_temp);

                      temp(0, 0) = raw_psi_ + cog_offset_angle_;
                      estimator_->getFuserEgomotion(i)->correction(temp);

                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::YAW_W_COG, 0, state(0,0));
                          estimator_->setEEState(BasicEstimator::YAW_W_COG, 1, state(1,0));
                          yaw_state.kf_pos = state(0, 0);
                          yaw_state.kf_vel = state(1, 0);
                        }
                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          yaw_state.kfb_pos = state(0, 0);
                          yaw_state.kfb_vel = state(1, 0);
                          yaw_state.kfb_bias = state(2, 0);

                        }
                    }
                  else if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::YAW_W_B)))
                    {
                      sigma_temp(0,0) = angle_noise_sigma_;
                      estimator_->getFuserEgomotion(i)->setMeasureSigma(sigma_temp);

                      temp(0, 0) = raw_psi_;
                      estimator_->getFuserEgomotion(i)->correction(temp);

                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          estimator_->setEEState(BasicEstimator::YAW_W_B, 0, state(0,0));
                          estimator_->setEEState(BasicEstimator::YAW_W_B, 1, state(1,0));
                          yaw_state.reserves.push_back(state(0, 0));
                          yaw_state.reserves.push_back(state(1, 0));
                        }
                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserEgomotion(i)->getEstimateState();
                          yaw_state.reserves.push_back(state(0, 0));
                          yaw_state.reserves.push_back(state(1, 0));
                          yaw_state.reserves.push_back(state(2, 0));
                        }
                    }
                }
            }

            if(estimate_mode_ & (1 << EXPERIMENT_MODE))
            {
              for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                {
                  if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W)))
                    {
                      sigma_temp(0,0) = pos_noise_sigma_;
                      estimator_->getFuserExperiment(i)->setMeasureSigma(sigma_temp);

                      temp(0, 0) = raw_pos_x_;
                      estimator_->getFuserExperiment(i)->correction(temp);

                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserExperiment(i)->getEstimateState();
                          x_state.reserves.push_back(state(0, 0));
                          x_state.reserves.push_back(state(1, 0));
                        }
                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::X_W, 0, state(0,0));
                          estimator_->setEXState(BasicEstimator::X_W, 1, state(1,0));
                          x_state.reserves.push_back(state(0, 0));
                          x_state.reserves.push_back(state(1, 0));
                          x_state.reserves.push_back(state(2, 0));
                        }
                    }
                  else if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W)))
                    {
                      sigma_temp(0,0) = pos_noise_sigma_;
                      estimator_->getFuserExperiment(i)->setMeasureSigma(sigma_temp);

                      temp(0, 0) = raw_pos_y_;
                      estimator_->getFuserExperiment(i)->correction(temp);

                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserExperiment(i)->getEstimateState();
                          y_state.reserves.push_back(state(0, 0));
                          y_state.reserves.push_back(state(1, 0));
                        }
                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::Y_W, 0, state(0,0));
                          estimator_->setEXState(BasicEstimator::Y_W, 1, state(1,0));
                          y_state.reserves.push_back(state(0, 0));
                          y_state.reserves.push_back(state(1, 0));
                          y_state.reserves.push_back(state(2, 0));
                        }
                    }
                  else if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::YAW_W_COG)))
                    {
                      sigma_temp(0,0) = angle_noise_sigma_;
                      estimator_->getFuserExperiment(i)->setMeasureSigma(sigma_temp);

                      temp(0, 0) = raw_psi_ + cog_offset_angle_;
                      estimator_->getFuserExperiment(i)->correction(temp);

                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::YAW_W_COG, 0, state(0,0));
                          estimator_->setEXState(BasicEstimator::YAW_W_COG, 1, state(1,0));
                          yaw_state.reserves.push_back(state(0, 0));
                          yaw_state.reserves.push_back(state(1, 0));
                        }
                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserExperiment(i)->getEstimateState();
                          yaw_state.reserves.push_back(state(0, 0));
                          yaw_state.reserves.push_back(state(1, 0));
                          yaw_state.reserves.push_back(state(2, 0));
                        }
                    }
                  else if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::YAW_W_B)))
                    {
                      sigma_temp(0,0) = angle_noise_sigma_;
                      estimator_->getFuserExperiment(i)->setMeasureSigma(sigma_temp);

                      temp(0, 0) = raw_psi_;
                      estimator_->getFuserExperiment(i)->correction(temp);

                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserExperiment(i)->getEstimateState();
                          estimator_->setEXState(BasicEstimator::YAW_W_B, 0, state(0,0));
                          estimator_->setEXState(BasicEstimator::YAW_W_B, 1, state(1,0));
                          yaw_state.reserves.push_back(state(0, 0));
                          yaw_state.reserves.push_back(state(1, 0));
                        }
                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {//temporary
                          Eigen::MatrixXd state = estimator_->getFuserExperiment(i)->getEstimateState();
                          yaw_state.reserves.push_back(state(0, 0));
                          yaw_state.reserves.push_back(state(1, 0));
                          yaw_state.reserves.push_back(state(2, 0));
                        }
                    }
                }
            }


            three_axis_state.states.push_back(x_state);
            three_axis_state.states.push_back(y_state);
            three_axis_state.states.push_back(yaw_state);

            slam_pub_.publish(three_axis_state);
          }

        //更新
        previous_secs = current_secs;
        prev_raw_pos_x = raw_pos_x_;
        prev_raw_pos_y = raw_pos_y_;
        prev_raw_pos_psi =  tf::getYaw(pose_msg->pose.orientation);
      }

    void cogOffsetCallback(std_msgs::Float32 offset_msg)
    {
      cog_offset_angle_ =  offset_msg.data;
    }


    };
};
#endif






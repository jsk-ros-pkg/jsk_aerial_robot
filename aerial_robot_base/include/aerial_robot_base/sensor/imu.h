/*
  1.the acc processing is wrong, related to the attitude estimation method
*/

#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <ros/ros.h>

#include <aerial_robot_base/Acc.h>
#include <geometry_msgs/Vector3.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>
#include <aerial_robot_msgs/SimpleImu.h>
#include <aerial_robot_msgs/Imu.h>

namespace sensor_plugin
{
  class Imu  :public sensor_base_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, std::vector< boost::shared_ptr<sensor_base_plugin::SensorBase> > sensors, std::vector<std::string> sensor_names, int sensor_index)
    {
      baseParamInit(nh, nhp, estimator, sensor_names[sensor_index], sensor_index);
      rosParamInit();

      if(imu_board_ == D_BOARD)
        {
          imu_sub_ = nh_.subscribe<aerial_robot_msgs::Imu>(imu_topic_name_, 1, boost::bind(&Imu::ImuCallback, this, _1, false)); 
        }

      if(imu_board_ == KDUINO)
        {
          imu_sub_ = nh_.subscribe<aerial_robot_msgs::SimpleImu>(imu_topic_name_, 1, &Imu::kduinoImuCallback, this); 
        }

      if(has_sub_imu_board_)
        {
          sub_imu_sub_ = nh_.subscribe<aerial_robot_msgs::Imu>(sub_imu_topic_name_, 1, boost::bind(&Imu::ImuCallback, this, _1, true)); 
        }

      acc_pub_ = nh_.advertise<aerial_robot_base::Acc>("acc", 2); 

      acc_xb_ = 0, acc_yb_ = 0, acc_zb_ = 0;
      gyro_xb_ = 0, gyro_yb_ = 0, gyro_zb_ = 0;
      mag_xb_ = 0, mag_yb_ = 0, mag_zb_ = 0;
      acc_xi_ = 0;      acc_yi_ = 0;

      acc_xw_ = 0;      acc_yw_ = 0;      acc_zw_ = 0;
      acc_xw_non_bias_ = 0;      acc_yw_non_bias_ = 0;      acc_zw_non_bias_ = 0;

      acc_x_bias_ = 0;      acc_y_bias_ = 0;      acc_z_bias_ = 0;
      acc_xw_bias_ = 0;      acc_yw_bias_ = 0;      acc_zw_bias_ = 0;

      pitch_ = 0;      roll_ = 0;      yaw_ = 0;
      height_ = 0;

      calib_count_ = 100; // temporarily

    }

    ~Imu () { }
    Imu () { }

    const static uint8_t D_BOARD = 1;
    const static uint8_t KDUINO = 0;

    inline void setPitch(float pitch_value) {   pitch_ = pitch_value;  }
    inline void setRoll(float roll_value) {    roll_ = roll_value;  }
    inline void setYaw(float yaw_value)  {    yaw_ = yaw_value;  }
    inline void setZ(float z_value)  {    height_ = z_value;  }
    inline float getPitch()  {    return pitch_;  }
    inline float getRoll()  {    return roll_;  }
    inline float getYaw()  {    return yaw_;  }

    inline float getAccXb()  {    return acc_xb_;  }
    inline float getAccYb()  {    return acc_yb_;  }
    inline float getAccZb()  {    return acc_zb_;  }

    inline ros::Time getImuStamp(){return imu_stamp_;}

  private:
    ros::Publisher  acc_pub_;
    ros::Subscriber  imu_sub_, sub_imu_sub_;
    ros::Subscriber  imu_simple_sub_;

    double level_acc_noise_sigma_, z_acc_noise_sigma_, acc_bias_noise_sigma_;

    double g_value_;
    double acc_scale_;
    double gyro_scale_;
    double mag_scale_;

    float acc_xb_, acc_yb_, acc_zb_;
    float gyro_xb_, gyro_yb_, gyro_zb_;
    float mag_xb_, mag_yb_, mag_zb_;

    float pitch_;  //pitch angle
    float roll_;    //roll angle
    float yaw_;    //yaw angle

    //*** trans_acc with intermediate frame between world frame and board frame
    float acc_xi_, acc_yi_;

    int yaw_acc_trans_for_experiment_estimation_;

    double landing_shock_force_;

    //***  world frame
    float acc_xw_, acc_xw_non_bias_;
    float acc_yw_, acc_yw_non_bias_;
    float acc_zw_, acc_zw_non_bias_;

    double acc_x_bias_;
    double acc_y_bias_;
    double acc_z_bias_;

    double acc_xw_bias_;
    double acc_yw_bias_;
    double acc_zw_bias_;

    std::string imu_topic_name_;
    int imu_board_;

    bool has_sub_imu_board_;  //whether there is a sub imu board for mag estimation
    std::string sub_imu_topic_name_;

    ros::Time imu_stamp_;

    float height_;
    float v_bat_;   //*** battery

    int calib_count_;
    double calib_time_;

    void ImuCallback(const aerial_robot_msgs::ImuConstPtr& imu_msg, bool sub_imu_board)
    {
      if(sub_imu_board) //use sub imu board for yaw estimation
        {
          yaw_   = imu_msg->angles[2];
          return;
        }

      imu_stamp_ = imu_msg->stamp;
      estimator_->setSystemTimeStamp(imu_stamp_);

      roll_  = imu_msg->angles[0];
      pitch_ = imu_msg->angles[1];
      if(!has_sub_imu_board_) yaw_   = imu_msg->angles[2];

      acc_xb_ = imu_msg->acc_data[0];
      acc_yb_ = imu_msg->acc_data[1];
      acc_zb_ = imu_msg->acc_data[2];
      gyro_xb_ = imu_msg->gyro_data[0];
      gyro_yb_ = imu_msg->gyro_data[1];
      gyro_zb_ = imu_msg->gyro_data[2];

      mag_xb_ = imu_msg->mag_data[0];
      mag_yb_ = imu_msg->mag_data[1];
      mag_zb_ = imu_msg->mag_data[2];

      imuDataConverter(imu_stamp_);
      updateHealthStamp(imu_msg->stamp.toSec());
    }

    void kduinoImuCallback(const aerial_robot_msgs::SimpleImuConstPtr& imu_msg)
    {
      imu_stamp_ = imu_msg->stamp;
      estimator_->setSystemTimeStamp(imu_stamp_);

      roll_  = M_PI * imu_msg->angle[0] / 10.0 / 180.0; //raw data is 10 times
      pitch_ = M_PI * imu_msg->angle[1] / 10.0 / 180.0; //raw data is 10 times
      yaw_   = M_PI * imu_msg->angle[2] / 180.0;

      acc_xb_ = imu_msg->accData[0] * acc_scale_;
      acc_yb_ = imu_msg->accData[1] * acc_scale_;
      acc_zb_ = imu_msg->accData[2] * acc_scale_;

      imuDataConverter(imu_stamp_);
    }

    void imuDataConverter(ros::Time stamp)
    {
      static int bias_calib = 0;
      static ros::Time prev_time;
      static double sensor_hz_ = 0;
      //* calculate accTran
#if 1 // use x,y for factor4 and z for factor3
      acc_xi_ = (acc_xb_) * cos(pitch_) + 
        (acc_yb_) * sin(pitch_) * sin(roll_) + 
        (acc_zb_) * sin(pitch_) * cos(roll_);
      acc_yi_ = (acc_yb_) * cos(roll_) - (acc_zb_) * sin(roll_);
      acc_zw_ = (acc_xb_) * (-sin(pitch_)) + 
        (acc_yb_) * cos(pitch_) * sin(roll_) + 
        (acc_zb_) * cos(pitch_) * cos(roll_) + (-g_value_);
      acc_zw_ = (acc_zb_) * cos(pitch_) * cos(roll_) + (- g_value_);
#else  // use approximation
      acc_xi_ =  (acc_zb_) * sin(pitch_) * cos(roll_);
      acc_yi_ =  - (acc_zb_) * sin(roll_);
      acc_zw_ = (acc_zb_) * cos(pitch_) * cos(roll_) + (- g_value_);
#endif
      if(estimator_->getLandingMode() && acc_zw_ > landing_shock_force_)
        {
          if(!estimator_->getLandedFlag())
            {
              ROS_WARN("imu: touch to ground");
              estimator_->setLandedFlag(true);
            }
        }

      /* check whether use imu yaw for control and estimation */
      if(estimator_->onlyImuYaw())
        {
          /* TODO: input to kalman filter */
          estimator_->setEEState(BasicEstimator::YAW_W_COG, 0, yaw_);
          estimator_->setEEState(BasicEstimator::YAW_W_B, 0, yaw_);
        }

      //bais calibration
      if(bias_calib < calib_count_)
        {
          bias_calib ++;

          if(bias_calib == 1)
            prev_time = imu_stamp_;

          double time_interval = imu_stamp_.toSec() - prev_time.toSec();
          if(bias_calib == 2)
            {
              calib_count_ = calib_time_ / time_interval;
              ROS_WARN("calib count is %d, time interval is %f", calib_count_, time_interval);

              /* check whether use imu yaw for contorl and estimation*/
              if(estimator_->onlyImuYaw()) ROS_WARN("use imu mag-based yaw value only for estimation and control");

            }

          //hz estimation
          sensor_hz_ += time_interval;

          //acc bias
          acc_x_bias_ += acc_xi_;
          acc_y_bias_ += acc_yi_;
          acc_z_bias_ += acc_zw_;

          if(bias_calib == calib_count_)
            {
              acc_x_bias_ /= calib_count_;
              acc_y_bias_ /= calib_count_;
              acc_z_bias_ /= calib_count_;

              sensor_hz_ /= (calib_count_ - 1 );

              ROS_WARN("accX bias is %f, accY bias is %f, accZ bias is %f, hz is %f", acc_x_bias_, acc_y_bias_, acc_z_bias_, sensor_hz_);
              if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
                {
                  for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                    {
                      estimator_->getFuserEgomotion(i)->updateModelFromDt(sensor_hz_);

                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {
                          Eigen::Matrix<double, 2, 1> temp = Eigen::MatrixXd::Zero(2, 1); 
                          temp(1,0) = acc_bias_noise_sigma_;
                          if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_B))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              estimator_->getFuserEgomotion(i)->setInitState(acc_x_bias_, 2);
                            }
                          else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              float yaw = estimator_->getEEState(BasicEstimator::YAW_W_B, 0);
                              float acc_xw_bias = cos(yaw) * acc_x_bias_ - sin(yaw) * acc_y_bias_;
                              estimator_->getFuserEgomotion(i)->setInitState(acc_xw_bias_, 2);
                            }
                          else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_B))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              estimator_->getFuserEgomotion(i)->setInitState(acc_y_bias_, 2);
                            }
                          else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              float yaw = estimator_->getEEState(BasicEstimator::YAW_W_B, 0);
                              float acc_yw_bias = sin(yaw) * acc_x_bias_ + cos(yaw) * acc_y_bias_;
                              estimator_->getFuserEgomotion(i)->setInitState(acc_yw_bias_, 2);
                            }
                          else if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                            {
                              temp(0,0) = z_acc_noise_sigma_;
                              estimator_->getFuserEgomotion(i)->setInitState(acc_z_bias_, 2);
                            }
                          estimator_->getFuserEgomotion(i)->setInputSigma(temp);
                        }

                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {//temporary
                          Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
                          if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_B))
                             || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_B)))
                            temp(0,0) = level_acc_noise_sigma_;
                          else if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                            temp(0,0) = z_acc_noise_sigma_;
                          estimator_->getFuserEgomotion(i)->setInputSigma(temp);
                        }
                      estimator_->getFuserEgomotion(i)->setInputFlag();
                    }
                }

              if(estimate_mode_ & (1 << EXPERIMENT_MODE))
                {
                  for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                    {
                      estimator_->getFuserExperiment(i)->updateModelFromDt(sensor_hz_);

                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {//temporary
                          Eigen::Matrix<double, 2, 1> temp = Eigen::MatrixXd::Zero(2, 1);
                          temp(1,0) = acc_bias_noise_sigma_;
                          if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_B))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              estimator_->getFuserExperiment(i)->setInitState(acc_x_bias_, 2);
                            }
                          else if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              float yaw = estimator_->getEXState(BasicEstimator::YAW_W_B, 0);
                              acc_xw_bias_ = cos(yaw) * acc_x_bias_ - sin(yaw) * acc_y_bias_;
                              estimator_->getFuserExperiment(i)->setInitState(acc_xw_bias_, 2);
                            }
                          else if (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_B))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              estimator_->getFuserExperiment(i)->setInitState(acc_y_bias_, 2);
                            }
                          else if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              float yaw = estimator_->getEXState(BasicEstimator::YAW_W_B, 0);
                              acc_yw_bias_ = sin(yaw) * acc_x_bias_ + cos(yaw) * acc_y_bias_;
                              estimator_->getFuserExperiment(i)->setInitState(acc_yw_bias_, 2);
                            }
                          else if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))
                            {
                              temp(0,0) = z_acc_noise_sigma_;
                              estimator_->getFuserExperiment(i)->setInitState(acc_z_bias_, 2);
                            }
                          estimator_->getFuserExperiment(i)->setInputSigma(temp);
                        }
                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {//temporary
                          Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
                          if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_B))
                             || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_B)))
                            temp(0,0) = level_acc_noise_sigma_;
                          else if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))
                            temp(0,0) = z_acc_noise_sigma_;
                          estimator_->getFuserExperiment(i)->setInputSigma(temp);
                        }

                      estimator_->getFuserExperiment(i)->setInputFlag();
                    }
                }
            }
        }

      if(bias_calib == calib_count_)
        {
          float yaw_gt = estimator_->getGTState(BasicEstimator::YAW_W_B, 0);
          float yaw_ee = estimator_->getEEState(BasicEstimator::YAW_W_B, 0);
          float yaw_ex = estimator_->getEXState(BasicEstimator::YAW_W_B, 0);

          if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
            {
              acc_xw_ = cos(yaw_ee) * acc_xi_ - sin(yaw_ee) * acc_yi_;
              acc_yw_ = sin(yaw_ee) * acc_xi_ + cos(yaw_ee) * acc_yi_;

              acc_xw_non_bias_ = cos(yaw_ee) * (acc_xi_ - acc_x_bias_) 
                - sin(yaw_ee) * (acc_yi_ -acc_y_bias_);
              acc_yw_non_bias_ = sin(yaw_ee) * (acc_xi_ - acc_x_bias_) 
                + cos(yaw_ee) * (acc_yi_ -acc_y_bias_);
              acc_zw_non_bias_ = acc_zw_ - acc_z_bias_;

              //temporary
              Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
              Eigen::Matrix<double, 2, 1> temp2 = Eigen::MatrixXd::Zero(2, 1); 

              for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                {
                  Eigen::MatrixXd egomotion_state;
                  int axis;

                  if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                    {//temporary
                      if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W)) 
                        {
                          temp(0, 0) = (double)acc_xw_non_bias_;
                          axis = BasicEstimator::X_W;
                        }
                      else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_B))
                        {
                          temp(0, 0) = (double)acc_xi_;
                          axis = BasicEstimator::X_B;
                        }
                      else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W)) 
                        {
                          temp(0, 0) = (double)acc_yw_non_bias_;
                          axis = BasicEstimator::Y_W;
                        }
                      else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_B))
                        {
                          temp(0, 0) = (double)acc_yi_;
                          axis = BasicEstimator::Y_B;
                        }
                      else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W))
                        {
                          temp(0, 0) = (double)acc_zw_non_bias_;
                          axis = BasicEstimator::Z_W;
			  /* considering the undescend mode, such as the phase of takeoff, the velocity should not below than 0*/
			  if(estimator_->getUnDescendMode() && (estimator_->getFuserEgomotion(i)->getEstimateState())(1,0) < 0)
			    estimator_->getFuserEgomotion(i)->resetState();  
                        }
                      estimator_->getFuserEgomotion(i)->prediction(temp);
                    }
                  if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                    {//temporary
                      if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W)) 
                        {
                          temp2(0, 0) = (double)acc_xw_;
                          axis = BasicEstimator::X_W;
                        }
                      else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_B))
                        {
                          temp2(0, 0) = (double)acc_xi_;
                          axis = BasicEstimator::X_B;
                        }
                      else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W)) 
                        {
                          temp2(0, 0) = (double)acc_yw_;
                          axis = BasicEstimator::Y_W;
                        }
                      else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_B))
                        {
                          temp2(0, 0) = (double)acc_yi_;
                          axis = BasicEstimator::Y_B;
                        }
                      else if(estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W))
                        {
                          temp2(0, 0) = (double)acc_zw_;
                          axis = BasicEstimator::Z_W;
			  /* considering the undescend mode, such as the phase of takeoff, the velocity should not below than 0*/
			  if(estimator_->getUnDescendMode() && (estimator_->getFuserEgomotion(i)->getEstimateState())(1,0) < 0)
			    estimator_->getFuserEgomotion(i)->resetState();  

                        }
                      estimator_->getFuserEgomotion(i)->prediction(temp2);

                    }
                  egomotion_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                  estimator_->setEEState(axis, 0, egomotion_state(0,0));
                  estimator_->setEEState(axis, 1, egomotion_state(1,0));
                }
            }
          estimator_->setEEState(BasicEstimator::Z_W, 2, acc_zw_non_bias_);


          if(estimate_mode_ & (1 << EXPERIMENT_MODE))
            {
              float yaw2;
              if(yaw_acc_trans_for_experiment_estimation_ == EGOMOTION_ESTIMATION_MODE)
                yaw2 = yaw_ee;
              if(yaw_acc_trans_for_experiment_estimation_ == GROUND_TRUTH_MODE)
                yaw2 = yaw_gt;
              if(yaw_acc_trans_for_experiment_estimation_ == EXPERIMENT_MODE)
                yaw2 = yaw_ex;

              acc_xw_ = cos(yaw2) * acc_xi_ - sin(yaw2) * acc_yi_;
              acc_yw_ = sin(yaw2) * acc_xi_ + cos(yaw2) * acc_yi_;

              acc_xw_non_bias_ = cos(yaw2) * (acc_xi_ - acc_x_bias_) 
                - sin(yaw2) * (acc_yi_ -acc_y_bias_);
              acc_yw_non_bias_ = sin(yaw2) * (acc_xi_ - acc_x_bias_) 
                + cos(yaw2) * (acc_yi_ -acc_y_bias_);
              acc_zw_non_bias_ = acc_zw_ - acc_z_bias_;

              //temporary
              Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
              Eigen::Matrix<double, 2, 1> temp2 = Eigen::MatrixXd::Zero(2, 1); 

              for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                {
                  Eigen::MatrixXd experiment_state;
                  int axis;

                  if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                    {//temporary
                      if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W)) 
                        {
                          temp(0, 0) = (double)acc_xw_non_bias_;
                          axis = BasicEstimator::X_W;
                        }
                      else if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_B))
                        {
                          temp(0, 0) = (double)acc_xi_;
                          axis = BasicEstimator::X_B;
                        }
                      else if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W)) 
                        {
                          temp(0, 0) = (double)acc_yw_non_bias_;
                          axis = BasicEstimator::Y_W;
                        }
                      else if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_B))
                        {
                          temp(0, 0) = (double)acc_yi_;
                          axis = BasicEstimator::Y_B;
                        }
                      else if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W))
                        {
                          temp(0, 0) = (double)acc_zw_non_bias_;
                          axis = BasicEstimator::Z_W;
			  /* considering the undescend mode, such as the phase of takeoff, the velocity should not below than 0*/
			  if(estimator_->getUnDescendMode() && (estimator_->getFuserEgomotion(i)->getEstimateState())(1,0) < 0)
			    estimator_->getFuserEgomotion(i)->resetState();  
                        }
                      estimator_->getFuserExperiment(i)->prediction(temp);
                    }
                  if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                    {//temporary
                      if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W)) 
                        {
                          temp2(0, 0) = (double)acc_xw_;
                          axis = BasicEstimator::X_W;
                        }
                      else if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_B))
                        {
                          temp2(0, 0) = (double)acc_xi_;
                          axis = BasicEstimator::X_B;
                        }
                      else if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W)) 
                        {
                          temp2(0, 0) = (double)acc_yw_;
                          axis = BasicEstimator::Y_W;
                        }
                      else if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_B))
                        {
                          temp2(0, 0) = (double)acc_yi_;
                          axis = BasicEstimator::Y_B;
                        }
                      else if(estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W))
                        {
                          temp2(0, 0) = (double)acc_zw_;
                          axis = BasicEstimator::Z_W;
                          /* considering the undescend mode, such as the phase of takeoff, the velocity should not below than 0*/
                          if(estimator_->getUnDescendMode() && (estimator_->getFuserEgomotion(i)->getEstimateState())(1,0) < 0)
                            estimator_->getFuserEgomotion(i)->resetState();  
                        }
                      estimator_->getFuserExperiment(i)->prediction(temp2);
                    }
                  experiment_state = estimator_->getFuserExperiment(i)->getEstimateState();
                  estimator_->setEXState(axis, 0, experiment_state(0,0));
                  estimator_->setEXState(axis, 1, experiment_state(1,0));
                }
            }

          publishAccData(stamp);
        }
      prev_time = imu_stamp_;
    }

    void publishAccData(ros::Time stamp)
    {
      aerial_robot_base::Acc acc_data;
      acc_data.header.stamp = stamp;

      acc_data.acc_raw.x = acc_xb_;
      acc_data.acc_raw.y = acc_yb_;
      acc_data.acc_raw.z = acc_zb_; // - g_value_;

      acc_data.acc_body_frame.x = acc_xi_;
      acc_data.acc_body_frame.y = acc_yi_; 
      acc_data.acc_body_frame.z = acc_zw_;

      acc_data.acc_world_frame.x = acc_xw_;
      acc_data.acc_world_frame.y = acc_yw_; 
      acc_data.acc_world_frame.z = acc_zw_;

      acc_data.acc_non_bias_world_frame.x = acc_xw_non_bias_;
      acc_data.acc_non_bias_world_frame.y = acc_yw_non_bias_;
      acc_data.acc_non_bias_world_frame.z = acc_zw_non_bias_;

      acc_pub_.publish(acc_data);
    }

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();

      nhp_.param("g_value", g_value_, 9.797 );
      printf(" g value is %f\n", g_value_);

      nhp_.param("imu_topic_name", imu_topic_name_, std::string("imu"));
      printf(" imu topic name is %s\n", imu_topic_name_.c_str());

      nhp_.param("level_acc_noise_sigma", level_acc_noise_sigma_, 0.01 );
      printf("level acc noise sigma  is %f\n", level_acc_noise_sigma_);

      nhp_.param("z_acc_noise_sigma", z_acc_noise_sigma_, 0.01 );
      printf("z acc noise sigma  is %f\n", z_acc_noise_sigma_);

      nhp_.param("acc_bias_noise_sigma", acc_bias_noise_sigma_, 0.01 );
      printf("acc bias noise sigma  is %f\n", acc_bias_noise_sigma_);

      nhp_.param("calib_time", calib_time_, 2.0 );
      printf("%s,  imu calib time is %f\n", ns.c_str(),  calib_time_);

      nhp_.param("landing_shock_force", landing_shock_force_, 5.0 );
      printf("%s,  landing shock force is %f\n", ns.c_str(),  landing_shock_force_);

      nhp_.param("yaw_acc_trans_for_experiment_estimation", yaw_acc_trans_for_experiment_estimation_, 0 );
      printf("%s,  yaw acc trans for experiment estimation is %d\n", ns.c_str(),  yaw_acc_trans_for_experiment_estimation_);


      nhp_.param("has_sub_imu_board", has_sub_imu_board_, false ); //whether have a sub imu board for yaw
      printf("%s, has sub imu mag board is %s\n", ns.c_str(),  has_sub_imu_board_?std::string("true").c_str():std::string("false").c_str());

      nhp_.param("sub_imu_topic_name", sub_imu_topic_name_, std::string("/sub/imu"));
      printf(" sub_imu topic name is %s\n", sub_imu_topic_name_.c_str());

      nhp_.param("imu_board", imu_board_, 0);
      if(imu_board_ != D_BOARD)
        ROS_WARN(" imu board is %s\n", (imu_board_ == KDUINO)?"kduino":"other board");

      if(imu_board_ == KDUINO)
        {
          nhp_.param("acc_scale", acc_scale_, g_value_ / 512.0);
          printf(" acc scale is %f\n", acc_scale_);
          nhp_.param("gyro_scale", gyro_scale_, (2279 * M_PI)/((32767.0 / 4.0f ) * 180.0));
          printf(" gyro scale is %f\n", gyro_scale_);
          nhp_.param("mag_scale", mag_scale_, 1200 / 32768.0);
          printf(" mag scale is %f\n", mag_scale_);
        }
    }

  };

};

#endif





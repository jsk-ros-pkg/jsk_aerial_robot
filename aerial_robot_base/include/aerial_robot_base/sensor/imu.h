/*
  1.the acc processing is wrong, related to the attitude estimation method
*/

#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <ros/ros.h>

#include <aerial_robot_base/ImuData.h>
#include <geometry_msgs/Vector3.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>
#include <aerial_robot_msgs/KduinoImu.h>
#include <aerial_robot_msgs/KduinoSimpleImu.h>
#include <jsk_stm/JskImu.h>

snamespace sensor_plugin
{
  class Imu  :public sensor_base_plugin::SensorBase
    {
    public:
      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* state_estimator)
      {
        nh_ = ros::NodeHandle(nh, "imu");
        nhp_ = ros::NodeHandle(nhp, "imu");
        state_estimator_ = state_estimator;

        baseRosParamInit();
        rosParamInit();


        if(imu_board_ == D_BOARD)
          {
            imu_sub_ = nh_.subscribe<jsk_stm::JskImu>("/imu", 1, &ImuData::ImuCallback, this, ros::TransportHints().tcpNoDelay()); 
          }

        if(imu_board_ == KDUINO)
          {
            imu_pub_ = nh_.advertise<aerial_robot_base::ImuData>("data", 2); 
            imu_sub_ = nh_.subscribe<aerial_robot_msgs::KduinoImu>("kduino/imu", 1, &ImuData::kduinoImuCallback, this, ros::TransportHints().tcpNoDelay()); 
            imu_simple_sub_ = nh_.subscribe<aerial_robot_msgs::KduinoSimpleImu>("kduino/simple_imu", 1, &ImuData::kduinoSimpleImuCallback, this, ros::TransportHints().tcpNoDelay()); 
          }


        acc_xb_ = 0, acc_yb_ = 0, acc_zb_ = 0,          
          gyro_xb_ = 0, gyro_yb_ = 0, gyro_zb_ = 0,
          mag_xb_ = 0, mag_yb_ = 0, mag_zb_ = 0,
          acc_xi_ = 0;      acc_yi_ = 0;

        acc_xw_ = 0;      acc_yw_ = 0;      acc_zw_ = 0;
        acc_xw_non_bias_ = 0;      acc_yw_non_bias_ = 0;      acc_zw_non_bias_ = 0;

        acc_x_bias_ = 0;      acc_y_bias_ = 0;      acc_z_bias_ = 0;

        pitch_ = 0;      roll_ = 0;      yaw_ = 0;
        height_ = 0;

        calib_count_ = 100; // temporarily

      }

      ~Imu () { }
      Imu () { }

      const static uint8_t D_BOARD = 0;
      const static uint8_t KDUINO = 1;

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
      ros::Publisher  imu_pub_;
      ros::Subscriber  imu_sub_;
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


      //***  world frame
      float acc_xw_, acc_xw_non_bias_;
      float acc_yw_, acc_yw_non_bias_;
      float acc_zw_, acc_zw_non_bias_;

      double acc_x_bias_;
      double acc_y_bias_;
      double acc_z_bias_;

      int imu_board_;

      ros::Time imu_stamp_;

      float height_;
      float v_bat_;   //*** battery

      int calib_count_;
      double calib_time_;

      void ImuCallback(const jsk_stm::JskImuConstPtr& imu_msg)
      {
        imu_stamp_ = imu_msg->header.stamp;
        state_estimator_->setSystemTimeStamp(imu_stamp_);

        roll_  = imu_msg->angles.x;
        pitch_ = imu_msg->angles.y;
        yaw_   = imu_msg->angles.z;

        acc_xb_ = imu_msg->acc_data.x;
        acc_yb_ = imu_msg->acc_data.y;
        acc_zb_ = imu_msg->acc_data.z;
        gyro_xb_ = imu_msg->gyro_data.x;
        gyro_yb_ = imu_msg->gyro_data.y;
        gyro_zb_ = imu_msg->gyro_data.z;

        mag_xb_ = imu_msg->mag_data.x;
        mag_yb_ = imu_msg->mag_data.y;
        mag_zb_ = imu_msg->mag_data.z;
        height_ = imu_msg->altitude;  //cm

        imuDataConverter(imu_stamp_);

      }


      void kduinoImuCallback(const aerial_robot_msgs::KduinoImuConstPtr& imu_msg)
      {
        imu_stamp_ = imu_msg->stamp;
        state_estimator_->setSystemTimeStamp(imu_stamp_);

        roll_  = M_PI * imu_msg->angle[0] / 10.0 / 180.0; //raw data is 10 times
        pitch_ = M_PI * imu_msg->angle[1] / 10.0 / 180.0; //raw data is 10 times
        yaw_   = M_PI * imu_msg->angle[2] / 180.0;

        acc_xb_ = imu_msg->accData[0] * acc_scale_;
        acc_yb_ = imu_msg->accData[1] * acc_scale_;
        acc_zb_ = imu_msg->accData[2] * acc_scale_;
        gyro_xb_ = imu_msg->gyroData[0] * gyro_scale_;
        gyro_yb_ = imu_msg->gyroData[1] * gyro_scale_;
        gyro_zb_ = imu_msg->gyroData[2] * gyro_scale_;

        mag_xb_ = imu_msg->magData[0] * acc_scale_;
        mag_yb_ = imu_msg->magData[1] * acc_scale_;
        mag_zb_ = imu_msg->magData[2] * acc_scale_;

        /*
          mag_xb_ = imu_msg->magData[0] * mag_scale_;
          mag_yb_ = imu_msg->magData[1] * mag_scale_;
          mag_zb_ = imu_msg->magData[2] * mag_scale_;
        */
        //* height
        //height_ = imu_msg->altitude / 100.0;  //cm
        height_ = imu_msg->altitude;  //cm

        imuDataConverter(imu_stamp_);
      }

      void kduinoSimpleImuCallback(const aerial_robot_msgs::KduinoSimpleImuConstPtr& imu_msg)
      {
        imu_stamp_ = imu_msg->stamp;
        state_estimator_->setSystemTimeStamp(imu_stamp_);

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
        static double hz_calib = 0;
        //* calculate accTran
#if 0 // use x,y for factor4 and z for factor3
        acc_xi_ = (acc_xb_) * cos(pitch_) + 
          (acc_yb_) * sin(pitch_) * sin(roll_) + 
          (acc_zb_) * sin(pitch_) * cos(roll_);
        acc_yi_ = (acc_yb_) * cos(roll_) - (acc_zb_) * sin(roll_);
        acc_zw_ = (acc_xb_) * (-sin(pitch_)) + 
          (acc_yb_) * cos(pitch_) * sin(roll_) + 
          (acc_zb_) * cos(pitch_) * cos(roll_) + (-g_value_);
#else  // use approximation
        acc_xi_ =  (acc_zb_) * sin(pitch_) * cos(roll_);
        acc_yi_ =  - (acc_zb_) * sin(roll_);
        acc_zw_ = (acc_zb_) * cos(pitch_) * cos(roll_) + (- g_value_);
#endif

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
              }

            //hz estimation
            hz_calib += time_interval;

            //acc bias
            acc_x_bias_ += acc_xi_;
            acc_y_bias_ += acc_yi_;
            acc_z_bias_ += acc_zw_;

            if(bias_calib == calib_count_)
              {
                acc_x_bias_ /= calib_count_;
                acc_y_bias_ /= calib_count_;
                acc_z_bias_ /= calib_count_;

                hz_calib /= (calib_count_ - 1 );

                ROS_WARN("accX bias is %f, accY bias is %f, accZ bias is %f, hz is %f", acc_x_bias_, acc_y_bias_, acc_z_bias_, hz_calib);
                if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
                  {
                    for(int i = 0; i < fuser_egomotion_no_; i++)
                      {
                        getFuserEgomotion(i)->updateModelFromDt(hz_calib);

                        if(getFuserEgomotionName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                          {//temporary
                            Eigen::Matrix<double, 2, 1> temp = Eigen::MatrixXd::Zero(2, 1); 
                            temp(1,0) = acc_bias_noise_sigma_;
                            ROS_WARN("this is bias mode");
                            if((getFuserEgomotionId(i) & (1 << X_W)) || (getFuserEgomotionId(i) & (1 << X_B)))
                              {
                                temp(0,0) = level_acc_noise_sigma_;
                                getFuserEgomotion(i)->setInitState(acc_x_bias_, 2);
                              }
                            else if((getFuserEgomotionId(i) & (1 << Y_W)) || (getFuserEgomotionId(i) & (1 << Y_B)))
                              {
                                temp(0,0) = level_acc_noise_sigma_;
                                getFuserEgomotion(i)->setInitState(acc_y_bias_, 2);
                              }
                            else if((getFuserEgomotionId(i) & (1 << Z_W)))
                              {
                                temp(0,0) = z_acc_noise_sigma_;
                                getFuserEgomotion(i)->setInitState(acc_z_bias_, 2);
                              }
                            getFuserEgomotion(i)->setInputSigma(temp);
                          }

                        if(getFuserEgomotionName(i) == "kalman_filter/kf_pose_vel_acc")
                          {//temporary
                            Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
                            if((getFuserEgomotionId(i) & (1 << X_W)) || (getFuserEgomotionId(i) & (1 << X_B))
                               || (getFuserEgomotionId(i) & (1 << Y_W)) || (getFuserEgomotionId(i) & (1 << Y_B)))
                                temp(0,0) = level_acc_noise_sigma_;
                            else if((getFuserEgomotionId(i) & (1 << Z_W)))
                                temp(0,0) = z_acc_noise_sigma_;
                            getFuserEgomotion(i)->setInputSigma(temp);
                          }
                        getFuserEgomotion(i)->setInputFlag;
                      }
                  }

                if(estimate_mode_ & (1 << EXPERIMENT_MODE))
                  {
                    for(int i = 0; i < fuser_experiment_no_; i++)
                      {
                        getFuserExperiment(i)->updateModelFromDt(hz_calib);

                        if(getFuserExperimentName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                          {//temporary
                            Eigen::Matrix<double, 2, 1> temp = Eigen::MatrixXd::Zero(2, 1); 
                            temp(1,0) = acc_bias_noise_sigma_;
                            ROS_WARN("this is bias mode");
                            if((getFuserExperimentId(i) & (1 << X_W)) || (getFuserExperimentId(i) & (1 << X_B)))
                              {
                                temp(0,0) = level_acc_noise_sigma_;
                                getFuserExperiment(i)->setInitState(acc_x_bias_, 2);
                              }
                            else if((getFuserExperimentId(i) & (1 << Y_W)) || (getFuserExperimentId(i) & (1 << Y_B)))
                              {
                                temp(0,0) = level_acc_noise_sigma_;
                                getFuserExperiment(i)->setInitState(acc_y_bias_, 2);
                              }
                            else if((getFuserExperimentId(i) & (1 << Z_W)))
                              {
                                temp(0,0) = z_acc_noise_sigma_;
                                getFuserExperiment(i)->setInitState(acc_z_bias_, 2);
                              }
                          }
                        if(getFuserExperimentName(i) == "kalman_filter/kf_pose_vel_acc")
                          {//temporary
                            Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
                            if((getFuserExperimentId(i) & (1 << X_W)) || (getFuserExperimentId(i) & (1 << X_B))
                               || (getFuserExperimentId(i) & (1 << Y_W)) || (getFuserExperimentId(i) & (1 << Y_B)))
                                temp(0,0) = level_acc_noise_sigma_;
                            else if((getFuserExperimentId(i) & (1 << Z_W)))
                                temp(0,0) = z_acc_noise_sigma_;
                            getFuserExperiment(i)->setInputSigma(temp);
                          }

                        getFuserExperiment(i)->setInputFlag;
                      }
                  }
              }
          }

        if(bias_calib == calib_count_)
          {
            float yaw_gt = state_estimator_->getGTState(YAW_W_B, 0);
            float yaw_ee = state_estimator_->getEEState(YAW_W_B, 0);
            float yaw_ex = state_estimator_->getEXState(YAW_W_B, 0);

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

                for(int i = 0; i < fuser_egomotion_no_; i++)
                  {
                    if(getFuserEgomotionName(i) == "kalman_filter/kf_pose_vel_acc")
                      {//temporary
                        if(getFuserEgomotionId(i) & (1 << X_W)) 
                          temp(0, 0) = (double)acc_xw_non_bias_;
                        else if(getFuserEgomotionId(i) & (1 << X_B))
                          temp(0, 0) = (double)acc_xi_;
                        else if(getFuserEgomotionId(i) & (1 << Y_W)) 
                          temp(0, 0) = (double)acc_yw_non_bias_;
                        else if(getFuserEgomotionId(i) & (1 << Y_B))
                          temp(0, 0) = (double)acc_yi_;
                        else if(getFuserEgomotionId(i) & (1 << Z_W))
                          temp(0, 0) = (double)acc_zw_non_bias_;

                        getFuserEgomotion(i)->prediction(temp);
                      }
                    if(getFuserEgomotionName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                      {//temporary
                        if(getFuserEgomotionId(i) & (1 << X_W)) 
                          temp2(0, 0) = (double)acc_xw_;
                        else if(getFuserEgomotionId(i) & (1 << X_B))
                          temp2(0, 0) = (double)acc_xi_;
                        else if(getFuserEgomotionId(i) & (1 << Y_W)) 
                          temp2(0, 0) = (double)acc_yw_;
                        else if(getFuserEgomotionId(i) & (1 << Y_B))
                          temp2(0, 0) = (double)acc_yi_;
                        else if(getFuserEgomotionId(i) & (1 << Z_W))
                          temp2(0, 0) = (double)acc_zw_;

                        getFuserEgomotion(i)->prediction(temp2);
                      }
                  }
              }

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

                for(int i = 0; i < fuser_egomotion_no_; i++)
                  {
                    if(getFuserExperimentName(i) == "kalman_filter/kf_pose_vel_acc")
                      {//temporary
                        if(getFuserExperimentId(i) & (1 << X_W)) 
                          temp(0, 0) = (double)acc_xw_non_bias_;
                        else if(getFuserExperimentId(i) & (1 << X_B))
                          temp(0, 0) = (double)acc_xi_;
                        else if(getFuserExperimentId(i) & (1 << Y_W)) 
                          temp(0, 0) = (double)acc_yw_non_bias_;
                        else if(getFuserExperimentId(i) & (1 << Y_B))
                          temp(0, 0) = (double)acc_yi_;
                        else if(getFuserExperimentId(i) & (1 << Z_W))
                          temp(0, 0) = (double)acc_zw_non_bias_;

                        getFuserExperiment(i)->prediction(temp);
                      }

                    if(getFuserExperimentName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                      {//temporary
                        if(getFuserExperimentId(i) & (1 << X_W)) 
                          temp2(0, 0) = (double)acc_xw_;
                        else if(getFuserExperimentId(i) & (1 << X_B))
                          temp2(0, 0) = (double)acc_xi_;
                        else if(getFuserExperimentId(i) & (1 << Y_W)) 
                          temp2(0, 0) = (double)acc_yw_;
                        else if(getFuserExperimentId(i) & (1 << Y_B))
                          temp2(0, 0) = (double)acc_yi_;
                        else if(getFuserExperimentId(i) & (1 << Z_W))
                          temp2(0, 0) = (double)acc_zw_;
                        getFuserExperiment(i)->prediction(temp2);
                      }
                  }
              }

            if(imu_board_ != D_BOARD) publishImuData(stamp);
          }
        prev_time = imu_stamp_;
      }

      void publishImuData(ros::Time stamp)
      {
        aerial_robot_base::ImuData imu_data;
        imu_data.header.stamp = stamp;

        imu_data.height = height_;

        imu_data.angles.x = roll_;
        imu_data.angles.y = pitch_;
        imu_data.angles.z = yaw_;

        imu_data.accelerometer.x = acc_xb_;
        imu_data.accelerometer.y = acc_yb_;
        imu_data.accelerometer.z = acc_zb_;

        imu_data.gyrometer.x = gyro_xb_;
        imu_data.gyrometer.y = gyro_yb_;
        imu_data.gyrometer.z = gyro_zb_;

        imu_data.magnetometer.x = mag_xb_;
        imu_data.magnetometer.y = mag_yb_;
        imu_data.magnetometer.z = mag_zb_;

        imu_data.acc_body_frame.x = acc_xi_;
        imu_data.acc_body_frame.y = acc_yi_; 
        imu_data.acc_body_frame.z = acc_zw_;

        imu_data.acc_world_frame.x = acc_xw_;
        imu_data.acc_world_frame.y = acc_yw_; 
        imu_data.acc_world_frame.z = acc_zw_;

        imu_data.acc_non_bias_world_frame.x = acc_xw_non_bias_;
        imu_data.acc_non_bias_world_frame.y = acc_yw_non_bias_;
        imu_data.acc_non_bias_world_frame.z = acc_zw_non_bias_;

        imu_pub_.publish(imu_data);
      }

      void rosParamInit(ros::NodeHandle nh)
      {
        std::string ns = nh.getNamespace();

        nh.param("g_value", g_value_, 9.797 );
        printf(" g value is %f\n", g_value_);

        nh.param("level_acc_noise_sigma", level_acc_noise_sigma_, 0.01 );
        printf("level acc noise sigma  is %f\n", level_acc_noise_sigma_);

        nh.param("z_acc_noise_sigma", z_acc_noise_sigma_, 0.01 );
        printf("z acc noise sigma  is %f\n", z_acc_noise_sigma_);

        nh.param("acc_bias_noise_sigma", acc_bias_noise_sigma_, 0.01 );
        printf("acc bias noise sigma  is %f\n", acc_bias_noise_sigma_);

        nh.param("calib_time", calib_time_, 2.0 );
        printf("%s,  imu calib time is %f\n", ns.c_str(),  calib_time_);

        nh.param("yaw_acc_trans_for_experiment_estimation", yaw_acc_trans_for_experiment_estimation_, 0 );
        printf("%s,  yaw acc trans for experiment estimation is %d\n", ns.c_str(),  yaw_acc_trans_for_experiment_estimation_);

        nh.param("imu_board", imu_board_, 0);
        if(imu_board_ != D_BOARD)
          ROS_WARN(" imu board is %s\n", (imu_board_ == KDUINO)?"kduino":"other board");

        if(imu_board_ == KDUINO)
          {
            nh.param("acc_scale", acc_scale_, g_value_ / 512.0);
            printf(" acc scale is %f\n", acc_scale_);
            nh.param("gyro_scale", gyro_scale_, (2279 * M_PI)/((32767.0 / 4.0f ) * 180.0));
            printf(" gyro scale is %f\n", gyro_scale_);
            nh.param("mag_scale", mag_scale_, 1200 / 32768.0);
            printf(" mag scale is %f\n", mag_scale_);
          }
      }

    };

};

#endif





/*
  1.the acc processing is wrong, related to the attitude estimation method
*/

#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <ros/ros.h>
#include <aerial_robot_base/ImuData.h>
#include <aerial_robot_base/state_estimation.h>
#include <kalman_filter/kf_pos_vel_acc.h>
#include <kalman_filter/digital_filter.h>

#include <geometry_msgs/Vector3.h>
#include <aerial_robot_msgs/KduinoImu.h>
#include <aerial_robot_msgs/KduinoSimpleImu.h>


/* #define acc_scale_  9.797 / 512.0 */
/* #define gyro_scale_ (2279 * M_PI)/((32767.0 / 4.0f ) * 180.0) */
/* #define mag_scale_  1200 / 32768.0 */


class ImuData
{
 public:
 ImuData(ros::NodeHandle nh,
         ros::NodeHandle nh_private,
         BasicEstimator* state_estimator,
         bool kalman_filter_flag,
         KalmanFilterPosVelAcc *kf_x,
         KalmanFilterPosVelAcc *kf_y,
         KalmanFilterPosVelAcc *kf_z,
         KalmanFilterPosVelAccBias *kfb_x,
         KalmanFilterPosVelAccBias *kfb_y,
         KalmanFilterPosVelAccBias *kfb_z,
         KalmanFilterPosVelAcc *kf_x_vel,
         KalmanFilterPosVelAcc *kf_y_vel,
         KalmanFilterPosVelAcc *kf_z2,
         KalmanFilterPosVelAccBias *kfb_x_vel,
         KalmanFilterPosVelAccBias *kfb_y_vel,
         bool kalman_filter_debug,
         int kalman_filter_axis,
         KalmanFilterPosVelAccBias *kf1,
         KalmanFilterPosVelAccBias *kf2,
         FirFilter *lpf_acc_x,
         FirFilter *lpf_acc_y,
         FirFilter *lpf_acc_z,
         bool simulation_flag)  
   : nh_(nh, "imu"), nhp_(nh_private, "imu")
    {
      state_estimator_ = state_estimator;
      
      imu_pub_ = nh_.advertise<aerial_robot_base::ImuData>("data", 2); 
      imu_sub_ = nh_.subscribe<aerial_robot_msgs::KduinoImu>("kduino/imu", 1, &ImuData::kduinoImuCallback, this, ros::TransportHints().tcpNoDelay()); 
      imu_simple_sub_ = nh_.subscribe<aerial_robot_msgs::KduinoSimpleImu>("kduino/simple_imu", 1, &ImuData::kduinoSimpleImuCallback, this, ros::TransportHints().tcpNoDelay()); 

      rosParamInit(nhp_);

      simulation_flag_ = simulation_flag;
      kalman_filter_flag_ = kalman_filter_flag;

      kf_x_ = kf_x;      kf_y_ = kf_y;      kf_z_ = kf_z;
      kfb_x_ = kfb_x;      kfb_y_ = kfb_y;      kfb_z_ = kfb_z;
  
      kf_x_vel_ = kf_x_vel;      kf_y_vel_ = kf_y_vel;      kf_z2_ = kf_z2;
      kfb_x_vel_ = kfb_x_vel;      kfb_y_vel_ = kfb_y_vel;

      kalman_filter_debug_ = kalman_filter_debug;
      kalman_filter_axis_  = kalman_filter_axis;
      kf1_ = kf1;      kf2_ = kf2;

      lpf_acc_x_ = lpf_acc_x;      lpf_acc_y_ = lpf_acc_y;      lpf_acc_z_ = lpf_acc_z;


      acc_xb_ = 0, acc_yb_ = 0, acc_zb_ = 0,
      gyro_xb_ = 0, gyro_yb_ = 0, gyro_zb_ = 0,
      mag_xb_ = 0, mag_yb_ = 0, mag_zb_ = 0,

      acc_xi_ = 0;      acc_yi_ = 0;

      acc_xw_ = 0;      acc_yw_ = 0;      acc_zw_ = 0;
      acc_xw_non_bias_ = 0;      acc_yw_non_bias_ = 0;      acc_zw_non_bias_ = 0;

      acc_x_bias_ = 0;      acc_y_bias_ = 0;      acc_z_bias_ = 0;

      pitch_ = 0;      roll_ = 0;      yaw_ = 0;
      height_ = 0;

    }

  ~ImuData ()
    {
    };

  const static int CALIB_COUNT = 200;


  inline void setPitchValue(float pitch_value) {   pitch_ = pitch_value;  }
  inline void setRollValue(float roll_value) {    roll_ = roll_value;  }
  inline void setYawImuValue(float yaw_value)  {    yaw_ = yaw_value;  }
  inline void setZImuValue(float z_value)  {    height_ = z_value;  }
  inline float getPitchValue()  {    return pitch_;  }
  inline float getRollValue()  {    return roll_;  }
  inline float getYawValue()  {    return yaw_;  }

  inline float getAccXbValue()  {    return acc_xb_;  }
  inline float getAccYbValue()  {    return acc_yb_;  }
  inline float getAccZbValue()  {    return acc_zb_;  }

  inline ros::Time getImuStamp(){return imu_stamp_;}

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher  imu_pub_;
  ros::Subscriber  imu_sub_;
  ros::Subscriber  imu_simple_sub_;

  BasicEstimator* state_estimator_;

  bool simulation_flag_;

  bool kalman_filter_flag_;
  KalmanFilterPosVelAcc *kf_x_;
  KalmanFilterPosVelAcc *kf_y_;
  KalmanFilterPosVelAcc *kf_z_;
  KalmanFilterPosVelAccBias *kfb_x_;
  KalmanFilterPosVelAccBias *kfb_y_;
  KalmanFilterPosVelAccBias *kfb_z_;

  KalmanFilterPosVelAcc *kf_x_vel_;
  KalmanFilterPosVelAcc *kf_y_vel_;
  KalmanFilterPosVelAcc *kf_z2_;
  KalmanFilterPosVelAccBias *kfb_x_vel_;
  KalmanFilterPosVelAccBias *kfb_y_vel_;

  bool kalman_filter_debug_;
  int kalman_filter_axis_;
  KalmanFilterPosVelAccBias *kf1_;
  KalmanFilterPosVelAccBias *kf2_;

  //FIR filter for acc
  FirFilter *lpf_acc_x_;
  FirFilter *lpf_acc_y_;
  FirFilter *lpf_acc_z_;

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

  float filtered_acc_x_; 
  float filtered_acc_y_; 
  float filtered_acc_z_; 

  //*** trans_acc with intermediate frame between world frame and board frame
  float acc_xi_, acc_yi_;


  //***  world frame
  float acc_xw_, acc_xw_non_bias_;
  float acc_yw_, acc_yw_non_bias_;
  float acc_zw_, acc_zw_non_bias_;

  double acc_x_bias_;
  double acc_y_bias_;
  double acc_z_bias_;


  ros::Time imu_stamp_;

  float height_;
  float v_bat_;   //*** battery



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
    mag_xb_ = imu_msg->magData[0] * mag_scale_;
    mag_yb_ = imu_msg->magData[1] * mag_scale_;
    mag_zb_ = imu_msg->magData[2] * mag_scale_;

    //* height
    height_ = imu_msg->altitude / 100.0;  //cm

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
    if(bias_calib < CALIB_COUNT)
      {
        bias_calib ++;

        //acc bias
        acc_x_bias_ += acc_xi_;
        acc_y_bias_ += acc_yi_;
        acc_z_bias_ += acc_zw_;

        if(bias_calib == CALIB_COUNT)
          {
            acc_x_bias_ /= CALIB_COUNT;
            acc_y_bias_ /= CALIB_COUNT;
            acc_z_bias_ /= CALIB_COUNT;
            ROS_WARN("accX bias is %f, accY bias is %f, accZ bias is %f", acc_x_bias_, acc_y_bias_, acc_z_bias_);

            if(kalman_filter_flag_)
              {
                kf_x_->setInputFlag();
                kf_y_->setInputFlag();
                kf_z_->setInputFlag();

                //for vel flow
                kf_x_vel_->setInputFlag();
                kf_y_vel_->setInputFlag();
                kf_z2_->setInputFlag();


                //for bias mode
                kfb_x_->setInitState(acc_x_bias_, 2);
                kfb_y_->setInitState(acc_y_bias_, 2);
                kfb_z_->setInitState(acc_z_bias_, 2);
                kfb_x_->setInputFlag();
                kfb_y_->setInputFlag();
                kfb_z_->setInputFlag();

                //for velocity
                kfb_x_vel_->setInitState(acc_x_bias_, 2);
                kfb_y_vel_->setInitState(acc_y_bias_, 2);
                kfb_x_vel_->setInputFlag();
                kfb_y_vel_->setInputFlag();

              }
            if(kalman_filter_debug_)
              {

                if(kalman_filter_axis_ == 0)
                  {
                    kf1_->setInitState(acc_x_bias_, 2);
                    kf2_->setInitState(acc_x_bias_, 2);
                    kf1_->setInputFlag();
                    kf2_->setInputFlag();

                  }
                if(kalman_filter_axis_ == 1)
                  {
                    kf1_->setInitState(acc_y_bias_, 2);
                    kf2_->setInitState(acc_y_bias_, 2);
                    kf1_->setInputFlag();
                    kf2_->setInputFlag();
                  }
              }
          }
      }

    float yaw2 = state_estimator_->getStatePsiBoard();

    if(bias_calib == CALIB_COUNT)
      {
        acc_xw_ = cos(yaw2) * acc_xi_ - sin(yaw2) * acc_yi_;
        acc_yw_ = sin(yaw2) * acc_xi_ + cos(yaw2) * acc_yi_;

        acc_xw_non_bias_ = cos(yaw2) * (acc_xi_ - acc_x_bias_) 
          - sin(yaw2) * (acc_yi_ -acc_y_bias_);
        acc_yw_non_bias_ = sin(yaw2) * (acc_xi_ - acc_x_bias_) 
          + cos(yaw2) * (acc_yi_ -acc_y_bias_);
        acc_zw_non_bias_ = acc_zw_ - acc_z_bias_;

        Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
        Eigen::Matrix<double, 2, 1> temp2 = Eigen::MatrixXd::Zero(1, 1); 
        //temp << 0;
        if(kalman_filter_flag_)
          {

            temp(0, 0) = (double)acc_xw_non_bias_;
            //kf_x_->KalmanFilter<2, 1, 1>::prediction(temp);
            temp(0, 0) = (double)acc_yw_non_bias_;
            //kf_y_->KalmanFilter<2, 1, 1>::prediction(temp);
            temp(0, 0) = (double)acc_zw_non_bias_;
            //kf_z_->KalmanFilter<2, 1, 1>::prediction(temp);

            //with bias
            temp2(0, 0) = (double)acc_xw_;
            kfb_x_->prediction(temp2);
            temp2(0, 0) = (double)acc_yw_;
            kfb_y_->prediction(temp2);
            temp2(0, 0) = (double)acc_zw_;
            kfb_z_->prediction(temp2);

            //optical without accurate time stamp
            temp(0, 0) = (double)acc_xi_ - acc_x_bias_;
            //kf_x_vel_->KalmanFilter<2, 1, 1>::prediction(temp);
            temp(0, 0) = (double)acc_yi_ - acc_y_bias_;
            //kf_y_vel_->KalmanFilter<2, 1, 1>::prediction(temp);

            temp(0, 0) = (double)acc_zw_non_bias_;
            //kf_z2_->KalmanFilter<2, 1, 1>::prediction(temp);
            temp2(0, 0) = (double)acc_xi_;
            kfb_x_vel_->prediction(temp2);
            temp2(0, 0) = (double)acc_yi_;
            kfb_y_vel_->prediction(temp2);
          }

        if(kalman_filter_debug_)
          {
            if(kalman_filter_axis_ == 0)
              { //x axis
                temp2(0, 0) = (double)acc_xw_;
                kf1_->prediction(temp2);
                kf2_->prediction(temp2);
              }
            else if(kalman_filter_axis_ == 1)
              { //y axis
                temp2(0, 0) = (double)acc_yw_;
                kf1_->prediction(temp2);
                kf2_->prediction(temp2);
              }
          }
        publishImuData(stamp);
      }
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
    nh.param("g_value", g_value_, 9.797 );
    printf(" acc scale is %f\n", g_value_);

    nh.param("acc_scale", acc_scale_, g_value_ / 512.0);
    printf(" acc scale is %f\n", acc_scale_);
    nh.param("gyro_scale", gyro_scale_, (2279 * M_PI)/((32767.0 / 4.0f ) * 180.0));
    printf(" gyro scale is %f\n", gyro_scale_);
    nh.param("mag_scale", mag_scale_, 1200 / 32768.0);
    printf(" mag scale is %f\n", mag_scale_);
  }

};

#endif





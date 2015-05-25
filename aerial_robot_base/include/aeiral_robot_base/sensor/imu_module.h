/*
  1.the acc processing is wrong, related to the attitude estimation method
*/

#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <ros/ros.h>
#include <aerial_robot_base/ImuData.h>
#include <aerial_robot_base/ImuSim.h>
#include <aerial_robot_base/state_estimation.h>
#include <aerial_robot_base/kalman_filter.h>
#include <aerial_robot_base/digital_filter.h>

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
         Estimator* state_estimator,
         bool kalman_filter_flag,
         KalmanFilterPosVelAcc *kf_x,
         KalmanFilterPosVelAcc *kf_y,
         KalmanFilterPosVelAcc *kf_z,
         KalmanFilterPosVelAccBias *kfb_x,
         KalmanFilterPosVelAccBias *kfb_y,
         KalmanFilterPosVelAccBias *kfb_z,
         KalmanFilterPosVelAcc *kf_x_opt,
         KalmanFilterPosVelAcc *kf_y_opt,
         KalmanFilterPosVelAcc *kf_z_opt,
         KalmanFilterPosVelAccBias *kfb_x_opt,
         KalmanFilterPosVelAccBias *kfb_y_opt,
         KalmanFilterPosVelAccBias *kfb_z_opt,
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
      imu_sub_ = nh_.subscribe<aerial_robot_msgs::KduinoImu>("kduino/imu", 1, &ImuData::kduinoImuCallback, this, this, ros::TransportHints().tcpNoDelay()); 
      imu_simple_sub_ = nh_.subscribe<aerial_robot_msgs::KduinoSimpleImu>("kduino/simple_imu", 1, &ImuData::kduinoSimpleImuCallback, this, ros::TransportHints().tcpNoDelay()); 

      rosParamInit(nhp_);

      simulation_flag_ = simulation_flag;
      kalman_filter_flag_ = kalman_filter_flag;

      kf_x_ = kf_x;      kf_y_ = kf_y;      kf_z_ = kf_z;
      kfb_x_ = kfb_x;      kfb_y_ = kfb_y;      kfb_z_ = kfb_z;
  
      kf_x_opt_ = kf_x_opt;      kf_y_opt_ = kf_y_opt;      kf_z_opt_ = kf_z_opt;
      kfb_x_opt_ = kfb_x_opt;      kfb_y_opt_ = kfb_y_opt;      kfb_z_opt_ = kfb_z_opt;

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



  inline void setPitchValue(float pitch_value) {   pitch_ = pitch_value;  }
  inline void setRollValue(float roll_value) {    roll_ = roll_value;  }
  inline void setYawImuValue(float yaw_value)  {    yaw = yaw_value;  }
  inline void setZImuValue(float z_value)  {    height_ = z_value;  }
  inline float getPitchValue()  {    return pitch_;  }
  inline float getRollValue()  {    return roll_;  }
  inline float getYawValue()  {    return yaw_;  }

  inline float getAccXbValue()  {    return acc_xb_;  }
  inline float getAccYbValue()  {    return acc_yb_;  }
  inline float getAccZbValue()  {    return acc_zb_;  }

  const static float G = 9.797;
  const static int CALIB_COUNT = 200;
  const static int KDUINO_BOARD = 0;
  const static int ASCTEC_BOARD = 1;

  inline ros::Time getImuStamp(){return imu_stamp_;}

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher  imu_pub_;
  ros::Subscriber  imu_sub_;
  ros::Subscriber  imu_simple_sub_;

  Estimator* state_estimator_;

  bool simulation_flag_;

  bool kalman_filter_flag_;
  KalmanFilterPosVelAcc *kf_x_;
  KalmanFilterPosVelAcc *kf_y_;
  KalmanFilterPosVelAcc *kf_z_;
  KalmanFilterPosVelAccBias *kfb_x_;
  KalmanFilterPosVelAccBias *kfb_y_;
  KalmanFilterPosVelAccBias *kfb_z_;

  KalmanFilterPosVelAcc *kf_x_opt_;
  KalmanFilterPosVelAcc *kf_y_opt_;
  KalmanFilterPosVelAcc *kf_z_opt_;
  KalmanFilterPosVelAccBias *kfb_x_opt_;
  KalmanFilterPosVelAccBias *kfb_y_opt_;
  KalmanFilterPosVelAccBias *kfb_z_opt_;

  bool kalman_filter_debug;
  int kalman_filter_axis_;
  KalmanFilterPosVelAccBias *kf1_;
  KalmanFilterPosVelAccBias *kf2_;

  //FIR filter for acc
  FirFilter *lpf_acc_x_;
  FirFilter *lpf_acc_y_;
  FirFilter *lpf_acc_z_;

  float acc_scale_;
  float gyro_scale_;
  float mag_scale_;

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




  void kduinoImuCallback(const aerial_robot_base_common::KduinoImuConstPtr& imu_msg)
  {
    imu_stamp_ = imu_msg->stamp;
    estimator_->setSystemTimeStamp(imu_stamp_);

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

  void kduinoSimpleImuCallback(const aerial_robot_base_common::KduinoSimpleImuConstPtr& imu_msg)
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
    //* calculate accTran
#if 0 // use x,y for factor4 and z for factor3
    acc_xi_ = (acc_xb_) * cos(pitch_) + 
      (acc_yb_) * sin(pitch_) * sin(roll_) + 
      (acc_zb_) * sin(pitch_) * cos(roll_);
    acc_yi_ = (acc_yb_) * cos(roll_) - (acc_zb_) * sin(roll_);
    acc_zw_ = (acc_xb_) * (-sin(pitch_)) + 
      (acc_yb_) * cos(pitch_) * sin(roll_) + 
      (acc_zb_) * cos(pitch_) * cos(roll_) + (-G);
#else  // use approximation
    acc_xi_ =  (acc_zb_) * sin(pitch_) * cos(roll_);
    acc_yi_ =  - (acc_zb_) * sin(roll_);
    acc_zw_ = (acc_zb_) * cos(pitch_) * cos(roll_) + (-G);
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

            kf_x_->setInputStartFlag();
            kf_y_->setInputStartFlag();
            kf_z_->setInputStartFlag();

            //for optical flow
            kf_x_opt_->setInputStartFlag();
            kf_y_opt_->setInputStartFlag();
            kf_z_opt_->setInputStartFlag();


            //for bias mode
            kfb_x_->setInitImuBias(acc_x_bias_);
            kfb_y_->setInitImuBias(acc_y_bias_);
            kfb_z_->setInitImuBias(acc_z_bias_);

            //for optical flow
            kfb_x_opt_->setInitImuBias(acc_x_bias_);
            kfb_y_opt_->setInitImuBias(acc_y_bias_);
            kfb_z_opt_->setInitImuBias(acc_z_bias_);

            if(kalman_filter_axis_ == 0)
              {
                kf1_->setInitImuBias(accXBias);
                kf2_->setInitImuBias(accXBias);
              }
            if(kalmanFilter_axis_ == 1)
              {
                kf1_->setInitImuBias(accYBias);
                kf2_->setInitImuBias(accYBias);
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

        if(kalman_filter_flag_)
          {
            //kf_x_->prediction((double)acc_xw_non_bias_, stamp);
            aerial_robot_base::ImuQuPtr x_non_bias_ptr
              = boost::shared_non_bias_ptr<aerial_robot_base::ImuQu> ( new aerial_robot_base::ImuQu() );
            x_non_bias_ptr->stamp = stamp;
            x_non_bias_ptr->acc   = acc_xw_non_bias_;
            kf_x_->imuQuPush(x_non_bias_ptr);

            //kf_y_->prediction((double)acc_yw_non_bias_, stamp);
            aerial_robot_base::ImuQuPtr y_non_bias_ptr
              = boost::shared_non_bias_ptr<aerial_robot_base::ImuQu> ( new aerial_robot_base::ImuQu() );
            y_non_bias_ptr->stamp = stamp;
            y_non_bias_ptr->acc   = acc_yw_non_bias_;
            kf_y_->imuQuPush(y_non_bias_ptr);

            //kf_z_->prediction((double)acc_zw_non_bias_, stamp);
            aerial_robot_base::ImuQuPtr z_non_bias_ptr
              = boost::shared_non_bias_ptr<aerial_robot_base::ImuQu> ( new aerial_robot_base::ImuQu() );
            z_non_bias_ptr->stamp = stamp;
            z_non_bias_ptr->acc   = acc_zw_non_bias_;
            kf_z_->imuQuPush(z_non_bias_ptr);

            //with bias
            //kfb_x_->prediction((double)acc_xw_, stamp);
            aerial_robot_base::ImuQuPtr x_bias_ptr
              = boost::shared_ptr<aerial_robot_base::ImuQu> ( new aerial_robot_base::ImuQu() );
            x_bias_ptr->stamp = stamp;
            x_bias_ptr->acc   = acc_xw_;
            kfb_x_->imuQuPush(x_bias_ptr);

            //kfb_y_->prediction((double)acc_yw_, stamp);
            aerial_robot_base::ImuQuPtr y_bias_ptr
              = boost::shared_ptr<aerial_robot_base::ImuQu> ( new aerial_robot_base::ImuQu() );
            y_bias_ptr->stamp = stamp;
            y_bias_ptr->acc   = acc_yw_;
            kfb_y_->imuQuPush(y_bias_ptr);

            //kfb_z_->prediction((double)acc_zw_, stamp);
            aerial_robot_base::ImuQuPtr z_bias_ptr
              = boost::shared_ptr<aerial_robot_base::ImuQu> ( new aerial_robot_base::ImuQu() );
            z_bias_ptr->stamp = stamp;
            z_bias_ptr->acc   = acc_zw_;
            kfb_z_->imuQuPush(z_bias_ptr);

            //optical without accurate time stamp
            kf_x_opt_->prediction((double)acc_xi_ - acc_x_bias_, stamp);
            kf_y_opt_->prediction((double)acc_yi_ - acc_y_bias_, stamp);
            kf_z_opt_->prediction((double)acc_zw_non_bias_, stamp);
            kfb_x_opt_->prediction((double)acc_xi_, stamp);
            kfb_y_opt_->prediction((double)acc_yi_, stamp);
            kfb_z_opt_->prediction((double)acc_zw_, stamp);

          }

        if(kalman_filter_debug_)
          {
            if(kalman_filter_axis_ == 0)
              { //x axis
                kf1_->prediction((double)acc_xw_, stamp);
                kf2_->prediction((double)acc_xw_, stamp);
              }
            else if(kalman_filter_axis_ == 1)
              { //y axis
                kf1_->prediction((double)acc_yw_, stamp);
                kf2_->prediction((double)acc_yw_, stamp);
              }
            else if(kalman_filter_axis_ == 3)
              { //simulation about 
                aerial_robot_base::ImuQuPtr imu_bias_qu_msg_ptr_
                  = boost::shared_ptr<aerial_robot_base::ImuQu> ( new aerial_robot_base::ImuQu() );
                imu_bias_qu_msg_ptr->stamp = stamp;
                imu_bias_qu_msg_ptr->acc   = accXw;
                kf1_->imuQuPush(imu_bias_qu_msg_ptr);
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

    if(kalman_filter_flag_)
      {
        imu_data.position.x = kf_x_->getEstimatePos();
        imu_data.position.y = kf_y_->getEstimatePos();
        imu_data.position.z = kf_z_->getEstimatePos();

        imu_data.velocity.x = kf_x_->getEstimateVel();
        imu_data.velocity.y = kf_y_->getEstimateVel();
        imu_data.velocity.z = kf_z_->getEstimateVel();
      }

    imu_pub_.publish(imu_data);
  }

  void rosParamInit(ros::NodeHandle nh)
  {
    nh.param("acc_scale", acc_scale_, 9.797 / 512.0);
    printf(" acc scale is %f\n", acc_scale_);
    nh.param("gyro_scale", gyro_scale_, (2279 * M_PI)/((32767.0 / 4.0f ) * 180.0));
    printf(" gyro scale is %f\n", gyro_scale_);
    nh.param("mag_scale", mag_scale_, 1200 / 32768.0);
    printf(" mag scale is %f\n", mag_scale_);
  }

};

#endif





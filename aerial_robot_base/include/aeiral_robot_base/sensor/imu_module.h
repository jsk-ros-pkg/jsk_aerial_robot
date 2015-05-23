#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <jsk_quadcopter/ImuDebug.h>
#include <jsk_quadcopter/ImuSim.h>
#include <jsk_quadcopter_common/KduinoImu.h>
#include <jsk_quadcopter_common/KduinoSimpleImu.h>
#include <jsk_quadcopter/state_estimation.h>
#include <jsk_quadcopter/kalman_filter.h>
#include <jsk_quadcopter/digital_filter.h>

//for kduino (mpu 9150)
#define ACC_SCALE  9.797 / 512.0
#define GYRO_SCALE (2279 * M_PI)/((32767.0 / 4.0f ) * 180.0)
#define MAG_SCALE  1200 / 32768.0


class ImuData
{
 public:
 ImuData(ros::NodeHandle nh,
         ros::NodeHandle nh_private,
         Estimator* state_estimator,
         bool kalman_filter_flag,
         KalmanFilterImuLaser *kf_x,
         KalmanFilterImuLaser *kf_y,
         KalmanFilterImuLaser *kf_z,
         KalmanFilterImuLaserBias *kfb_x,
         KalmanFilterImuLaserBias *kfb_y,
         KalmanFilterImuLaserBias *kfb_z,
         KalmanFilterImuLaser *kf_x_opt,
         KalmanFilterImuLaser *kf_y_opt,
         KalmanFilterImuLaser *kf_z_opt,
         KalmanFilterImuLaserBias *kfb_x_opt,
         KalmanFilterImuLaserBias *kfb_y_opt,
         KalmanFilterImuLaserBias *kfb_z_opt,
         bool kalman_filter_debug,
         int kalman_filter_axis,
         KalmanFilterImuLaserBias *kf1,
         KalmanFilterImuLaserBias *kf2,
         FirFilter *filter_acc_x,
         FirFilter *filter_acc_y,
         FirFilter *filter_acc_z,
         bool simulation_flag)  
   : imuDataNodeHandle_(nh, "imu"),
    imuDataNodeHandlePrivate_(nh_private, "imu")
    {

      imuPub_ = imuDataNodeHandle_.advertise<jsk_quadcopter::ImuDebug>("debug", 2); 
      imuSub_ = imuDataNodeHandle_.subscribe<jsk_quadcopter_common::KduinoImu>("kduino/imu", 2, boost::bind(&ImuData::kduinoImuCallback, this, _1, state_estimator)); 
      imuSimpleSub_ = imuDataNodeHandle_.subscribe<jsk_quadcopter_common::KduinoSimpleImu>("kduino/simple_imu", 2, boost::bind(&ImuData::kduinoSimpleImuCallback, this, _1, state_estimator)); 


      simulationFlag = simulation_flag;
      kalmanFilterFlag = kalman_filter_flag;

      kfXImu_ = kf_x;
      kfYImu_ = kf_y;
      kfZImu_ = kf_z;
      kfbXImu_ = kfb_x;
      kfbYImu_ = kfb_y;
      kfbZImu_ = kfb_z;
  
      kfXOpt_ = kf_x_opt;
      kfYOpt_ = kf_y_opt;
      kfZOpt_ = kf_z_opt;
      kfbXOpt_ = kfb_x_opt;
      kfbYOpt_ = kfb_y_opt;
      kfbZOpt_ = kfb_z_opt;

      kalmanFilterDebug = kalman_filter_debug;
      kalmanFilterAxis  = kalman_filter_axis;
      kf1_ = kf1;
      kf2_ = kf2;

      filterAccXImu_ = filter_acc_x;
      filterAccYImu_ = filter_acc_y;
      filterAccZImu_ = filter_acc_z;


      //??????
      kalmanFilterBoard = KDUINO_BOARD;
      printf("kalman filter board is kduino\n");

      accelerationX = 0, accelerationY = 0, accelerationZ = 0,
      gyroX = 0, gyroY = 0, gyroZ = 0,
      magX = 0, magY = 0, magZ = 0,

      accXc = 0;
      accYc = 0;
      accXw = 0;
      accYw = 0;
      accZw = 0;
      accXwNonBias = 0;
      accYwNonBias = 0;
      accZwNonBias = 0;

      accXBias = 0;
      accYBias = 0;
      accZBias = 0;

      pitch = 0;
      roll = 0;
      yaw = 0;
      height = 0;

    }

  ~ImuData ()
    {
    };



  void setPitchValue(float pitch_value)
  {
    pitch = pitch_value;
  }

  void setRollValue(float roll_value)
  {
    roll = roll_value;
  }

  void setYawImuValue(float yaw_value)
  {
    yaw = yaw_value;
  }

  void setZImuValue(float z_value)
  {
    height = z_value;
  }

  float getPitchValue()
  {
    return pitch;
  }

  float getRollValue()
  {
    return roll;
  }

  float getYawValue()
  {
    return yaw;
  }

  float getAccXbValue()
  {
    return accelerationX; 
  }

  float getAccYbValue()
  {
    return accelerationY;
  }

  float getAccZbValue()
  {
    return accelerationZ;
  }
  void setKalmanFilterBoardToAsctec()
  {
    kalmanFilterBoard = ASCTEC_BOARD;
    printf("change kalman filter board from kduino to asctec\n");
  }

  const static float G = 9.797;
  const static int CALIB_COUNT = 200;
  const static int KDUINO_BOARD = 0;
  const static int ASCTEC_BOARD = 1;


 private:
  ros::NodeHandle imuDataNodeHandle_;
  ros::NodeHandle imuDataNodeHandlePrivate_;
  ros::Publisher  imuPub_;
  ros::Subscriber  imuSub_;
  ros::Subscriber  imuSimpleSub_;

  bool simulationFlag;

  bool kalmanFilterFlag;
  KalmanFilterImuLaser *kfXImu_;
  KalmanFilterImuLaser *kfYImu_;
  KalmanFilterImuLaser *kfZImu_;
  KalmanFilterImuLaserBias *kfbXImu_;
  KalmanFilterImuLaserBias *kfbYImu_;
  KalmanFilterImuLaserBias *kfbZImu_;

  KalmanFilterImuLaser *kfXOpt_;
  KalmanFilterImuLaser *kfYOpt_;
  KalmanFilterImuLaser *kfZOpt_;
  KalmanFilterImuLaserBias *kfbXOpt_;
  KalmanFilterImuLaserBias *kfbYOpt_;
  KalmanFilterImuLaserBias *kfbZOpt_;

  bool kalmanFilterDebug;
  int kalmanFilterAxis;
  KalmanFilterImuLaserBias *kf1_;
  KalmanFilterImuLaserBias *kf2_;

  //FIR filter for acceleration
  FirFilter *filterAccXImu_;
  FirFilter *filterAccYImu_;
  FirFilter *filterAccZImu_;

  int kalmanFilterBoard;

  float accelerationX, accelerationY, accelerationZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;


  float pitch;  //pitch angle
  float roll;    //roll angle
  float yaw;    //yaw angle

  float filteredAccelerationX; 
  float filteredAccelerationY; 
  float filteredAccelerationZ; 

    //*** trans_acc with intermediate frame between world frame and boady frame
  float accXc;
  float accYc;

  //***  world frame
  float accXw, accXwNonBias;
  float accYw, accYwNonBias;
  float accZw, accZwNonBias;

  //*** world coordinate
  float accX;
  float accY;
  float accZ;
  float height;

  float vBat;   //*** battery



  double accXBias;
  double accYBias;
  double accZBias;


  void kduinoImuCallback(const jsk_quadcopter_common::KduinoImuConstPtr& imu_msg, Estimator* state_estimator)
  {
    ros::Time imuTimeStamp = imu_msg->stamp;


    roll  = M_PI * imu_msg->angle[0] / 10.0 / 180.0; //raw data is 10 times
    pitch = M_PI * imu_msg->angle[1] / 10.0 / 180.0; //raw data is 10 times
    yaw   = M_PI * imu_msg->angle[2] / 180.0;

    accelerationX = imu_msg->accData[0] * ACC_SCALE;
    accelerationY = imu_msg->accData[1] * ACC_SCALE;
    accelerationZ = imu_msg->accData[2] * ACC_SCALE;
    gyroX = imu_msg->gyroData[0] * GYRO_SCALE;
    gyroY = imu_msg->gyroData[1] * GYRO_SCALE;
    gyroZ = imu_msg->gyroData[2] * GYRO_SCALE;

#if 0 
    magX = imu_msg->magData[0] * MAG_SCALE;
    magY = imu_msg->magData[1] * MAG_SCALE;
    magZ = imu_msg->magData[2] * MAG_SCALE;
#else //debug
    magX = imu_msg->magData[0] * ACC_SCALE;
    magY = imu_msg->magData[1] * ACC_SCALE;
    magZ = imu_msg->magData[2] * ACC_SCALE + (-G);
#endif

    //* height
    height = imu_msg->altitude / 100.0;  //cm

    imuDataConverter(imuTimeStamp, state_estimator);
  }

  void kduinoSimpleImuCallback(const jsk_quadcopter_common::KduinoSimpleImuConstPtr& imu_msg, Estimator* state_estimator)
  {
    ros::Time imuTimeStamp = imu_msg->stamp;

    roll  = M_PI * imu_msg->angle[0] / 10.0 / 180.0; //raw data is 10 times
    pitch = M_PI * imu_msg->angle[1] / 10.0 / 180.0; //raw data is 10 times
    yaw   = M_PI * imu_msg->angle[2] / 180.0;

    accelerationX = imu_msg->accData[0] * ACC_SCALE;
    accelerationY = imu_msg->accData[1] * ACC_SCALE;
    accelerationZ = imu_msg->accData[2] * ACC_SCALE;

    imuDataConverter(imuTimeStamp, state_estimator);
  }

  void imuDataConverter(ros::Time topicTimeStamp, Estimator* state_estimator)
  {
    static int biasCalib = 0;

    //* calculate accTran
#if 0 // use x,y for factor4 and z for factor3
    accXc = (accelerationX) * cos(pitch) + 
      (accelerationY) * sin(pitch) * sin(roll) + 
      (accelerationZ) * sin(pitch) * cos(roll);
    accYc = (accelerationY) * cos(roll) - (accelerationZ) * sin(roll);
    accZw = (accelerationX) * (-sin(pitch)) + 
      (accelerationY) * cos(pitch) * sin(roll) + 
      (accelerationZ) * cos(pitch) * cos(roll) + (-G);
#else  // use approximation
    accXc =  (accelerationZ) * sin(pitch) * cos(roll);
    accYc =  - (accelerationZ) * sin(roll);
    accZw = (accelerationZ) * cos(pitch) * cos(roll) + (-G);
#endif

    //bais calibration
    if(biasCalib < CALIB_COUNT)
      {
        biasCalib ++;

        //acc bias
        accXBias += accXc;
        accYBias += accYc;
        accZBias += accZw;

        if(biasCalib == CALIB_COUNT)
          {
            accXBias /= CALIB_COUNT;
            accYBias /= CALIB_COUNT;
            accZBias /= CALIB_COUNT;
            ROS_WARN("accX bias is %f, accY bias is %f, accZ bias is %f", accXBias, accYBias, accZBias);

            kfXImu_->setInputStartFlag();
            kfYImu_->setInputStartFlag();
            kfZImu_->setInputStartFlag();

            //for optical flow
            kfXOpt_->setInputStartFlag();
            kfYOpt_->setInputStartFlag();
            kfZOpt_->setInputStartFlag();


            //for bais mode
            kfbXImu_->setInitImuBias(accXBias);
            kfbYImu_->setInitImuBias(accYBias);
            kfbZImu_->setInitImuBias(accZBias);

            //for optical flow
            kfbXOpt_->setInitImuBias(accXBias);
            kfbYOpt_->setInitImuBias(accYBias);
            kfbZOpt_->setInitImuBias(accZBias);

            if(kalmanFilterAxis == 0)
              {
                kf1_->setInitImuBias(accXBias);
                kf2_->setInitImuBias(accXBias);
              }
            if(kalmanFilterAxis == 1)
              {
                kf1_->setInitImuBias(accYBias);
                kf2_->setInitImuBias(accYBias);
              }
          }
      }

    float yaw2 = state_estimator->getStatePsiBoard();

    if(biasCalib == CALIB_COUNT)
      {
        accXw = cos(yaw2) * accXc - sin(yaw2) * accYc;
        accYw = sin(yaw2) * accXc + cos(yaw2) * accYc;

        accXwNonBias = cos(yaw2) * (accXc - accXBias) 
          - sin(yaw2) * (accYc -accYBias);
        accYwNonBias = sin(yaw2) * (accXc - accXBias) 
          + cos(yaw2) * (accYc -accYBias);
        accZwNonBias = accZw - accZBias;

        if(kalmanFilterFlag)
          {
            //kfXImu_->prediction((double)accXwNonBias, topicTimeStamp);
            jsk_quadcopter::ImuQuPtr imuXAxisNonBiasQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuXAxisNonBiasQuMsgPtr_->stamp = topicTimeStamp;
            imuXAxisNonBiasQuMsgPtr_->acc   = accXwNonBias;
            kfXImu_->imuQuPush(imuXAxisNonBiasQuMsgPtr_);

            //kfYImu_->prediction((double)accYwNonBias, topicTimeStamp);
            jsk_quadcopter::ImuQuPtr imuYAxisNonBiasQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuYAxisNonBiasQuMsgPtr_->stamp = topicTimeStamp;
            imuYAxisNonBiasQuMsgPtr_->acc   = accYwNonBias;
            kfYImu_->imuQuPush(imuYAxisNonBiasQuMsgPtr_);

            //kfZImu_->prediction((double)accZwNonBias, topicTimeStamp);
            jsk_quadcopter::ImuQuPtr imuZAxisNonBiasQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuZAxisNonBiasQuMsgPtr_->stamp = topicTimeStamp;
            imuZAxisNonBiasQuMsgPtr_->acc   = accZwNonBias;
            kfZImu_->imuQuPush(imuZAxisNonBiasQuMsgPtr_);

            //with bias
            //kfbXImu_->prediction((double)accXwSlamYaw, topicTimeStamp);
            jsk_quadcopter::ImuQuPtr imuXAxisBiasQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuXAxisBiasQuMsgPtr_->stamp = topicTimeStamp;
            imuXAxisBiasQuMsgPtr_->acc   = accXw;
            kfbXImu_->imuQuPush(imuXAxisBiasQuMsgPtr_);

            //kfbYImu_->prediction((double)accYwSlamYaw, topicTimeStamp);
            jsk_quadcopter::ImuQuPtr imuYAxisBiasQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuYAxisBiasQuMsgPtr_->stamp = topicTimeStamp;
            imuYAxisBiasQuMsgPtr_->acc   = accYw;
            kfbYImu_->imuQuPush(imuYAxisBiasQuMsgPtr_);

            //kfbZImu_->prediction((double)accZw2, topicTimeStamp);
            jsk_quadcopter::ImuQuPtr imuZAxisBiasQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuZAxisBiasQuMsgPtr_->stamp = topicTimeStamp;
            imuZAxisBiasQuMsgPtr_->acc   = accZw;
            kfbZImu_->imuQuPush(imuZAxisBiasQuMsgPtr_);

#if 0 //optical with accurate time stamp

            jsk_quadcopter::ImuQuPtr imuXAxisNonBiasOptQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuXAxisNonBiasOptQuMsgPtr_->stamp = topicTimeStamp;
            imuXAxisNonBiasOptQuMsgPtr_->acc   = accXc - accXBias;
            kfXOpt_->imuQuPush(imuXAxisNonBiasOptQuMsgPtr_);

            jsk_quadcopter::ImuQuPtr imuYAxisNonBiasOptQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuYAxisNonBiasOptQuMsgPtr_->stamp = topicTimeStamp;
            imuYAxisNonBiasOptQuMsgPtr_->acc   = accYc - accYBias;
            kfYOpt_->imuQuPush(imuYAxisNonBiasOptQuMsgPtr_);

            jsk_quadcopter::ImuQuPtr imuZAxisNonBiasOptQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuZAxisNonBiasOptQuMsgPtr_->stamp = topicTimeStamp;
            imuZAxisNonBiasOptQuMsgPtr_->acc   = accZwNonBias;

            //with bias

            jsk_quadcopter::ImuQuPtr imuXAxisBiasOptQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuXAxisBiasOptQuMsgPtr_->stamp = topicTimeStamp;
            imuXAxisBiasOptQuMsgPtr_->acc   = accXc;
            kfbXOpt_->imuQuPush(imuXAxisBiasQuMsgPtr_);

            jsk_quadcopter::ImuQuPtr imuYAxisBiasOptQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuYAxisBiasOptQuMsgPtr_->stamp = topicTimeStamp;
            imuYAxisBiasOptQuMsgPtr_->acc   = accYc;
            kfbYOpt_->imuQuPush(imuYAxisBiasOptQuMsgPtr_);

            jsk_quadcopter::ImuQuPtr imuZAxisBiasOptQuMsgPtr_
              = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
            imuZAxisBiasOptQuMsgPtr_->stamp = topicTimeStamp;
            imuZAxisBiasOptQuMsgPtr_->acc   = accZw;
            kfbZOpt_->imuQuPush(imuZAxisBiasOptQuMsgPtr_);

#else //optical without accurate time stamp
            kfXOpt_->prediction((double)accXc - accXBias, topicTimeStamp);
            kfYOpt_->prediction((double)accYc - accYBias, topicTimeStamp);
            kfZOpt_->prediction((double)accZwNonBias, topicTimeStamp);
            kfbXOpt_->prediction((double)accXc, topicTimeStamp);
            kfbYOpt_->prediction((double)accYc, topicTimeStamp);
            kfbZOpt_->prediction((double)accZw, topicTimeStamp);
#endif
          }

        if(kalmanFilterDebug)
          {
            if(kalmanFilterAxis == 0)
              { //x axis
                kf1_->prediction((double)accXw, topicTimeStamp);
                kf2_->prediction((double)accXw, topicTimeStamp);
              }
            else if(kalmanFilterAxis == 1)
              { //y axis
                kf1_->prediction((double)accYw, topicTimeStamp);
                kf2_->prediction((double)accYw, topicTimeStamp);
              }
            else if(kalmanFilterAxis == 3)
              { //simulation about 
                jsk_quadcopter::ImuQuPtr imuBiasQuMsgPtr_
                  = boost::shared_ptr<jsk_quadcopter::ImuQu> ( new jsk_quadcopter::ImuQu() );
                imuBiasQuMsgPtr_->stamp = topicTimeStamp;
                imuBiasQuMsgPtr_->acc   = accXw;
                kf1_->imuQuPush(imuBiasQuMsgPtr_);
              }
          }
        publishImuData(topicTimeStamp, imuPub_);
      }
  }

  void publishImuData(ros::Time topicTimeStamp, ros::Publisher imu_pub)
  {
    jsk_quadcopter::ImuDebug imuDebug_;
    imuDebug_.header.stamp = topicTimeStamp;

    imuDebug_.height = height;

    imuDebug_.angles.x = roll;
    imuDebug_.angles.y = pitch;
    imuDebug_.angles.z = yaw;

    imuDebug_.accelerometer.x = accelerationX;
    imuDebug_.accelerometer.y = accelerationY;
    imuDebug_.accelerometer.z = accelerationZ;

    imuDebug_.gyrometer.x = gyroX;
    imuDebug_.gyrometer.y = gyroY;
    imuDebug_.gyrometer.z = gyroZ;

    imuDebug_.magnetometer.x = magX;
    imuDebug_.magnetometer.y = magY;
    imuDebug_.magnetometer.z = magZ;


    imuDebug_.acceleration_body_frame.x = accXc;
    imuDebug_.acceleration_body_frame.y = accYc; 
    imuDebug_.acceleration_body_frame.z = accZw;

    imuDebug_.acceleration_world_frame.x = accXw;
    imuDebug_.acceleration_world_frame.y = accYw; 
    imuDebug_.acceleration_world_frame.z = accZw;

    imuDebug_.acceleration_non_bias_world_frame.x = accXwNonBias;
    imuDebug_.acceleration_non_bias_world_frame.y = accYwNonBias;
    imuDebug_.acceleration_non_bias_world_frame.z = accZwNonBias;

    if(kalmanFilterFlag)
      {
        imuDebug_.preZPos = kfZImu_->getEstimatePos();
        imuDebug_.preZVel = kfZImu_->getEstimateVel();
        imuDebug_.preXPos = kfXImu_->getEstimatePos();
        imuDebug_.preXVel = kfXImu_->getEstimateVel();
        imuDebug_.preYPos = kfYImu_->getEstimatePos();
        imuDebug_.preYVel = kfYImu_->getEstimateVel();
      }

    imu_pub.publish(imuDebug_);
  }

};

#endif





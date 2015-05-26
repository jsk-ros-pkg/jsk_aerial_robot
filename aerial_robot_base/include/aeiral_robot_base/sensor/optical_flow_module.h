#ifndef OPTICAL_FLOW_MODULE_H
#define OPTICAL_FLOW_MODULE_H

#include <ros/ros.h>
#include <aerial_robot_base/state_estimation.h>
//* filter
#include <aerial_robot_base/kalman_filter.h>
#include <aerial_robot_base/digital_filter.h>

#include <aerial_robot_base/States.h>
#include <aerial_robot_base/OpticalFlow.h>
#include <std_msgs/Int32.h>


class OpticalFlowData
{
 public:

 OpticalFlowData(ros::NodeHandle nh,
                 ros::NodeHandle nh_private,
                 Estimator* state_estimator,
                 bool kalman_filter_flag,
                 KalmanFilterImuLaser *kf_x, 
                 KalmanFilterImuLaser *kf_y,
                 KalmanFilterImuLaser *kf_z,
                 KalmanFilterImuLaserBias *kfb_x, 
                 KalmanFilterImuLaserBias *kfb_y,
                 KalmanFilterImuLaserBias *kfb_z)
   : nh_(nh, "optical_flow"),
    nhp_(nh_private, "optical_flow")
    {
      optical_flow_pub_ = nh_.advertise<aeiral_robot_base::OpticalFlowDaya>("data",10);
      optical_flow_sub_ = nh_.subscribe<px_comm::OpticalFlow>("opt_flow", 1, &OpticalFlowData::opticalFlowCallback, this, ros::TransportHints().tcpNoDelay());

      rosParamInit(nhp_);

      kalman_filter_flag = kalman_filter_flag;
      kf_x_ = kf_x; kf_y_ = kf_y; kf_z_ = kf_z;
      kfb_x_ = kfb_x; kfb_y_ = kfb_y; kfb_z_ = kfb_z;

      setRocketStartFlag();

      raw_pos_z_ = 0;
      pos_z_ = 0;
      raw_vel_z_ = 0;
      vel_z_ = 0;

      raw_vel_x_ = 0;
      filtered_vel_x_ = 0;
      pos_x_ = 0;
      raw_vel_y_ = 0;
      filtered_vel_y_ = 0;
      pos_y_ = 0;
    }

  ~OpticalFlowData()
    {
    }


  inline float getPosX()  { return 0; }
  inline float getPosY()  { return 0; }
  inline float getPosZ()  { return raw_pos_z_; }

  inline float getFilteredVelX() {  return filtered_vel_x_;  }
  inline float getFilteredVelY() {  return filtered_vel_y_;  }

  inline float getRawVelX() {  return raw_vel_x_; }
  inline float getRawVelY() {  return raw_vel_y_; }
  inline float getRawVelZ() {  return raw_vel_z_; }

  inline bool  getRocketStartFlag() { return rocket_start_flag_; }

  void  setRocketStartFlag()
  {
    if(use_rocket_start_)
      {
        rocket_start_flag_ = true;
        bool stop_flag = false;
        kf_x_->setMeasureStartFlag(stop_flag);
        kf_y_->setMeasureStartFlag(stop_flag);
        kf_z_->setMeasureStartFlag(stop_flag);
      
        kfb_x_->setMeasureStartFlag(stop_flag);
        kfb_y_->setMeasureStartFlag(stop_flag);
        kfb_z_->setMeasureStartFlag(stop_flag);
        ROS_ERROR(" set measure start flag to false");
      }
    else
      {
        rocket_start_flag_ = false;
      }
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher optical_flow_pub_;
  ros::Subscriber optical_flow_sub_;
  ros::Time optical_flow_stamp_;

  Estimator* state_estimator_;

  bool   useRocket_start_;
  bool   rocket_start_flag;
  double rocket_start_upper_thre_;
  double rocket_start_lower_thre_;
  double rocket_start_vel_;

  double x_axis_direction_;
  double y_axis_direction_;

  bool kalman_filter_flag_;
  KalmanFilterImuLaser *kf_x_;
  KalmanFilterImuLaser *kf_y_;
  KalmanFilterImuLaser *kf_z_;

  KalmanFilterImuLaserBias *kfb_x_;
  KalmanFilterImuLaserBias *kfb_y_;
  KalmanFilterImuLaserBias *kfb_z_;

  float raw_pos_z_;
  float pos_z_;
  float raw_vel_z_;
  float vel_z_;

  float raw_vel_x_;
  float filtered_vel_x_;

  float raw_vel_y_;
  float filtered_vel_y_;


  void opticalFlowCallback(const px_comm::OpticalFlowConstPtr & optical_flow_msg)
  {
    static int cnt = 0;
    static int CNT = 1;
    static float prev_raw_pos_z;
    static bool first_flag = true;
    static double previous_secs;
    double current_secs = optical_flow_msg->header.stamp.toSec();

    //**** 高さ方向情報の更新
    raw_pos_z_ = optical_flow_msg->ground_distance;

    // rocket start mode
    if( raw_pos_z_ < rocket_start_upper_thre_ &&
        raw_pos_z_ > rocket_start_lower_thre_ &&
	prev_raw_pos_z < rocket_start_lower_thre_ &&
	prev_raw_pos_z > (rocket_start_lower_thre_ - 0.1) &&
        rocket_start_flag_)
      {//pose init
	//TODO: start flag fresh arm, or use air pressure => refined

	ROS_INFO("prev_raw_pos_z : %f", prev_raw_pos_z);
	ROS_ERROR("start rocket!");

        kf_x_->setInitState(0, optical_flow_msg->flow_x /1000.0);
        kfb_x_->setInitState(0, optical_flow_msg->flow_x /1000.0);

        kf_y_->setInitState(0, -optical_flow_msg->flow_y /1000.0);
        kfb_y_->setInitState(0, -optical_flow_msg->flow_y /1000.0);

        kf_z_->setInitState(optical_flow_msg->ground_distance, rocket_start_vel_);
        kfb_z_->setInitState(optical_flow_msg->ground_distance, rocket_start_vel_);

        bool start_flag = true;

        kf_x_->setMeasureStartFlag(start_flag);
        kf_y_->setMeasureStartFlag(start_flag);
        kf_z_->setMeasureStartFlag(start_flag);

        kfb_x_->setMeasureStartFlag(start_flag);
        kfb_y_->setMeasureStartFlag(start_flag);
        kfb_z_->setMeasureStartFlag(start_flag);

        rocket_start_flag_ = false;
      }

    if(first_flag)
      {
        first_flag = false;
      }
    else
      {
        if(!rocket_start_flag_)
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
                cnt++;
                if(cnt == CNT) //50 Hz !!!!!!!!!!!!!!!!!!!!!
                  {
                    cnt = 0; 
                    // optical without accurate timestamp
                    if(optical_flow_msg->quality == 0 || raw_vel_x_ == 0 || raw_pos_z_ > 2.5)
                      { // remove the raw_vel_x case is not good !!
                        kf_x_->correctionOnlyVelocity(filtered_vel_x_, optical_flow_msg->header.stamp);  //velocity
                        kfb_x_->correctionOnlyVelocity(filtered_vel_x_, optical_flow_msg->header.stamp); //velocity
                      }
                    else  
                      {
                        kf_x_->correctionOnlyVelocity(raw_vel_x_, optical_flow_msg->header.stamp);  //velocity
                        kfb_x_->correctionOnlyVelocity(raw_vel_x_, optical_flow_msg->header.stamp); //velocity
                      }
                    if(optical_flow_msg->quality == 0 || raw_vel_y_ == 0 || raw_pos_z > 2.5)
                      { // remove the raw_vel_y case is not good !!
                        kf_y_->correctionOnlyVelocity(filtered_vel_y_, optical_flow_msg->header.stamp); //velocity
                        kfb_y_->correctionOnlyVelocity(filtered_vel_y_, optical_flow_msg->header.stamp); //velocity
                      }
                    else
                      {
                        kf_y_->correctionOnlyVelocity(raw_vel_y_, optical_flow_msg->header.stamp); //velocity
                        kfb_y_->correctionOnlyVelocity(raw_vel_y_, optical_flow_msg->header.stamp); //velocity
                      }
                  }
                
                if(raw_pos_z_ != prev_raw_pos_z && raw_pos_z_ < 2.5) //100Hz
                  {
                    kfb_z_->correction(raw_pos_z, optical_flow_msg->header.stamp);
                    kf_z_->correction(raw_pos_z, optical_flow_msg->header.stamp);
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
                x_state.kf_pos = kf_x_->getEstimatePos();
                x_state.kf_vel = kf_x_->getEstimateVel();
                x_state.kfb_pos = kfb_x_->getEstimatePos();
                x_state.kfb_vel = kfb_x_->getEstimateVel();
                x_state.kfb_bias = kfb_x_->getEstimateBias();

                y_state.kf_pos = kf_y_->getEstimatePos();
                y_state.kf_vel = kf_y_->getEstimateVel();
                y_state.kfb_pos = kfb_y_->getEstimatePos();
                y_state.kfb_vel = kfb_y_->getEstimateVel();
                y_state.kfb_bias = kfb_y_->getEstimateBias();

                z_state.kf_pos = kf_z_->getEstimatePos();
                z_state.kf_vel = kf_z_->getEstimateVel();
                z_state.kfb_pos = kfb_z_->getEstimatePos();
                z_state.kfb_vel = kfb_z_->getEstimateVel();
                z_state.kfb_bias = kfb_z_->getEstimateBias();
              }

            opt_data.states.push_back(x_state);
            opt_data.states.push_back(y_state);
            opt_data.states.push_back(z_state);

            optical_flow_pub_.publish(opt_data);
          }
      }

    //更新
    previous_secs = current_secs;
    prev_raw_pos_z = raw_pos_z_;
  }


  void rosParamInit(ros::NodeHandle nh)
  {
    //name space in /quadcopter/navigator/...
    ros::NodeHandle navi_nh("~navigator");
    if (!navi_nh.getParam ("use_rocket_start", use_rocket_start_))
      use_rocket_start_ = false;
    printf("%s: use_rocket_start is %s\n", navi_nh.getNamespace().c_str(), use_rocket_start_ ? ("true") : ("false"));
    if (!navi_nh.getParam ("rocket_start_upper_thre", rocket_start_upper_thre_))
      rocket_start_upper_thre_ = 0;
    printf("%s: rocket_start_upper_thre_ is %.3f\n", navi_nh.getNamespace().c_str(), rocket_start_upper_thre_);
    if (!navi_nh.getParam ("rocket_start_lower_thre", rocket_start_lower_thre_))
      rocket_start_lower_thre_ = 0;
    printf("%s: rocket_start_lower_thre_ is %.3f\n", navi_nh.getNamespace().c_str(), rocket_start_lower_thre_);
    if (!navi_nh.getParam ("rocket_start_vel", rocket_start_vel_))
      rocket_start_vel_ = 0;
    printf("%s: rocket_start_vel_ is %.3f\n", navi_nh.getNamespace().c_str(), rocket_start_vel_);

    std::string ns = nh.getNamespace();
    if (!nh.getParam ("x_axis_direction", x_axis_direction_))
      x_axis_direction_ = 1.0;
    printf("%s: x_axis_direction_ is %.3f\n", ns.c_str(), x_axis_direction_);

    if (!nh.getParam ("y_axis_direction", y_axis_direction_))
      y_axis_direction_ = -1.0; //-1 is default
    printf("%s: y_axisDirection_ is %.3f\n", ns.c_str(), y_axis_direction_);

};


#endif




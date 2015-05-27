#ifndef AR_MARKER_DATA_H
#define AR_MARKER_DATA_H

//* ros
#include <ros/ros.h>

#include <aerial_robot_base/ArMarkerData.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_base/StateKalmanFilterConfig.h>
#include <aerial_robot_msgs/DynamicReconfigureLevels.h>
#include <tf/transform_broadcaster.h>
//* filter
#include <aerial_robot_base/kalman_filter.h>
#include <aerial_robot_base/digital_filter.h>

//* for debug
#include <iostream>
//* state
#include <aerial_robot_base/basic_state_estimation.h>

class ARMarkerData
{
 public:
  ARMarkerData (ros::NodeHandle nh);
  ARMarkerData (ros::NodeHandle nh,
                bool kalman_filter_flag,
                KalmanFilterPosVelAcc *kf_x, 
                KalmanFilterPosVelAcc *kf_y,
                KalmanFilterPosVelAcc *kf_z);
  ~ARMarkerData ();

  void setPosXValue(float pos_x_value);
  void setPosYValue(float pos_y_value);
  void setPosZValue(float pos_z_value);
  void setThetaValue(float theta_value);
  void setPhyValue(float phy_value);
  void setPsiValue(float psi_value);

  float getPosXValue();
  float getPosYValue();
  float getPosZValue();
  float getThetaValue();
  float getPhyValue();
  float getPsiValue();

  void setRawPosXValue(float raw_pos_x_value);
  void setRawPosYValue(float raw_pos_y_value);
  void setRawPosZValue(float raw_pos_z_value);
  void setRawThetaValue(float raw_theta_value);
  void setRawPhyValue(float raw_phy_value);
  void setRawPsiValue(float raw_psi_value);

  float getRawPosXValue();
  float getRawPosYValue();
  float getRawPosZValue();
  float getRawThetaValue();
  float getRawPhyValue();
  float getRawPsiValue();

  void setVelXValue(float vel_x_value);
  void setVelYValue(float vel_y_value);
  void setVelZValue(float vel_z_value);
  void setVelThetaValue(float vel_theta_value);
  void setVelPhyValue(float vel_phy_value);
  void setVelPsiValue(float vel_psi_value);

  float getVelXValue();
  float getVelYValue();
  float getVelZValue();
  float getVelThetaValue();
  float getVelPhyValue();
  float getVelPsiValue();

  void setRawVelXValue(float raw_vel_x_value);
  void setRawVelYValue(float raw_vel_y_value);
  void setRawVelZValue(float raw_vel_z_value);
  void setRawVelThetaValue(float raw_vel_theta_value);
  void setRawVelPhyValue(float raw_vel_phy_value);
  void setRawVelPsiValue(float raw_vel_psi_value);

  float getRawVelXValue();
  float getRawVelYValue();
  float getRawVelZValue();
  float getRawVelThetaValue();
  float getRawVelPhyValue();
  float getRawVelPsiValue();

  //** callback function
  //void arMarkerPoseStampedCallback(const ar_pose::ARMarkers &ar_marker_pose_msg);

  bool arMarkerKalmanFilterFlag_;
  KalmanFilterPosVelAcc *kf_x_;
  KalmanFilterPosVelAcc *kf_y_;
  KalmanFilterPosVelAcc *kf_z_;

  const static int CALIBCOUNT = 30;

 private:
  ros::NodeHandle nh_;
  ros::Publisher ar_pub_;
  ros::Subscriber ar_sub_;
  ros::Time stamp_;

  int calib_count_;

  tf::Transform init_ar_tf_;
  float init_ar_pose_x_;
  float init_ar_pose_y_;
  float init_ar_pose_z_;
  float init_ar_pose_theta_;
  float init_ar_pose_phy_;
  float init_ar_pose_psi_;


  float pos_x_;
  float pos_y_;
  float pos_z_;
  float theta_;
  float phy_;  
  float psi_; 
  
  float raw_pos_x_;
  float raw_pos_y_;
  float raw_pos_z_;
  float raw_theta_;
  float raw_phy_;
  float raw_psi_;

  float vel_x_;
  float vel_y_;
  float vel_z_;
  float vel_theta_; 
  float vel_phy_; 
  float vel_psi_; 

  float raw_vel_x_;
  float raw_vel_y_;
  float raw_vel_z_;
  float raw_vel_theta_;
  float raw_vel_phy_;
  float raw_vel_psi_;

};


#endif

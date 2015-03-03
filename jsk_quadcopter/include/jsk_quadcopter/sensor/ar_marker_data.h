#ifndef AR_MARKER_DATA_H
#define AR_MARKER_DATA_H

//* ros
#include <ros/ros.h>

#include <jsk_quadcopter/ArMarkerDebug.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <jsk_quadcopter/StateKalmanFilterConfig.h>
#include <jsk_quadcopter/DynamicReconfigureLevels.h>
#include <tf/transform_broadcaster.h>
//* filter
#include <jsk_quadcopter/kalman_filter.h>
#include <jsk_quadcopter/digital_filter.h>

//* for debug
#include <iostream>


//* state
#include <jsk_quadcopter/state_estimation.h>


class ARMarkerData
{
 public:
  ARMarkerData (ros::NodeHandle nh);
  ARMarkerData (ros::NodeHandle nh,
                bool slamKalmanFilterFlag,
                KalmanFilterImuLaserBias *kfb_x, 
                KalmanFilterImuLaserBias *kfb_y,
                KalmanFilterImuLaserBias *kfb_z);
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
  KalmanFilterImuLaserBias *kfbXARMarker_;
  KalmanFilterImuLaserBias *kfbYARMarker_;
  KalmanFilterImuLaserBias *kfbZARMarker_;

  const static int CALIBCOUNT = 30;

 private:
  ros::NodeHandle arMarkerDataNodeHandle_;
  ros::Publisher arMarkerPub_;
  ros::Subscriber arMarkerSub_;
  ros::Time slamStamp_;

  int calibCount;

  tf::Transform initARMarkerTF;
  float initARMarkerPoseX;
  float initARMarkerPoseY;
  float initARMarkerPoseZ;
  float initARMarkerPoseQ1;
  float initARMarkerPoseQ2;
  float initARMarkerPoseQ3;
  float initARMarkerPoseQ4;


  float initARMarkerPoseTheta;
  float initARMarkerPosePhy;
  float initARMarkerPosePsi;


  //R_Camera   = R_initARMarkerTF * R_ARMarkerTF_Inverse
  //Vec_Camera = R_ARMarkerTF_Inverse * (-Vec_ARMarkerTF) + Vec_Init_ARMarker
  //tf::Transform ARMarkerReTF;
  /* float ARMarkerPoseX; */
  /* float ARMarkerPoseY; */
  /* float ARMarkerPoseZ; */
  /* float ARMarkerPoseTheta; */
  /* float ARMarkerPosePhy; */
  /* float ARMarkerPosePsi; */

  float posX;
  float posY;
  float posZ;
  float theta;
  float phy;  
  float psi; //yaw angle
  
  float rawPosX;
  float rawPosY;
  float rawPosZ;
  float rawTheta;
  float rawPhy;
  float rawPsi;

  float velX;
  float velY;
  float velZ;
  float velTheta; 
  float velPhy; 
  float velPsi; 

  float rawVelX;
  float rawVelY;
  float rawVelZ;
  float rawVelTheta;
  float rawVelPhy;
  float rawVelPsi;

};


#endif

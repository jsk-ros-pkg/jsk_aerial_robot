#include "jsk_quadcopter/ar_marker_data.h"


ARMarkerData::ARMarkerData(ros::NodeHandle nh,
                           bool arMarkerKalmanFilterFlag,
                           KalmanFilterImuLaserBias *kfb_x, 
                           KalmanFilterImuLaserBias *kfb_y,
                           KalmanFilterImuLaserBias *kfb_z)
  : arMarkerDataNodeHandle_(nh, "arMarker")
{
#if 0
  arMarkerPub_ = arMarkerDataNodeHandle_.advertise<jsk_quadcopter::ARMarkerDebug>("debug", 10); 
   arMarkerSub_ = arMarkerDataNodeHandle_.subscribe("ar_pose_marker", 5, &ARMarkerData::arMarkerPoseStampedCallback, this, ros::TransportHints().tcpNoDelay());
#endif

  posX = 0;
  posY = 0;
  posZ = 0;
  theta = 0;
  phy = 0;
  psi = 0;

  rawPosX = 0;
  rawPosY = 0;
  rawPosZ = 0;
  rawTheta = 0;
  rawPhy = 0;
  rawPsi = 0;

  velX = 0;
  velY = 0;
  velZ = 0;
  velTheta = 0; 
  velPhy = 0; 
  velPsi = 0; 

  rawVelX = 0;
  rawVelY = 0;
  rawVelZ = 0;
  rawVelTheta = 0;
  rawVelPhy = 0;
  rawVelPsi = 0;

  arMarkerKalmanFilterFlag_ = arMarkerKalmanFilterFlag;
  kfbXARMarker_ = kfb_x; 
  kfbYARMarker_ = kfb_y;
  kfbZARMarker_ = kfb_z;

  initARMarkerTF.setIdentity();

  calibCount = CALIBCOUNT;

}

 ARMarkerData::~ARMarkerData()
 {
 }

 void ARMarkerData::setPosXValue(float pos_x_value)
 {
   posX = pos_x_value;
 }

 void ARMarkerData::setPosYValue(float pos_y_value)
 {
   posY = pos_y_value;
 }

 void ARMarkerData::setPosZValue(float pos_z_value)
 {
   posZ = pos_z_value;
 }

 void ARMarkerData::setThetaValue(float theta_value)
 {
   theta = theta_value;
 }

 void ARMarkerData::setPhyValue(float phy_value)
 {
   phy = phy_value;
 }

 void ARMarkerData::setPsiValue(float psi_value)
 {
   psi = psi_value;
 }

 float ARMarkerData::getPosXValue()
 {
   return  posX;
 }

 float ARMarkerData::getPosYValue()
 {
   return posY;
 }
 float ARMarkerData::getPosZValue()
 {
   return posZ;
 }

 float ARMarkerData::getThetaValue()
 {
   return theta;
 }

 float ARMarkerData::getPhyValue()
 {
   return phy;
 }

 float ARMarkerData::getPsiValue()
 {
   return psi;
 }

 void ARMarkerData::setRawPosXValue(float raw_pos_x_value)
 {
   rawPosX = raw_pos_x_value;
 }

 void ARMarkerData::setRawPosYValue(float raw_pos_y_value)
 {
   rawPosY = raw_pos_y_value;
 }

 void ARMarkerData::setRawPosZValue(float raw_pos_z_value)
 {
   rawPosZ = raw_pos_z_value;
 }

 void ARMarkerData::setRawThetaValue(float raw_psi_value)
 {
   rawTheta = raw_psi_value;
 }

 void ARMarkerData::setRawPhyValue(float raw_psi_value)
 {
   rawPhy = raw_psi_value;
 }

 void ARMarkerData::setRawPsiValue(float raw_psi_value)
 {
   rawPsi = raw_psi_value;
 }

 float ARMarkerData::getRawPosXValue()
 {
   return rawPosX;
 }
 float ARMarkerData::getRawPosYValue()
 {
   return rawPosY;
 }

 float ARMarkerData::getRawPosZValue()
 {
   return rawPosZ;
 }

 float ARMarkerData::getRawThetaValue()
 {
   return rawTheta;
 }

 float ARMarkerData::getRawPhyValue()
 {
   return rawPhy;
 }

 float ARMarkerData::getRawPsiValue()
 {
   return rawPsi;
 }

 void ARMarkerData::setVelXValue(float vel_x_value)
 {
   velX = vel_x_value;
 }

 void ARMarkerData::setVelYValue(float vel_y_value)
 {
   velY = vel_y_value;
 }

 void ARMarkerData::setVelZValue(float vel_z_value)
 {
   velZ = vel_z_value;
 }

 void ARMarkerData::setVelThetaValue(float vel_theta_value)
 {
   velTheta = vel_theta_value;
 }

 void ARMarkerData::setVelPhyValue(float vel_phy_value)
 {
   velPhy = vel_phy_value;
 }

 void ARMarkerData::setVelPsiValue(float vel_psi_value)
 {
   velPsi = vel_psi_value;
 }

 float ARMarkerData::getVelXValue()
 {
   return velX;
 }

 float ARMarkerData::getVelYValue()
 {
   return velY;
 }

 float ARMarkerData::getVelZValue()
 {
   return velZ;
 }

 float ARMarkerData::getVelThetaValue()
 {
   return velTheta;
 }

 float ARMarkerData::getVelPhyValue()
 {
   return velPhy;
 }

 float ARMarkerData::getVelPsiValue()
 {
   return velPsi;
 }

 void ARMarkerData::setRawVelXValue(float raw_vel_x_value)
 {
   rawVelX = raw_vel_x_value;
 }

 void ARMarkerData::setRawVelYValue(float raw_vel_y_value)
 {
   rawVelY = raw_vel_y_value;
 }

 void ARMarkerData::setRawVelZValue(float raw_vel_z_value)
 {
   rawVelZ = raw_vel_z_value;
 }

 void ARMarkerData::setRawVelThetaValue(float raw_vel_theta_value)
 {
   rawVelTheta = raw_vel_theta_value;
 }

 void ARMarkerData::setRawVelPhyValue(float raw_vel_phy_value)
 {
   rawVelPhy = raw_vel_phy_value;
 }

 void ARMarkerData::setRawVelPsiValue(float raw_vel_psi_value)
 {
   rawVelPsi = raw_vel_psi_value;
 }

 float ARMarkerData::getRawVelXValue()
 {
   return rawVelX;
 }

 float ARMarkerData::getRawVelYValue()
 {
   return rawVelY;
 }

 float ARMarkerData::getRawVelZValue()
 {
   return rawVelZ;
 }

 float ARMarkerData::getRawVelThetaValue()
 {
   return rawVelTheta;
 }

 float ARMarkerData::getRawVelPhyValue()
 {
   return rawVelPhy;
 }

 float ARMarkerData::getRawVelPsiValue()
 {
   return rawVelPsi;
 }


#if 0
void ARMarkerData::arMarkerPoseStampedCallback(const ar_pose::ARMarkers &ar_marker_pose_msg)
{

  static double previous_secs;
  static float prev_raw_pos_x, prev_raw_pos_y, prev_raw_pos_z, prev_raw_psi;

  if(ar_marker_pose_msg.markers.size() == 0) return;

  double current_secs = ar_marker_pose_msg.markers[0].header.stamp.toSec();

  //+*+*+: debug
  //ROS_INFO("the time diff is %lf", current_secs - previous_secs);

  //**** 位置情報の更新
  //**** 座標がおかしい, z in AR Marker is x, x in AR Marker is y, y in AR Marker is z

  if(calibCount > 0)
    {
      // initARMarkerPoseX  += (float)ar_marker_pose_msg.markers[0].pose.pose.position.z;
      // initARMarkerPoseY  += (float)ar_marker_pose_msg.markers[0].pose.pose.position.x;
      // initARMarkerPoseZ  += (float)ar_marker_pose_msg.markers[0].pose.pose.position.y;
      initARMarkerPoseX  += (float)ar_marker_pose_msg.markers[0].pose.pose.position.x;
      initARMarkerPoseY  += (float)ar_marker_pose_msg.markers[0].pose.pose.position.y;
      initARMarkerPoseZ  += (float)ar_marker_pose_msg.markers[0].pose.pose.position.z;
      initARMarkerPoseQ1 += (float)ar_marker_pose_msg.markers[0].pose.pose.orientation.x;
      initARMarkerPoseQ2 += (float)ar_marker_pose_msg.markers[0].pose.pose.orientation.y;
      initARMarkerPoseQ3 += (float)ar_marker_pose_msg.markers[0].pose.pose.orientation.z;
      initARMarkerPoseQ4 += (float)ar_marker_pose_msg.markers[0].pose.pose.orientation.w;
      calibCount--;

      ROS_WARN("Calib,  initARMarkerPoseZ is %f",initARMarkerPoseZ);
    }
  else if(calibCount == 0)
    {
      initARMarkerPoseX /= CALIBCOUNT;
      initARMarkerPoseY /= CALIBCOUNT;
      initARMarkerPoseZ /= CALIBCOUNT;
      initARMarkerPosePsi /= CALIBCOUNT;
      initARMarkerPoseQ1 /= CALIBCOUNT;
      initARMarkerPoseQ2 /= CALIBCOUNT;
      initARMarkerPoseQ3 /= CALIBCOUNT;
      initARMarkerPoseQ4 /= CALIBCOUNT;

      initARMarkerTF.setOrigin(tf::Vector3(initARMarkerPoseX,
                                           initARMarkerPoseY,
                                           initARMarkerPoseZ));
      initARMarkerTF.setRotation(tf::Quaternion(initARMarkerPoseQ1,
                                                initARMarkerPoseQ2,
                                                initARMarkerPoseQ3,
                                                initARMarkerPoseQ4));

      calibCount--;
      ROS_WARN("Calib End, initARMarkerPoseX is %f",initARMarkerPoseX);
      ROS_WARN("           initARMarkerPoseY is %f",initARMarkerPoseY);
      ROS_WARN("           initARMarkerPoseZ is %f",initARMarkerPoseZ);

    }
  else
    {


      tf::Transform arMarkerTF; arMarkerTF.setIdentity();
      tf::Transform cameraTF; cameraTF.setIdentity();


      arMarkerTF.setOrigin(tf::Vector3(ar_marker_pose_msg.markers[0].pose.pose.position.x,
                                       ar_marker_pose_msg.markers[0].pose.pose.position.y,
                                       ar_marker_pose_msg.markers[0].pose.pose.position.z));
      // arMarkerTF.setOrigin(tf::Vector3(ar_marker_pose_msg.markers[0].pose.pose.position.z,
      //                                  ar_marker_pose_msg.markers[0].pose.pose.position.x,
      //                                  ar_marker_pose_msg.markers[0].pose.pose.position.y));
      arMarkerTF.setRotation(tf::Quaternion(ar_marker_pose_msg.markers[0].pose.pose.orientation.x,
                                            ar_marker_pose_msg.markers[0].pose.pose.orientation.y,
                                            ar_marker_pose_msg.markers[0].pose.pose.orientation.z,
                                            ar_marker_pose_msg.markers[0].pose.pose.orientation.w));

      cameraTF.mult(initARMarkerTF, arMarkerTF.inverse());

      rawPosX = cameraTF.getOrigin().z();
      rawPosY = cameraTF.getOrigin().x();
      rawPosZ = cameraTF.getOrigin().y();

      // tfScalar yaw, roll, pitch;
      // cameraTF.getBasis().getRPY(roll, pitch, yaw);
      cameraTF.getBasis().getRPY((tfScalar&)rawPhy, (tfScalar&)rawTheta, (tfScalar&)rawPsi);


      //**** 速度情報の更新
      rawVelX = (rawPosX - prev_raw_pos_x) / (current_secs - previous_secs);
      rawVelY = (rawPosY - prev_raw_pos_y) / (current_secs - previous_secs);
      rawVelZ = (rawPosZ - prev_raw_pos_z) / (current_secs - previous_secs);


      if(rawPsi - prev_raw_psi > M_PI)
        rawVelPsi = (- 2 * M_PI + rawPsi - prev_raw_psi)
          / (current_secs - previous_secs);
      else if(rawPsi - prev_raw_psi < -M_PI)
        rawVelPsi = (2 * M_PI + rawPsi - prev_raw_psi)
          / (current_secs - previous_secs);
      else
        rawVelPsi = (rawPsi - prev_raw_psi)
          / (current_secs - previous_secs);


      //debug 
      double xAxisDelay = 0, yAxisDelay = 0, zAxisDelay = 0;
#if 0
      if(slamKalmanFilterFlag_)
        {

          xAxisDelay = kfbXARMarker_->correction(rawPosX, ar_marker_pose_msg.markers[0].header.stamp);
#if 1 // Slam Yaw
          yAxisDelay = kfbYARMarker_->correction(rawPosY,ar_marker_pose_msg.markers[0].header.stamp);
#else // Imu Yaw Test, using the Kalman Filter Bias of Y axis
          yAxisDelay = kfbYARMarker_->correction(rawPosX,ar_marker_pose_msg.markers[0].header.stamp);
#endif
          zAxisDelay = kfbZARMarker_->correction(rawPosZ,ar_marker_pose_msg.markers[0].header.stamp);

        }
#endif


      jsk_quadcopter::ARMarkerDebug arMarkerDebug_;
      arMarkerDebug_.header.stamp = ar_marker_pose_msg.markers[0].header.stamp;
      arMarkerDebug_.posX = posX;
      arMarkerDebug_.rawPosX = rawPosX;
      arMarkerDebug_.velX = velX;
      arMarkerDebug_.rawVelX = rawVelX;
      if(arMarkerKalmanFilterFlag_)
        {
          arMarkerDebug_.crrPosX2 = kfbXARMarker_->getEstimatePos();
          arMarkerDebug_.crrVelX2 = kfbXARMarker_->getEstimateVel();
          arMarkerDebug_.crrXBias = kfbXARMarker_->getEstimateBias();
        }

      arMarkerDebug_.posY = posY;
      arMarkerDebug_.rawPosY = rawPosY;
      arMarkerDebug_.velY = velY;
      arMarkerDebug_.rawVelY = rawVelY;

      if(arMarkerKalmanFilterFlag_)
        {
          arMarkerDebug_.crrPosY2  = kfbYARMarker_->getEstimatePos();
          arMarkerDebug_.crrVelY2  = kfbYARMarker_->getEstimateVel();
          arMarkerDebug_.crrYBias = kfbYARMarker_->getEstimateBias();
        }

      arMarkerDebug_.posZ = posZ;
      arMarkerDebug_.rawPosZ = rawPosZ;
      arMarkerDebug_.velZ = velZ;
      arMarkerDebug_.rawVelZ = rawVelZ;

      if(arMarkerKalmanFilterFlag_)
        {
          arMarkerDebug_.crrPosZ2  = kfbZARMarker_->getEstimatePos();
          arMarkerDebug_.crrVelZ2  = kfbZARMarker_->getEstimateVel();
          arMarkerDebug_.crrZBias = kfbZARMarker_->getEstimateBias();
        }

      arMarkerDebug_.psi = psi;
      arMarkerDebug_.rawPsi = rawPsi;
      arMarkerDebug_.velPsi = velPsi;
      arMarkerDebug_.rawVelPsi = rawVelPsi;
      arMarkerPub_.publish(arMarkerDebug_);



    }


   //更新
   previous_secs = current_secs;
   prev_raw_pos_x = rawPosX;
   prev_raw_pos_y = rawPosY;
   prev_raw_pos_z = rawPosZ;
   prev_raw_psi =  rawPsi;
}
#endif

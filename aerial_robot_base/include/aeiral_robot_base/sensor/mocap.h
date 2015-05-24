#ifndef MOCAP_DATA_H
#define MOCAP_DATA_H

//* ros
#include <ros/ros.h>
#include <jsk_quadcopter/state_estimation.h>
#include <jsk_quadcopter/digital_filter.h>
#include <jsk_quadcopter/SlamDebug.h>
#include <std_msgs/Float32.h>
//for hydra control

#include <tf/transform_listener.h>

class MocapData
{
 public:

 MocapData(ros::NodeHandle nh, ros::NodeHandle nh_private)  : mocapDataNodeHandle_(nh, "mocap"), mocapDataNodeHandlePrivate_(nh_private, "mocap")
      {//test
        rosParamInit();
        if(cog_offset_)
          {
            cogOffsetSub_ = mocapDataNodeHandle_.subscribe<std_msgs::Float32>("/hydra/cog_rotate", 5, &MocapData::cogOffsetCallback, this, ros::TransportHints().tcpNoDelay()); //absolute, should change the name
          }
        mocapSub_ = mocapDataNodeHandle_.subscribe<jsk_quadcopter::SlamDebug>("/ground_truth/pose", 5, &MocapData::testPoseCallback, this, ros::TransportHints().tcpNoDelay());

        
      }

 MocapData(ros::NodeHandle nh,
          ros::NodeHandle nh_private,
           Estimator* estimator)
   : mocapDataNodeHandle_(nh, "mocap"),
    mocapDataNodeHandlePrivate_(nh_private, "mocap")
      {
        rosParamInit();

        if(cog_offset_)
          {
            cogOffsetSub_ = mocapDataNodeHandle_.subscribe<std_msgs::Float32>("/hydra/cog_rotate", 5, &MocapData::cogOffsetCallback, this, ros::TransportHints().tcpNoDelay()); //absolute, should change the name
          }

        mocapSub_ = mocapDataNodeHandle_.subscribe<jsk_quadcopter::SlamDebug>("/ground_truth/pose", 5, boost::bind(&MocapData::poseCallback, this, _1, estimator));

        estimator->setOuterEstimatePoseFlag(estimator->X_AXIS | estimator->Y_AXIS | estimator->Z_AXIS | estimator->YAW_AXIS);
        estimator->setOuterEstimateVelFlag(estimator->X_AXIS | estimator->Y_AXIS | estimator->Z_AXIS | estimator->YAW_AXIS);

      }
  ~MocapData()
    {
    }


 private:
  ros::NodeHandle mocapDataNodeHandle_;
  ros::NodeHandle mocapDataNodeHandlePrivate_;
  ros::Subscriber mocapSub_;
  ros::Subscriber cogOffsetSub_;

  float cog_rotate_offset_;
  bool cog_offset_;
 
  void rosParamInit()
  {
    std::string ns = mocapDataNodeHandlePrivate_.getNamespace();
    mocapDataNodeHandlePrivate_.param ("cog_offset", cog_offset_, false);
    printf("%s: COG offset is %s\n", ns.c_str(), cog_offset_ ? ("true") : ("false"));

    cog_rotate_offset_ = 0;
  }

  void cogOffsetCallback(std_msgs::Float32 offset_msg)
  {
    cog_rotate_offset_ =  offset_msg.data;
  }

  void testPoseCallback(const jsk_quadcopter::SlamDebugConstPtr & pose_msg)
  {
    ROS_INFO("angle from map to cog is %f", pose_msg->psi + cog_rotate_offset_);
  }

  void poseCallback(const jsk_quadcopter::SlamDebugConstPtr & pose_msg,
                    Estimator* estimator)
  {
    estimator->setStatePosX(pose_msg->posX);
    estimator->setStatePosY(pose_msg->posY);
    estimator->setStatePosZ(pose_msg->posZ);
    estimator->setStatePsiCog(pose_msg->psi + cog_rotate_offset_);
    estimator->setStatePsiBoard(pose_msg->psi);
    //ROS_INFO("angle from map to cog is %f", pose_msg->psi + cog_rotate_offset_);
    estimator->setStateVelX(pose_msg->velX);
    estimator->setStateVelY(pose_msg->velY);
    estimator->setStateVelZ(pose_msg->velZ);
    estimator->setStateVelPsi(pose_msg->velPsi);

  }

};

#endif


#ifndef MOCAP_RETIME_H
#define MOCAP_RETIME_H

#include <ros/ros.h>
#include <jsk_quadcopter/MirrorModuleDebug.h>
#include <jsk_quadcopter/ImuDebug.h>
#include <jsk_quadcopter/SlamDebug.h>
#include <jsk_quadcopter/digital_filter.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_broadcaster.h"

class MocapRetime{
 public : 
  MocapRetime(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~MocapRetime();

  static const int TIME_SYNC_CALIB_COUNT = 10;

 protected:
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle nodeHandlePrivate_;

  ros::Subscriber poseSub_;
  ros::Subscriber qaudcopterSub_;
  ros::Publisher poseStampedPub_;

  //deprecated
  ros::Subscriber poseSimSub_;

  bool retimeFlag_;
  bool simFlag_;
  IirFilter filterX_,filterY_,filterZ_, filterPsi_;
  IirFilter filterAccX_,filterAccY_,filterAccZ_;
  double rxFreq_;
  double cutoffPosFreq_;
  double cutoffVelFreq_;
  std::string pubName_;

  long offset, secOffset, nSecOffset;
  int timeSyncCount;
  float posXOffset, posYOffset, posZOffset;

  void poseCallback(const geometry_msgs::PoseConstPtr & msg);
  void quadcopterCallback(const jsk_quadcopter::ImuDebugConstPtr & msg);

  //deprecated
  void poseSimCallback(const jsk_quadcopter::SlamDebugConstPtr & msg);

};
#endif

#include "mocap_retime/mocap_retime.h"


MocapRetime::MocapRetime(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nodeHandle_(nh), nodeHandlePrivate_(nh_private)
{
  offset = 0;
  secOffset = 0;
  nSecOffset = 0;
  timeSyncCount = TIME_SYNC_CALIB_COUNT;

  nodeHandlePrivate_.param("simFlag", simFlag_, false);
  nodeHandlePrivate_.param("retimeFlag", retimeFlag_, true);
  nodeHandlePrivate_.param("pubName", pubName_, std::string("ground_truth/pose"));
  nodeHandlePrivate_.param("rxFreq", rxFreq_, 100.0);
  nodeHandlePrivate_.param("cutoffPosFreq", cutoffPosFreq_, 50.0);
  nodeHandlePrivate_.param("cutoffVelFreq", cutoffVelFreq_, 50.0);

  filterX_ =  IirFilter((float)rxFreq_,
                        (float)cutoffPosFreq_,
                        (float)cutoffVelFreq_);
  filterY_ =  IirFilter((float)rxFreq_,
                        (float)cutoffPosFreq_,
                        (float)cutoffVelFreq_);
  filterZ_ =  IirFilter((float)rxFreq_,
                        (float)cutoffPosFreq_,
                        (float)cutoffVelFreq_);
  filterPsi_ =  IirFilter((float)rxFreq_,
                          (float)cutoffPosFreq_,
                          (float)cutoffVelFreq_);

  filterAccX_ =  IirFilter((float)rxFreq_,
                           (float)cutoffPosFreq_);
  filterAccY_ =  IirFilter((float)rxFreq_,
                           (float)cutoffPosFreq_);
  filterAccZ_ =  IirFilter((float)rxFreq_,
                           (float)cutoffPosFreq_);


  poseStampedPub_ = nodeHandle_.advertise<jsk_quadcopter::SlamDebug>(pubName_, 5); 

  qaudcopterSub_ = nodeHandle_.subscribe<jsk_quadcopter::ImuDebug>("imu/debug", 1, &MocapRetime::quadcopterCallback, this, ros::TransportHints().tcpNoDelay());

  if(!retimeFlag_)
    timeSyncCount = 0;

  if(!simFlag_)
    poseSub_ = nodeHandle_.subscribe<geometry_msgs::Pose>("/quadcopter/pose", 1, &MocapRetime::poseCallback, this, ros::TransportHints().tcpNoDelay());
  else
    poseSimSub_ = nodeHandle_.subscribe<jsk_quadcopter::SlamDebug>("/ground_truth/pose", 1, &MocapRetime::poseSimCallback, this, ros::TransportHints().tcpNoDelay());

}

MocapRetime::~MocapRetime()
{
}

void MocapRetime::poseCallback(const geometry_msgs::PoseConstPtr & msg)
{
  static float prev_pos_x, prev_pos_y, prev_pos_z, prev_psi;
  static float prev_vel_x, prev_vel_y, prev_vel_z;
  static bool first_flag = true;
  static ros::Time previous_time;

  double raw_pos_x = 0, raw_pos_y = 0, raw_pos_z = 0, raw_theta = 0, raw_phy = 0, raw_psi = 0;
  double raw_vel_x = 0, raw_vel_y = 0, raw_vel_z = 0, raw_vel_theta = 0, raw_vel_phy = 0, raw_vel_psi = 0;
  double pos_x = 0, pos_y = 0, pos_z = 0, theta = 0, phy = 0, psi = 0;
  double vel_x = 0, vel_y = 0, vel_z = 0, vel_theta = 0, vel_phy = 0, vel_psi = 0;

  double raw_acc_x = 0, raw_acc_y = 0, raw_acc_z = 0;
  double acc_x = 0, acc_y = 0, acc_z = 0;

  if(timeSyncCount == 0)
    {
      if(!first_flag)
        {

          raw_pos_x = msg->position.x - posXOffset;
          raw_vel_x = (msg->position.x - prev_pos_x) / (ros::Time::now().toSec() - previous_time.toSec());
          raw_acc_x = (raw_vel_x - prev_vel_x) / (ros::Time::now().toSec() - previous_time.toSec());
          raw_pos_y = msg->position.y - posYOffset;
          raw_vel_y = (msg->position.y - prev_pos_y) / (ros::Time::now().toSec() - previous_time.toSec());
          raw_acc_y = (raw_vel_y - prev_vel_y) / (ros::Time::now().toSec() - previous_time.toSec());
          raw_pos_z = msg->position.z - posZOffset;
          raw_vel_z = (msg->position.z - prev_pos_z) / (ros::Time::now().toSec() - previous_time.toSec());
          raw_acc_z = (raw_vel_z - prev_vel_z) / (ros::Time::now().toSec() - previous_time.toSec());

          tf::Quaternion q(msg->orientation.x,
                           msg->orientation.y,
                           msg->orientation.z,
                           msg->orientation.w);
          tf::Matrix3x3(q).getRPY(raw_phy, raw_theta, raw_psi);
          raw_vel_psi = (raw_psi - prev_psi) / (ros::Time::now().toSec() - previous_time.toSec());

          filterX_.filterFunction(raw_pos_x, pos_x, raw_vel_x, vel_x);
          filterY_.filterFunction(raw_pos_y, pos_y, raw_vel_y, vel_y);
          filterZ_.filterFunction(raw_pos_z, pos_z, raw_vel_z, vel_z);
          filterPsi_.filterFunction(raw_psi, psi, raw_vel_psi, vel_psi);

          filterAccX_.filterFunction(raw_acc_x, acc_x);
          filterAccY_.filterFunction(raw_acc_y, acc_y);
          filterAccZ_.filterFunction(raw_acc_z, acc_z);


          jsk_quadcopter::SlamDebug ground_truth_pose;
          ground_truth_pose.header.stamp.fromNSec(ros::Time::now().toNSec() - offset);
          ground_truth_pose.rawPosX = raw_pos_x;
          ground_truth_pose.rawVelX = raw_vel_x;
          ground_truth_pose.posX = pos_x;
          ground_truth_pose.velX = vel_x;
          ground_truth_pose.emptyX1 = acc_x;
          ground_truth_pose.emptyX2 = raw_acc_x;
          ground_truth_pose.rawPosY = raw_pos_y;
          ground_truth_pose.rawVelY = raw_vel_y;
          ground_truth_pose.posY = pos_y;
          ground_truth_pose.velY = vel_y;
          ground_truth_pose.emptyY1 = acc_y;
          ground_truth_pose.emptyY2 = raw_acc_y;
          ground_truth_pose.rawPosZ = raw_pos_z;
          ground_truth_pose.rawVelZ = raw_vel_z;
          ground_truth_pose.posZ = pos_z;
          ground_truth_pose.velZ = vel_z;
          ground_truth_pose.emptyZ1 = acc_z;
          ground_truth_pose.emptyZ2 = raw_acc_z;

          ground_truth_pose.rawPsi = raw_psi;
          ground_truth_pose.rawVelPsi = raw_vel_psi;
          ground_truth_pose.psi = psi;
          ground_truth_pose.velPsi = vel_psi;
          ground_truth_pose.emptyPsi1 = raw_theta;
          ground_truth_pose.emptyPsi2 = raw_phy;

          poseStampedPub_.publish(ground_truth_pose);

        }

      if(first_flag)
        {
          prev_pos_x = msg->position.x;
          prev_pos_y = msg->position.y;
          prev_pos_z = msg->position.z;

          posXOffset = msg->position.x;
          posYOffset = msg->position.y;
          posZOffset = msg->position.z;
          first_flag = false;
        }

      prev_pos_x = msg->position.x;
      prev_pos_y = msg->position.y;
      prev_pos_z = msg->position.z;
      prev_vel_x = raw_vel_x;
      prev_vel_y = raw_vel_y;
      prev_vel_z = raw_vel_z;

      prev_psi = raw_psi;
      previous_time = ros::Time::now();

    }
}


void MocapRetime::poseSimCallback(const jsk_quadcopter::SlamDebugConstPtr & msg)
{
  static float prev_pos_x, prev_pos_y, prev_pos_z;
  static float prev_vel_x, prev_vel_y, prev_vel_z;
  static bool first_flag = true;
  static ros::Time previous_time;

  if(timeSyncCount == 0)
    {
      double raw_pos_x = 0, raw_pos_y = 0, raw_pos_z = 0;
      double raw_vel_x = 0, raw_vel_y = 0, raw_vel_z = 0;
      double raw_acc_x = 0, raw_acc_y = 0, raw_acc_z = 0;
      double pos_x = 0, pos_y = 0, pos_z = 0;
      double vel_x = 0, vel_y = 0, vel_z = 0;
      double acc_x = 0, acc_y = 0, acc_z = 0;

      if(!first_flag)
        {
          raw_pos_x = msg->rawPosX - posXOffset;
          raw_vel_x = (msg->rawPosX - prev_pos_x) / (ros::Time::now().toSec() - previous_time.toSec());
          raw_acc_x = (raw_vel_x - prev_vel_x) / (ros::Time::now().toSec() - previous_time.toSec());
          raw_pos_y = msg->rawPosY - posYOffset;
          raw_vel_y = (msg->rawPosY - prev_pos_y) / (ros::Time::now().toSec() - previous_time.toSec());
          raw_acc_y = (raw_vel_y - prev_vel_y) / (ros::Time::now().toSec() - previous_time.toSec());
          raw_pos_z = msg->rawPosZ - posZOffset;
          raw_vel_z = (msg->rawPosZ - prev_pos_z) / (ros::Time::now().toSec() - previous_time.toSec());
          raw_acc_z = (raw_vel_z - prev_vel_z) / (ros::Time::now().toSec() - previous_time.toSec());

          filterX_.filterFunction(raw_pos_x, pos_x, raw_vel_x, vel_x);
          filterY_.filterFunction(raw_pos_y, pos_y, raw_vel_y, vel_y);
          filterZ_.filterFunction(raw_pos_z, pos_z, raw_vel_z, vel_z);

          filterAccX_.filterFunction(raw_acc_x, acc_x);
          filterAccY_.filterFunction(raw_acc_y, acc_y);
          filterAccZ_.filterFunction(raw_acc_z, acc_z);

          jsk_quadcopter::SlamDebug ground_truth_pose;
          ground_truth_pose.header.stamp.fromNSec(ros::Time::now().toNSec() - offset);
          ground_truth_pose.rawPosX = raw_pos_x;
          ground_truth_pose.rawVelX = raw_vel_x;
          ground_truth_pose.posX = pos_x;
          ground_truth_pose.velX = vel_x;
          ground_truth_pose.emptyX1 = acc_x;
          ground_truth_pose.emptyX2 = raw_acc_x;
          ground_truth_pose.rawPosY = raw_pos_y;
          ground_truth_pose.rawVelY = raw_vel_y;
          ground_truth_pose.posY = pos_y;
          ground_truth_pose.velY = vel_y;
          ground_truth_pose.emptyY1 = acc_y;
          ground_truth_pose.emptyY2 = raw_acc_y;
          ground_truth_pose.rawPosZ = raw_pos_z;
          ground_truth_pose.rawVelZ = raw_vel_z;
          ground_truth_pose.posZ = pos_z;
          ground_truth_pose.velZ = vel_z;
          ground_truth_pose.emptyZ1 = acc_z;
          ground_truth_pose.emptyZ2 = raw_acc_z;

          poseStampedPub_.publish(ground_truth_pose);
        }

      if(first_flag)
        {
          prev_pos_x = msg->rawPosX;
          prev_pos_y = msg->rawPosY;
          prev_pos_z = msg->rawPosZ;

          posXOffset = msg->rawPosX;
          posYOffset = msg->rawPosY;
          posZOffset = msg->rawPosZ;
          first_flag = false;
        }

      prev_pos_x = msg->rawPosX;
      prev_pos_y = msg->rawPosY;
      prev_pos_z = msg->rawPosZ;
      prev_vel_x = raw_vel_x;
      prev_vel_y = raw_vel_y;
      prev_vel_z = raw_vel_z;
      previous_time = ros::Time::now();
      //prev_theta ;
    }
}


void MocapRetime::quadcopterCallback(const jsk_quadcopter::ImuDebugConstPtr & msg)
{
  if(timeSyncCount > 0)
    {
      timeSyncCount --;
      if(timeSyncCount < TIME_SYNC_CALIB_COUNT)
        {
          long offsetTmp = ros::Time::now().toNSec() - msg->header.stamp.toNSec();
          secOffset  = secOffset +  (offsetTmp / 1000000000);
          nSecOffset = nSecOffset + (offsetTmp % 1000000000);
          //ROS_INFO("mocap time is %ld, imu time is %ld, secOffset is %ld, nSecOffset is %ld", ros::Time::now().toNSec(), msg->header.stamp.toNSec(), offsetTmp, nSecOffset);
        }
      if(timeSyncCount == 0)
        {
          secOffset /= TIME_SYNC_CALIB_COUNT;
          nSecOffset /= TIME_SYNC_CALIB_COUNT;
          offset = secOffset * 1000000000 + nSecOffset;
          
          if(retimeFlag_)
            poseSub_ = nodeHandle_.subscribe<geometry_msgs::Pose>("/quadcopter/pose", 1, &MocapRetime::poseCallback, this, ros::TransportHints().tcpNoDelay());

          ROS_INFO("offset is %ld.%ld", secOffset, nSecOffset);
          qaudcopterSub_.shutdown();
        }
    }
}






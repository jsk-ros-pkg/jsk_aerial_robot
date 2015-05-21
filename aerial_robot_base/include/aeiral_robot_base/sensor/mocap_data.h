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

    /*
    ROS_INFO("pos_x: %f, pos_y: %f, pos_z: %f, psi: %f, vel_x: %f, vel_y: %f, vel_z: %f, _velpsi: %f,",
             estimator->getStatePosX(),
             estimator->getStatePosY(),
             estimator->getStatePosZ(),
             estimator->getStatePsi(),
             estimator->getStateVelX(),
             estimator->getStateVelY(),
             estimator->getStateVelZ(),
             estimator->getStateVelPsi());
    */
  }

};

#endif






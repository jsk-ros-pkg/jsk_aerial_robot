#ifndef CHECKER_BOARD_H
#define CHECKER_BOARD_H

#include <ros/ros.h>
#include <posedetection_msgs/ObjectDetection.h>
#include <boost/thread/mutex.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <aerial_robot_base/FlightNav.h>

class CheckerBoard
{
 public:

 CheckerBoard(ros::NodeHandle nh,	ros::NodeHandle nh_private): nh_(nh), nhp_(nh_private, "tracking")
    {
      rosParamInit(nh_);

      checkerboard_dectect_time = ros::Time().now();

      checkerboard_sub_ = nh_.subscribe<posedetection_msgs::ObjectDetection>("/checkerdetector/ObjectDetection", 1, &CheckerBoard::checkerBoardCallback, this, ros::TransportHints().tcpNoDelay());

      timer_ = nhp_.createTimer(ros::Duration(1.0 / loop_rate_), &CheckerBoard::loopFunc, this);
 }
  ~CheckerBoard();

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber checkerboard_sub_;

  ros::Timer  timer_;

  tf::TransformListener tf_;

  boost::mutex mutex_time_;

  ros::Time checkerboard_dectect_time_;
  double loop_rate_;
  double detect_time_limit_;
  double target_x_;
  double target_y_;
  double target_z_;
  double target_psi_;

  std::string camera_frame_;
  std::string checkerboard_frame_;
  



  void loopFunc(const ros::TimerEvent & e)
  {
    boost::mutex::scoped_lock lock(mutex_time);
    if(ros::Time().now().toSec() - checkerboard_dectect_time.toSec() > detect_time_limit_ &&
       ros::Time().now().toSec() - checkerboard_dectect_time.toSec() < detect_time_limit_ + 4 / loop_rate_)
      {
        //navigation 
        navigator_->set_x_yControlMode(navigator_->VEL_LOCAL_BASED_CONTROL_MODE);

        //estimation
        estimator_->setOuterEstimatePoseFlag(0);
        estimator_->setOuterEstimateVelFlag(0);
        ROS_WARN("stop");
      }
  }

  void checkerBoardCallback(const posedetection_msgs::ObjectDetectionConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_time_);

    checkerboard_dectect_time_ = ros::Time().now();

    if(msg->objects.size() > 0)
      {

#if  1 //use tf
        float pos_x = 0, pos_y = 0, pos_z = 0;
        float theta = 0, phy = 0, psi =0;

        try
          {
            tf::StampedTransform stamped_pose;
            tf_.waitForTransform(checkerboard_frame_, camera_frame_, checkerboard_dectect_time, ros::Duration(2.0));
            tf_.lookupTransform(checkerboard_frame_, camera_frame_, checkerboard_dectect_time, stamped_pose);

            pos_x = stamped_pose.getOrigin().get_x_();
            pos_y = stamped_pose.getOrigin().getY();
            pos_z = stamped_pose.getOrigin().getZ();
            stamped_pose.getBasis().getEulerYPR((double&)psi, (double&)theta, (double&)phy);
          }
        catch(tf::TransformException e)
          {
            ROS_ERROR("Transform from %s to %s failed\n", camera_frame_.c_str(), checkerboard_frame_.c_str());
          }
#else //deprectaed
        tf::Quaternion tmp;
        tf::quaternionMsgToTF(msg->objects[0].pose.orientation, tmp);

        tf::Transform inverse_tf 
          = tf::Transform(tmp, tf::Vector3(msg->objects[0].pose.position.x,
                                           msg->objects[0].pose.position.y,
                                           msg->objects[0].pose.position.z)).inverse();
        //navigation
        navigator_->set_x_yControlMode(navigator_->POS_LOCAL_BASED_CONTROL_MODE);
        navigator_->setTargetPos_x_(target_x_);
        navigator_->setTargetPosY(targetY);
        navigator_->setTargetPsi(targetPsi);

        float pos_x = inverse_tf.getOrigin().get_x_();
        float pos_y = inverse_tf.getOrigin().getY();
        float pos_z = inverse_tf.getOrigin().getZ();
        float theta, phy, psi;
        inverse_tf.getBasis().getRPY((double&)phy, (double&)theta, (double&)psi);

        //estimation
        estimator_->setOuterEstimatePoseFlag(estimator_->_x__A_x_IS | estimator_->Y_A_x_IS | estimator_->YAW_A_x_IS);
        estimator_->setStatePosX(pos_x);
        estimator_->setStatePosY(pos_y);
        estimator_->setStatePsi(psi);

#endif

        aerial_robot_base::FlightNav navi_command;
        pose.header.stamp = ros::Time().now();
        pose.position.x = pos_x;
        pose.position.y = pos_y;
        pose.position.z = pos_z;

        pose.rpy.z = psi;
  
        quadcopter_relative_pose_pub_.publish(pose);
        //debug
        ROS_INFO("get checker board pose");
      }
  }


  void rosParamInit(ros::NodeHandle nh)
  {
    std::string ns = nh.getNamespace();
    if (!nh.getParam ("loop_rate", loop_rate_))
      loop_rate_ = 2.0;
    printf("%s: loop_rate_ is %.3f\n", ns.c_str(), loop_rate_);

    if (!nh.getParam ("detect_time_limit", detect_time_limit_))
      detect_time_limit_ = 2.0;
    printf("%s: detect_time_limit_ is %.3f\n", ns.c_str(), detect_time_limit_);

    if (!nh.getParam ("target_x", target_x_))
      target_x_ = 0;
    printf("%s: target_x_ is %.3f\n", ns.c_str(), target_x_);

    if (!nh.getParam ("target_y", target_y_))
      target_y_ = 0;
    printf("%s: target_y_ is %.3f\n", ns.c_str(), target_y_);

    if (!nh.getParam ("target_z", target_z_))
      target_z_ = 0;
    printf("%s: target_z_ is %.3f\n", ns.c_str(), target_z_);

    if (!nh.getParam ("target_psi", target_psi_))
      target_psi_ = 0;
    printf("%s: target_psi_ is %.3f\n", ns.c_str(), target_psi_);

    if (!nh.getParam ("camera_frame", camera_frame_))
      camera_frame_ = std::string("camera_attached_body");
    printf("%s: camera_frame_ is %s\n", ns.c_str(), camera_frame_.c_str());

    if (!nh.getParam ("checkerboard_frame", checkerboard_frame_))
      checkerboard_frame_ = std::string("checkerboard");
    printf("%s: checkerboard_frame_ is %s\n", ns.c_str(), checkerboard_frame_.c_str());
  }


};


#endif



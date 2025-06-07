// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <spinal/DesireCoord.h>

namespace aerial_robot_navigation
{
  class GimbalrotorNavigator : public BaseNavigator
  {
  public:
    GimbalrotorNavigator();
    ~GimbalrotorNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    void update() override;

  private:
    ros::Publisher curr_target_baselink_rot_pub_;
    ros::Subscriber final_target_baselink_rot_sub_;

    void baselinkRotationProcess();
    void rosParamInit() override;
    void setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg);
    void naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg) override;


    /* target baselink rotation */
    double prev_rotation_stamp_;
    tf::Vector3 curr_target_baselink_rot_, final_target_baselink_rot_;

    /* rosparam */
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;
  };
};

// -*- mode: c++ -*-
#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <spinal/DesireCoord.h>

namespace aerial_robot_navigation
{
  class RollingNavigator : public BaseNavigator
  {
  public:
    RollingNavigator();
    ~RollingNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator) override;

    void update() override;

  private:
    ros::Publisher curr_target_baselink_rot_pub_;
    ros::Subscriber final_target_baselink_rot_sub_;

    void baselinkRotationProcess();
    void rosParamInit() override;
    void setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg);

    /* target baselink rotation */
    double prev_rotation_stamp_;
    std::vector<double> target_gimbal_angles_;
    tf::Vector3 curr_target_baselink_rot_, final_target_baselink_rot_;

    /* rosparam */
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;
  };
};

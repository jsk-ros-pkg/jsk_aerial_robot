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

    tf::Vector3 getCurrTargetBaselinkRot(){return curr_target_baselink_rot_;}
    tf::Vector3 getFinalTargetBaselinkRot(){return final_target_baselink_rot_;}
 
    void setFinalTargetBaselinkRot(tf::Vector3 final_target_baselink_rot){final_target_baselink_rot_ = final_target_baselink_rot;}
  protected:
    void rosParamInit() override;
    virtual void setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg);
    
  private:
    ros::Publisher curr_target_baselink_rot_pub_;
    ros::Subscriber final_target_baselink_rot_sub_;

    void baselinkRotationProcess();
    /* target baselink rotation */
    double prev_rotation_stamp_;
    tf::Vector3 curr_target_baselink_rot_, final_target_baselink_rot_;

    /* rosparam */
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;
  };
};

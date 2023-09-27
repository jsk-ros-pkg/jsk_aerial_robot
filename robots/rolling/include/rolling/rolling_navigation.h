// -*- mode: c++ -*-
#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <spinal/DesireCoord.h>

namespace aerial_robot_navigation
{
  enum rolling_mode
    {
     FLYING_STATE,
     STANDING_STATE,
     STEERING_STATE,
     ROLLING_STATE
    };

  class RollingNavigator : public BaseNavigator
  {
  public:
    RollingNavigator();
    ~RollingNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator) override;

    void update() override;
    void reset() override;
    void setFinalTargetBaselinkRot(tf::Vector3 rot);
    inline tf::Vector3 getCurrTargetBaselinkRot() {return curr_target_baselink_rot_;}
    inline int getGroundNavigationMode() {return current_ground_navigation_mode_;}

  private:
    ros::Publisher curr_target_baselink_rot_pub_;
    ros::Subscriber final_target_baselink_rot_sub_;
    ros::Subscriber joy_sub_;

    void baselinkRotationProcess();
    void landingProcess();
    void rosParamInit() override;
    void setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg);
    void joyCallback(const sensor_msgs::JoyConstPtr & joy_msg);
    void setGroundNavigationMode(int state) {current_ground_navigation_mode_ = state;}

    /* navigation mode */
    int current_ground_navigation_mode_;

    /* target baselink rotation */
    double prev_rotation_stamp_;
    std::vector<double> target_gimbal_angles_;
    tf::Vector3 curr_target_baselink_rot_, final_target_baselink_rot_;

    /* landing process */
    bool landing_flag_;

    /* rosparam */
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;
  };
};

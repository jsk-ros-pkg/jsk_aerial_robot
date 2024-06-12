// -*- mode: c++ -*-
#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <delta/model/delta_robot_model.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/Int16.h>

namespace aerial_robot_navigation
{
  enum rolling_mode
    {
     NONE,
     FLYING_STATE,
     STANDING_STATE,
     ROLLING_STATE,
     DOWN_STATE
    };

  class RollingNavigator : public BaseNavigator
  {
  public:
    RollingNavigator();
    ~RollingNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    void update() override;
    void reset() override;

    void setPrevGroundNavigationMode(int mode) {prev_ground_navigation_mode_ = mode;}
    void setGroundNavigationMode(int state);
    inline int getCurrentGroundNavigationMode() {return current_ground_navigation_mode_;}
    inline int getPrevGroundNavigationMode() {return prev_ground_navigation_mode_;}

    void setControllersResetFlag(bool flag) {controllers_reset_flag_ = flag;}
    bool getControllersResetFlag() {return controllers_reset_flag_;}

    double getCurrentTargetBaselinkRpyRoll() {return curr_target_baselink_rpy_roll_;}
    double getCurrentTargetBaselinkRpyPitch() {return curr_target_baselink_rpy_pitch_;}
    void setCurrentTargetBaselinkRpyRoll(double roll) {curr_target_baselink_rpy_roll_ = roll;}
    void setCurrentTargetBaselinkRpyPitch(double pitch) {curr_target_baselink_rpy_pitch_ = pitch;}
    void setFinalTargetBaselinkQuat(tf::Quaternion quat) {final_target_baselink_quat_ = quat;}
    void setCurrentTargetBaselinkQuat(tf::Quaternion quat) {curr_target_baselink_quat_ = quat;}

    double getTargetPitchAngVel() {return target_pitch_ang_vel_;}
    double getTargetyawAngVel() {return target_yaw_ang_vel_;}
    bool getPitchAngVelUpdating() {return pitch_ang_vel_updating_;}
    bool getYawAngVelUpdating() {return yaw_ang_vel_updating_;}

    void setBaselinkRotForceUpdateMode(bool flag) {baselink_rot_force_update_mode_ = flag;}
    bool getBaselinkRotForceUpdateMode() {return baselink_rot_force_update_mode_;}
    std::string indexToGroundNavigationModeString(int index);

  private:
    /* baselink rotation process */
    ros::Publisher desire_coord_pub_;
    ros::Subscriber final_target_baselink_quat_sub_, final_target_baselink_rpy_sub_;

    /* joy */
    ros::Subscriber joy_sub_;

    /* ground mode */
    ros::Subscriber ground_navigation_mode_sub_;
    ros::Publisher ground_navigation_mode_pub_;

    boost::shared_ptr<RollingRobotModel> rolling_robot_model_;

    void rollingPlanner();
    void baselinkRotationProcess();
    void groundModeProcess();
    void rosPublishProcess();

    void rosParamInit() override;

    void groundNavigationModeCallback(const std_msgs::Int16Ptr & msg);
    void setFinalTargetBaselinkQuatCallback(const geometry_msgs::QuaternionStampedConstPtr & msg);
    void setFinalTargetBaselinkRpyCallback(const geometry_msgs::Vector3StampedConstPtr & msg);
    void joyCallback(const sensor_msgs::JoyConstPtr & joy_msg);

    /* navigation mode */
    int current_ground_navigation_mode_;
    int prev_ground_navigation_mode_;

    /* flight mode variable */
    bool controllers_reset_flag_;

    /* ground mode variable */
    double target_pitch_ang_vel_;
    double target_yaw_ang_vel_;
    double rolling_max_pitch_ang_vel_;
    double rolling_max_yaw_ang_vel_;
    bool pitch_ang_vel_updating_;
    bool yaw_ang_vel_updating_;
    double down_mode_roll_anglvel_;
    double down_start_time_;


    /* param for joy stick control */
    double joy_stick_deadzone_;

    /* target baselink rotation */
    double prev_rotation_stamp_;
    tf::Quaternion curr_target_baselink_quat_, final_target_baselink_quat_;
    double curr_target_baselink_rpy_roll_, curr_target_baselink_rpy_pitch_;
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;
    bool baselink_rot_force_update_mode_;

    /* landing process */
    bool landing_flag_;
  };
};

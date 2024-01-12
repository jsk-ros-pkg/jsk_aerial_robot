// -*- mode: c++ -*-
#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <delta/model/rolling_robot_model.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/Int16.h>

namespace aerial_robot_navigation
{
  enum rolling_mode
    {
     FLYING_STATE,
     STANDING_STATE,
     STEERING_STATE,
     ROLLING_STATE,
     RECOVERING_STATE
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

    inline tf::Vector3 getCurrTargetBaselinkRot() {return curr_target_baselink_rot_;}
    inline int getCurrentGroundNavigationMode() {return current_ground_navigation_mode_;}
    inline int getPrevGroundNavigationMode() {return prev_ground_navigation_mode_;}
    void setPrevGroundNavigationMode(int mode) {prev_ground_navigation_mode_ = mode;}
    void setGroundNavigationMode(int state);
    inline tf::Vector3 getStandingInitialPos() {return standing_initial_pos_;}
    inline tf::Vector3 getStandingInitialEuler() {return standing_initial_euler_;}
    inline tf::Vector3 getSteeringInitialPos() {return steering_initial_pos_;}
    inline tf::Vector3 getSteeringInitialEuler() {return steering_initial_euler_;}
    inline tf::Vector3 getRollingInitialPos() {return rolling_initial_pos_;}
    inline tf::Vector3 getRollingInitialEuler() {return rolling_initial_euler_;}
    double getTargetPitchAngVel() {return target_pitch_ang_vel_;}
    double getTargetyawAngVel() {return target_yaw_ang_vel_;}
    bool getPitchAngVelUpdating() {return pitch_ang_vel_updating_;}
    bool getYawAngVelUpdating() {return yaw_ang_vel_updating_;}
    void setFinalTargetBaselinkRot(tf::Vector3 rot) {final_target_baselink_rot_.setValue(rot.x(), rot.y(), rot.z());}
    void setFinalTargetBaselinkRotRoll(double rad) {final_target_baselink_rot_.setX(rad);}
    void setFinalTargetBaselinkRotPitch(double rad) {final_target_baselink_rot_.setY(rad);}
    void setCurrentTargetBaselinkRot(tf::Vector3 rot) {curr_target_baselink_rot_.setValue(rot.x(), rot.y(), rot.z());}
    void setCurrentTargetBaselinkRotRoll(double rad) {curr_target_baselink_rot_.setX(rad);}
    void setCurrentTargetBaselinkRotPitch(double rad) {curr_target_baselink_rot_.setY(rad);}
    double getCurrTargetBaselinkRotRoll() {return curr_target_baselink_rot_roll_;}
    double getCurrTargetBaselinkRotPitch() {return curr_target_baselink_rot_pitch_;}
    double getFinalTargetBaselinkRotRoll() {return final_target_baselink_rot_roll_;}
    double getFinalTargetBaselinkRotPitch() {return final_target_baselink_rot_pitch_;}
    void setBaselinkRotForceUpdateMode(bool flag) {baselink_rot_force_update_mode_ = flag;}
    bool getBaselinkRotForceUpdateMode() {return baselink_rot_force_update_mode_;}

  private:
    /* baselink rotation process */
    ros::Publisher curr_target_baselink_rot_pub_;
    ros::Subscriber final_target_baselink_rot_sub_;

    /* joy */
    ros::Subscriber joy_sub_;

    /* ground mode */
    ros::Subscriber ground_navigation_mode_sub_;
    ros::Publisher ground_navigation_mode_pub_;

    boost::shared_ptr<RollingRobotModel> rolling_robot_model_;

    void baselinkRotationProcess();
    void landingProcess();
    void groundModeProcess();
    void rosPublishProcess();

    void rosParamInit() override;

    void groundNavigationModeCallback(const std_msgs::Int16Ptr & msg);
    void setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg);
    void joyCallback(const sensor_msgs::JoyConstPtr & joy_msg);

    /* navigation mode */
    int current_ground_navigation_mode_;
    int prev_ground_navigation_mode_;

    /* standing mode variable */
    tf::Vector3 standing_initial_pos_;
    tf::Vector3 standing_initial_euler_;

    /* steering mode variable */
    tf::Vector3 steering_initial_pos_;
    tf::Vector3 steering_initial_euler_;

    /* rolling mode variable */
    tf::Vector3 rolling_initial_pos_;
    tf::Vector3 rolling_initial_euler_;
    double target_pitch_ang_vel_;
    double target_yaw_ang_vel_;
    double rolling_max_pitch_ang_vel_;
    double rolling_max_yaw_ang_vel_;
    bool pitch_ang_vel_updating_;
    bool yaw_ang_vel_updating_;

    /* param for joy stick control */
    double joy_stick_deadzone_;

    /* target baselink rotation */
    double baselink_rotation_stop_error_;
    double prev_rotation_stamp_;
    tf::Vector3 curr_target_baselink_rot_, final_target_baselink_rot_;
    double curr_target_baselink_rot_roll_, curr_target_baselink_rot_pitch_, final_target_baselink_rot_roll_, final_target_baselink_rot_pitch_;
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;
    bool baselink_rot_force_update_mode_;

    /* landing process */
    bool landing_flag_;

  };
};

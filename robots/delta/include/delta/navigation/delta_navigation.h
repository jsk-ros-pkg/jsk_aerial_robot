// -*- mode: c++ -*-
#pragma once

#include <delta/model/delta_robot_model.h>
#include <aerial_robot_control/flight_navigation.h>
#include <aerial_robot_control/trajectory/trajectory_reference/polynomial.hpp>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>
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
    void startTakeoff() override;

    void setPrevGroundNavigationMode(int mode) {prev_ground_navigation_mode_ = mode;}
    void setGroundNavigationMode(int state);
    void setGroundMotionMode(int state);
    inline int getCurrentGroundNavigationMode() {return current_ground_navigation_mode_;}
    inline int getPrevGroundNavigationMode() {return prev_ground_navigation_mode_;}

    void setControllersResetFlag(bool flag) {controllers_reset_flag_ = flag;}
    bool getControllersResetFlag() {return controllers_reset_flag_;}

    void setRotationControlLink(std::string link_name);
    void setCurrentTargetControlLinkRotation(KDL::Rotation rot) {curr_target_cog_R_link_ = rot;}
    void setFinalTargetControlLinkRotation(KDL::Rotation rot) {final_target_cog_R_link_ = rot;}
    void setFinalTargetBaselinkQuat(tf::Quaternion quat) {final_target_baselink_quat_ = quat;}
    tf::Quaternion getFinalTargetBaselinkQuat() {return final_target_baselink_quat_;}
    void setCurrentTargetBaselinkQuat(tf::Quaternion quat) {curr_target_baselink_quat_ = quat;}
    tf::Quaternion getCurrentTargetBaselinkQuat() {return curr_target_baselink_quat_;}

    double getTargetPitchAngVel() {return target_pitch_ang_vel_;}
    double getTargetyawAngVel() {return target_yaw_ang_vel_;}
    bool getPitchAngVelUpdating() {return pitch_ang_vel_updating_;}
    bool getYawAngVelUpdating() {return yaw_ang_vel_updating_;}

    std::string indexToGroundNavigationModeString(int index);

    void setEstimatedExternalWrench(Eigen::VectorXd est_external_wrench) {est_external_wrench_ = est_external_wrench;}
    void setEstimatedExternalWrenchCog(Eigen::VectorXd est_external_wrench_cog) {est_external_wrench_cog_ = est_external_wrench_cog;}

  private:
    /* baselink rotation process */
    ros::Publisher desire_coord_pub_;
    ros::Subscriber final_target_baselink_quat_sub_, final_target_baselink_rpy_sub_;

    /* transform planning */
    ros::Publisher joints_control_pub_;
    ros::Subscriber joints_control_sub_;
    ros::Subscriber ik_target_rel_ee_pos_sub_;

    /* joy */
    ros::Subscriber joy_sub_;

    /* ground mode */
    ros::Subscriber ground_navigation_mode_sub_;
    ros::Publisher ground_navigation_mode_pub_;

    boost::shared_ptr<RollingRobotModel> rolling_robot_model_;
    boost::shared_ptr<RollingRobotModel> robot_model_for_plan_;

    void rollingPlanner();
    void IK();
    void transformPlanner();
    void baselinkRotationProcess();
    void groundModeProcess();
    void rosPublishProcess();

    void rosParamInit() override;

    void groundNavigationModeCallback(const std_msgs::Int16Ptr & msg);
    void setFinalTargetBaselinkQuatCallback(const geometry_msgs::QuaternionStampedConstPtr & msg);
    void setFinalTargetBaselinkRpyCallback(const geometry_msgs::Vector3StampedConstPtr & msg);
    void jointsControlCallback(const sensor_msgs::JointStatePtr & msg);
    void ikTargetRelEEPosCallback(const geometry_msgs::Vector3Ptr & msg);
    void joyCallback(const sensor_msgs::JoyConstPtr & joy_msg);

    /* navigation mode */
    int current_ground_navigation_mode_;
    int prev_ground_navigation_mode_;

    /* flight mode variable */
    bool controllers_reset_flag_;

    /* ground mode variable */
    double standing_baselink_roll_converged_thresh_;
    double rolling_pitch_update_thresh_;
    double target_pitch_ang_vel_;
    double target_yaw_ang_vel_;
    double rolling_max_pitch_ang_vel_;
    double rolling_max_yaw_ang_vel_;
    bool pitch_ang_vel_updating_;
    bool yaw_ang_vel_updating_;
    double down_mode_roll_anglvel_;
    double down_start_time_;
    tf::Quaternion down_start_baselink_quat_;

    /* standing mode trajectory generation */
    agi::Polynomial<> poly_;
    bool ground_trajectory_mode_;
    double ground_trajectory_start_time_;
    double ground_trajectory_duration_;

    /* joint transformation */
    bool transforming_flag_;
    double joint_angvel_;
    double transform_finish_time_;
    std::vector<double> transform_target_joint_angles_;
    KDL::Rotation transform_initial_cog_R_contact_link_; // cog_R_contactlink

    /* ik */
    bool ik_solving_flag_;
    int ik_solve_step_;
    std::string ik_cl_name_, ik_ee_name_;
    Eigen::Vector3d ik_target_cl_p_ee_in_initial_cog_;
    KDL::JntArray ik_current_joint_positons_;
    KDL::Rotation ik_initial_cog_R_cl_;

    /* motion planning based on external wrench estimation */
    Eigen::VectorXd est_external_wrench_;
    Eigen::VectorXd est_external_wrench_cog_;

    /* param for joy stick control */
    double joy_stick_deadzone_;

    /* target baselink rotation */
    double prev_rotation_stamp_;
    tf::Quaternion curr_target_baselink_quat_, final_target_baselink_quat_;
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;
    std::string rotation_control_link_name_;
    KDL::Rotation curr_target_cog_R_link_, final_target_cog_R_link_;
  };
};

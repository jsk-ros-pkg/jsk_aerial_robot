// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/gimbalrotor_navigation.h>
#include <beetle/model/beetle_robot_model.h>
#include <diagnostic_msgs/KeyValue.h>

namespace aerial_robot_navigation
{
  /* target frmae*/
  enum target_framea
    {
     COG,
     BASE_LINK,
     CONTACT_POINT,
     CENTER_OF_MOVING
    };
  class BeetleNavigator : public GimbalrotorNavigator
  {
  public:
    BeetleNavigator();
    ~BeetleNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    void update() override;

    inline void setTargetPosCandX( float value){  target_pos_candidate_.setX(value);}
    inline void setTargetPosCandY( float value){  target_pos_candidate_.setY(value);}
    inline void setTargetPosCandZ( float value){  target_pos_candidate_.setZ(value);}
    inline void setTargetPosCand( tf::Vector3 value){  target_pos_candidate_ = value ;}
    inline tf::Vector3 getTargetPosCand() {return target_pos_candidate_;}

  protected:
    void rosParamInit() override;

  private:
    tf::Vector3 target_pos_candidate_;
    tf::Vector3 pre_target_pos_;
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Subscriber assembly_nav_sub_;
    ros::Subscriber assembly_target_rot_sub_;

    double max_target_roll_pitch_rate_;

    bool roll_pitch_control_flag_;
    bool pre_assembled_ ; 

    void naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg) override;
    void assemblyNavCallback(const aerial_robot_msgs::FlightNavConstPtr & msg);
    void setAssemblyFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg);
    void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg) override; 
    void rotateContactPointFrame();
    tf2_ros::TransformBroadcaster br_;
    boost::shared_ptr<BeetleRobotModel> beetle_robot_model_;
    map<string, ros::Subscriber> assembly_flag_subs_;
    void assemblyFlagCallback(const diagnostic_msgs::KeyValue & msg);
    void convertTargetPosFromCoG2CoM();
  };
};

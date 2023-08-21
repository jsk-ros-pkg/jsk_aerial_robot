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
     CONTACT_POINT
    };
  class BeetleNavigator : public GimbalrotorNavigator
  {
  public:
    BeetleNavigator();
    ~BeetleNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator) override;

    void update() override;

  protected:
    void rosParamInit() override;
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    int max_modules_num_;
    void naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg) override;
    void rotateContactPointFrame();
    tf2_ros::TransformBroadcaster br_;
    boost::shared_ptr<BeetleRobotModel> beetle_robot_model_;
    map<string, ros::Subscriber> assembly_flag_subs_;
    map<int, bool> assembly_flags_;
    void assemblyFlagCallback(const diagnostic_msgs::KeyValue & msg);
  };
};

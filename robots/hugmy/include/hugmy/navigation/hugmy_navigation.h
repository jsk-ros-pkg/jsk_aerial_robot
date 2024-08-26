#pragma once

#include <aerial_robot_control/flight_navigation.h>

namespace aerial_robot_navigation
{
  class HugmyNavigator : public BaseNavigator
  {
  public:
    HugmyNavigator();
    virtual ~HugmyNavigator() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;
    void update() override;

  private:
    ros::Subscriber perching_flag_sub_;

    void perchingFlagCallback(const std_msgs::UInt8 & msg);

    bool perching_flag_;
  };
};

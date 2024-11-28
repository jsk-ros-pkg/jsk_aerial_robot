#pragma once

#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <sensor_msgs/Imu.h>
#include <spinal/Imu.h>


namespace sensor_plugin
{
  class LidarInertialOdometry : public SensorBase
  {
  public:
    LidarInertialOdometry(){};
    ~LidarInertialOdometry(){};

    void initialize(ros::NodeHandle nh,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    string sensor_name, int index) override;


  private:
    ros::Subscriber imu_sub_;
    ros::Publisher lidar_imu_pub_;

    std::string lidar_frame_;

    void ImuCallback(const spinal::ImuConstPtr& imu_msg);
  };
};


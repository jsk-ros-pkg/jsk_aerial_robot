//
// Created by li-jinjie on 24-10-28.
//

#include "aerial_robot_estimation/sensor/imu_4_wrench_est.h"

namespace
{
bool first_flag = true;
};

namespace sensor_plugin
{
void Imu4WrenchEst::initialize(ros::NodeHandle nh,
                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                           string sensor_name, int index)
{
  Imu::initialize(nh, robot_model, estimator, std::string("sensor_plugin/imu"), index);

  //low pass filter
  double sample_freq, cutoff_freq;
  getParam<double>("cutoff_freq", cutoff_freq, 20.0);
  getParam<double>("sample_freq", sample_freq, 200.0);
  lpf_omega_ = IirFilter(sample_freq, cutoff_freq, 3);

  // debug
  omega_filter_pub_ = indexed_nhp_.advertise<geometry_msgs::Vector3Stamped>(string("filter_angular_velocity"), 1);

  ROS_INFO("Imu type: Imu4WrenchEst");
}


// override to get filtered gyro data
void Imu4WrenchEst::ImuCallback(const spinal::ImuConstPtr& imu_msg)
{
  imu_stamp_ = imu_msg->stamp;
  tf::Vector3 filtered_omega;

  for(int i = 0; i < 3; i++)
  {
    if(std::isnan(imu_msg->acc_data[i]) || std::isnan(imu_msg->angles[i]) ||
        std::isnan(imu_msg->gyro_data[i]) || std::isnan(imu_msg->mag_data[i]))
    {
      ROS_ERROR_THROTTLE(1.0, "IMU sensor publishes Nan value!");
      return;
    }

    acc_b_[i] = imu_msg->acc_data[i];
    g_b_[i] = imu_msg->angles[i];
    omega_[i] = imu_msg->gyro_data[i];
    mag_[i] = imu_msg->mag_data[i];
  }

  if(first_flag)
  {
    lpf_omega_.setInitValues(omega_);
    first_flag = false;
  }
  filtered_omega = lpf_omega_.filterFunction(omega_);
  geometry_msgs::Vector3Stamped omega_msg;
  omega_msg.header.stamp = imu_msg->stamp;
  tf::vector3TFToMsg(filtered_omega, omega_msg.vector);
  omega_filter_pub_.publish(omega_msg);

  // get filtered angular and linear velocity of CoG
  tf::Transform cog2baselink_tf;
  tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
  int estimate_mode = estimator_->getEstimateMode();
  setFilteredOmegaCog(cog2baselink_tf.getBasis() * filtered_omega);
  setFilteredVelCog(estimator_->getVel(Frame::BASELINK, estimate_mode)
                    + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                          * (filtered_omega.cross(cog2baselink_tf.inverse().getOrigin())));

  estimateProcess();
  updateHealthStamp();
}

};
/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Imu4WrenchEst, sensor_plugin::SensorBase);
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

void Imu4WrenchEst::initialize(ros::NodeHandle nh, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, string sensor_name,
                               int index)
{
  Imu::initialize(nh, robot_model, estimator, std::string("sensor_plugin/imu"), index);

  // debug
  pub_acc_ = indexed_nhp_.advertise<geometry_msgs::AccelStamped>(string("acc_lin_ang_baselink"), 1);
  ROS_INFO("Imu type: Imu4WrenchEst");
}

// override to get filtered gyro data
void Imu4WrenchEst::ImuCallback(const spinal::ImuConstPtr& imu_msg)
{
  imu_stamp_ = imu_msg->stamp;

  for (int i = 0; i < 3; i++)
  {
    if (std::isnan(imu_msg->acc_data[i]) || std::isnan(imu_msg->angles[i]) || std::isnan(imu_msg->gyro_data[i]) ||
        std::isnan(imu_msg->mag_data[i]))
    {
      ROS_ERROR_THROTTLE(1.0, "IMU sensor publishes Nan value!");
      return;
    }

    acc_b_[i] = imu_msg->acc_data[i];
    g_b_[i] = imu_msg->angles[i];
    omega_[i] = imu_msg->gyro_data[i];
    mag_[i] = imu_msg->mag_data[i];
  }

  if (first_flag)
  {
    prev_omega_ = omega_;
    prev_time_ = imu_msg->stamp.toSec();
    first_flag = false;
    return;
  }

  // get omega dot
  tf::Vector3 omega_dot;

  omega_dot = (omega_ - prev_omega_) / (imu_msg->stamp.toSec() - prev_time_);
  prev_omega_ = omega_;
  prev_time_ = imu_msg->stamp.toSec();

  // assign values. TODO: We don't use lpf filter now, since we don't know why using the IIR filter causes 50ms delay.
  tf::Vector3 filtered_acc = acc_b_;
  tf::Vector3 filtered_omega = omega_;
  tf::Vector3 filtered_omega_dot = omega_dot;

  // publish acc
  geometry_msgs::AccelStamped acc_msg;
  acc_msg.header.stamp = imu_msg->stamp;
  tf::vector3TFToMsg(filtered_acc, acc_msg.accel.linear);
  tf::vector3TFToMsg(filtered_omega_dot, acc_msg.accel.angular);
  pub_acc_.publish(acc_msg);

  // get filtered angular and linear velocity of CoG
  tf::Transform cog2baselink_tf;
  tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
  int estimate_mode = estimator_->getEstimateMode();
  setFilteredOmegaCogInCog(cog2baselink_tf.getBasis() * filtered_omega);
  setFilteredVelCogInW(estimator_->getVel(Frame::BASELINK, estimate_mode) +
                       estimator_->getOrientation(Frame::BASELINK, estimate_mode) *
                           (filtered_omega.cross(cog2baselink_tf.inverse().getOrigin())));

  // TODO: this is a simple version of the acceleration estimation. Need to improve.
  setFilteredAccCogInCog(cog2baselink_tf.getBasis() * filtered_acc);
  setFilteredOmegaDotCogInCog(cog2baselink_tf.getBasis() * filtered_omega_dot);

  estimateProcess();
  updateHealthStamp();
}

};  // namespace sensor_plugin
/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Imu4WrenchEst, sensor_plugin::SensorBase);
// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <dragon/sensor/imu.h>

namespace
{
  bool first_flag = true;
};

namespace sensor_plugin
{
  void DragonImu::initialize(ros::NodeHandle nh,
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
  }


  // override to get filtred gyro data
  void DragonImu::ImuCallback(const spinal::ImuConstPtr& imu_msg)
  {
    imu_stamp_ = imu_msg->stamp;
    tf::Vector3 filtered_omega;

    for(int i = 0; i < 3; i++)
      {
        if(std::isnan(imu_msg->acc[i]) || std::isnan(imu_msg->gyro[i]) || std::isnan(imu_msg->mag[i]))
          {
            ROS_ERROR_THROTTLE(1.0, "IMU plugin receives Nan value in IMU sensors !");
            return;
          }

        acc_b_[i] = imu_msg->acc[i];
        omega_[i] = imu_msg->gyro[i];
        mag_[i] = imu_msg->mag[i];
      }

    if(std::isnan(imu_msg->quaternion[0]) || std::isnan(imu_msg->quaternion[1]) ||
       std::isnan(imu_msg->quaternion[2]) || std::isnan(imu_msg->quaternion[3]))
      {
        ROS_ERROR_THROTTLE(1.0, "IMU plugin receives Nan value in Quaternion!");
        return;
      }

    tf::Quaternion raw_q(imu_msg->quaternion[0], imu_msg->quaternion[1],
                         imu_msg->quaternion[2], imu_msg->quaternion[3]);
    raw_rot_ = tf::Matrix3x3(raw_q);


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

    // workaround: use raw roll&pitch omega (not filtered in spinal) for both angular and linear CoG velocity estimation, yaw is still filtered
    // note: this is different with hydrus-like control which use filtered omega for CoG estimation
    omega_.setZ(filtered_omega.z());

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
PLUGINLIB_EXPORT_CLASS(sensor_plugin::DragonImu, sensor_plugin::SensorBase);




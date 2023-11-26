#pragma once

#include <ros/ros.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <state_estimate/state_estimate.h>
#include <tf/LinearMath/Transform.h>
#include <cassert>
#include <limits>

namespace hardware_interface
{
  class MujocoSpinalInterface : public HardwareInterface
  {
  public:
    MujocoSpinalInterface();
    bool init(ros::NodeHandle& nh, int motor_num);

    void setImuValue(double acc_x, double acc_y, double acc_z, double gyro_x, double gyro_y, double gyro_z);
    void setMagValue(double mag_x, double mag_y, double mag_z);
    void setTrueBaselinkOrientation(double q_x, double q_y, double q_z, double q_w);
    void setTrueBaselinkAngular(double w_x, double w_y, double w_z);

    tf::Vector3 getTrueBaselinkRPY();
    inline tf::Vector3 getTrueBaselinkAngular() { return baselink_angular_;}
    tf::Vector3 getTrueCogRPY();
    tf::Vector3 getTrueCogAngular();

    void stateEstimate();
    inline void onGround(bool flag) { on_ground_ = flag; }
    StateEstimate* getEstimatorPtr() {return &spinal_state_estimator_;}
    std::string getName() const{return "";}
    int getMotorNum() {return motor_num_;}
    double getForce(int index){return force_.at(index);}
    void setForce(int index, double force) {force_.at(index) = force;}

  private:
    /* attitude estimator */
    bool on_ground_;

    int motor_num_;
    std::vector<double> force_;
    tf::Matrix3x3 baselink_rot_;
    tf::Vector3 baselink_angular_;
    StateEstimate spinal_state_estimator_;
  };
}

#include <aerial_robot_simulation/mujoco/mujoco_spinal_interface.h>

namespace hardware_interface
{
  MujocoSpinalInterface::MujocoSpinalInterface():
    on_ground_(true)
  {
  }

  bool MujocoSpinalInterface::init(ros::NodeHandle& nh, int motor_num)
  {
    motor_num_ = motor_num;
    force_.resize(motor_num_);
    spinal_state_estimator_.init(&nh);
    return true;
  }


  void MujocoSpinalInterface::stateEstimate()
  {
    if(on_ground_)
      {
        /* assume the robot is static, acc: [0, 0, g] */
        setImuValue(0, 0, aerial_robot_estimation::G, 0, 0, 0);
      }

    spinal_state_estimator_.update();
  }


  void MujocoSpinalInterface::setImuValue(double acc_x, double acc_y, double acc_z, double gyro_x, double gyro_y, double gyro_z)
  {
    spinal_state_estimator_.getAttEstimator()->setAcc(acc_x, acc_y, acc_z);
    spinal_state_estimator_.getAttEstimator()->setGyro(gyro_x, gyro_y, gyro_z);
  }

  void MujocoSpinalInterface::setMagValue(double mag_x, double mag_y, double mag_z)
  {
    spinal_state_estimator_.getAttEstimator()->setMag(mag_x, mag_y, mag_z);
  }

  tf::Vector3 MujocoSpinalInterface::getTrueBaselinkRPY()
  {
    double r,p,y;
    baselink_rot_.getRPY(r, p, y);
    return tf::Vector3(r,p,y);
  }

  tf::Vector3 MujocoSpinalInterface::getTrueCogAngular()
  {
    return spinal_state_estimator_.getAttEstimator()->getDesiredCoordTf() * baselink_angular_;
  }

  tf::Vector3 MujocoSpinalInterface::getTrueCogRPY()
  {
    tf::Matrix3x3 rot = baselink_rot_ * spinal_state_estimator_.getAttEstimator()->getDesiredCoordTf().transpose() ;
    double r,p,y;
    rot.getRPY(r, p, y);
    return tf::Vector3(r,p,y);
  }

  void MujocoSpinalInterface::setTrueBaselinkOrientation(double q_x, double q_y, double q_z, double q_w)
  {
    baselink_rot_.setRotation(tf::Quaternion(q_x, q_y, q_z, q_w));
  }

  void MujocoSpinalInterface::setTrueBaselinkAngular(double w_x, double w_y, double w_z)
  {
    baselink_angular_.setValue(w_x, w_y, w_z);
  }

}

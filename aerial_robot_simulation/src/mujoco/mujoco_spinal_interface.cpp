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

  void MujocoSpinalInterface::setGroundTruthStates(double q_x, double q_y, double q_z, double q_w,
                                                   double w_x, double w_y, double w_z)
  {
    /* directly overwrite the state in attitude estimation */
    ap::Quaternion q(q_w, q_x, q_y, q_z);
    ap::Matrix3f rot; q.rotation_matrix(rot);
    ap::Vector3f ang_vel(w_x, w_y, w_z);
    spinal_state_estimator_.getAttEstimator()->setGroundTruthStates(rot, ang_vel);
  }

}

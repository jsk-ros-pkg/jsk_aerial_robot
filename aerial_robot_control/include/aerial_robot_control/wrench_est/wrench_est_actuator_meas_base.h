//
// Created by jinjie on 24/11/04.
//
// FSM version of wrench_est_actuator_meas_base.h
// - Only uses standard library for the FSM (switch-case).
// - States: STOPPED, CALIBRATING, RUNNING
// - The update() now accepts kinematics: vel, ang_vel, acc, ang_acc (tf::Vector3).
// - Transitions:
//     STOPPED -> CALIBRATING: when |vel| and |acc| are below thresholds.
//     CALIBRATING -> RUNNING: after a fixed duration (e.g., 3s).
//     CALIBRATING/RUNNING -> STOPPED: when |vel| or |acc| exceed thresholds.
// - In RUNNING: getDistForceW() / getDistTorqueCOG() publish (return) the
//   *offset external wrench* obtained from calibration.
// - During STOPPED/CALIBRATING: publish zeros.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_ACTUATOR_MEAS_BASE_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_ACTUATOR_MEAS_BASE_H

#include "aerial_robot_control/wrench_est/wrench_est_base.h"
#include "aerial_robot_estimation/sensor/imu_4_wrench_est.h"
#include "aerial_robot_control/wrench_est/utils.h"

#include <sensor_msgs/JointState.h>
#include "spinal/ESCTelemetryArray.h"
#include "spinal/FourAxisCommand.h"

namespace aerial_robot_control
{

class WrenchEstActuatorMeasBase : public WrenchEstBase
{
public:
  WrenchEstActuatorMeasBase() = default;

  // --- FSM state enum ---
  enum class State
  {
    STOPPED = 0,
    CALIBRATING,
    RUNNING
  };

  void initWrenchPub() override
  {
    pub_disturb_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("dist_w_f_cog_tq/ext", 1);
    pub_disturb_wrench_coeff_ = nh_.advertise<geometry_msgs::WrenchStamped>("dist_w_f_cog_tq/ext_coeff", 1);
  }

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator, double ctrl_loop_du) override
  {
    WrenchEstBase::initialize(nh, robot_model, estimator, ctrl_loop_du);
    ros::NodeHandle wrench_est_nh(nh_, "controller/wrench_est");

    // state machine mode switch
    lin_vel_thresh_calib_.resize(3, 0.0);
    ang_vel_thresh_calib_.resize(3, 0.0);
    wrench_est_nh.getParam("lin_vel_thresh_calib", lin_vel_thresh_calib_);
    wrench_est_nh.getParam("ang_vel_thresh_calib", ang_vel_thresh_calib_);

    lin_vel_thresh_stop_.resize(3, 0.0);
    ang_vel_thresh_stop_.resize(3, 0.0);
    wrench_est_nh.getParam("lin_vel_thresh_stop", lin_vel_thresh_stop_);
    wrench_est_nh.getParam("ang_vel_thresh_stop", ang_vel_thresh_stop_);

    double duration_t;
    getParam<double>(wrench_est_nh, "calib_duration_t", duration_t, 3.0);
    calib_duration_t_ = ros::Duration(duration_t);

    // overall limit
    ext_force_limit_.resize(3, 0.0);
    ext_torque_limit_.resize(3, 0.0);
    wrench_est_nh.getParam("ext_force_limit", ext_force_limit_);
    wrench_est_nh.getParam("ext_torque_limit", ext_torque_limit_);

    // threshold for small noise
    double thresh_force, thresh_torque, steepness_force, steepness_torque;
    getParam<double>(wrench_est_nh, "thresh_force", thresh_force, 0.1);
    getParam<double>(wrench_est_nh, "thresh_torque", thresh_torque, 0.01);
    getParam<double>(wrench_est_nh, "steepness_force", steepness_force, 1);
    getParam<double>(wrench_est_nh, "steepness_torque", steepness_torque, 1);

    coeff_force_.resize(3);
    coeff_torque_.resize(3);
    for (int i = 0; i < 3; ++i)
    {
      coeff_force_[i].initialize(steepness_force, thresh_force);
      coeff_torque_[i].initialize(steepness_torque, thresh_torque);
    }

    // sensors
    ros::NodeHandle motor_nh(nh_, "motor_info");
    getParam<double>(motor_nh, "krpm_square_to_thrust_ratio", krpm_square_to_thrust_ratio_, 0.0);
    getParam<double>(motor_nh, "krpm_square_to_thrust_bias", krpm_square_to_thrust_bias_, 0.0);

    joint_angles_.resize(robot_model_->getJointNum(), 0.0);
    sub_joint_states_ = nh_.subscribe("joint_states", 1, &WrenchEstActuatorMeasBase::callbackJointStates, this);

    thrust_meas_.resize(robot_model_->getRotorNum(), 0.0);
    sub_esc_telem_ = nh_.subscribe("esc_telem", 1, &WrenchEstActuatorMeasBase::callbackESCTelem, this);

    thrust_cmd_.resize(robot_model_->getRotorNum(), 0.0);
    sub_thrust_cmd_ = nh_.subscribe("four_axes/command", 1, &WrenchEstActuatorMeasBase::callbackFourAxisCmd, this);
  }

  void reset() override
  {
    WrenchEstBase::reset();

    calibrator_force_.reset();
    calibrator_torque_.reset();
    for (int i = 0; i < 3; ++i)
    {
      coeff_force_[i].reset();
      coeff_torque_[i].reset();
    }

    // Reset FSM
    enter(State::STOPPED);
  }

  virtual void update(const tf::Vector3& vel, const tf::Vector3& ang_vel)
  {
    switch (state_)
    {
      case State::STOPPED: {
        const bool condition_calib = absElementLessEqualThan(vel, lin_vel_thresh_calib_) &&
                                     absElementLessEqualThan(ang_vel, ang_vel_thresh_calib_);
        if (condition_calib)
        {
          enter(State::CALIBRATING);
        }
        // No output in STOPPED
        break;
      }

      case State::CALIBRATING: {
        const bool condition_stop = !(absElementLessEqualThan(vel, lin_vel_thresh_stop_) &&
                                      absElementLessEqualThan(ang_vel, ang_vel_thresh_stop_));
        if (condition_stop)
        {
          // Abort calibration if motion becomes large
          enter(State::STOPPED);
          break;
        }

        // Accumulate raw external wrench to compute offset
        calibrator_force_.update(raw_dist_force_w_);
        calibrator_torque_.update(raw_dist_torque_cog_);

        // After calibration duration, fix offsets and go RUNNING
        if ((ros::Time::now() - state_enter_time_) >= calib_duration_t_)
        {
          Eigen::Vector3d calib_force = calibrator_force_.getOffsetValue();
          ROS_INFO("The offset force in world frame (N): %d samples, %f, %f, %f", calibrator_force_.getSampleCount(),
                   calib_force(0), calib_force(1), calib_force(2));
          Eigen::Vector3d calib_torque = calibrator_torque_.getOffsetValue();
          ROS_INFO("The offset torque in cog frame (Nm): %d samples, %f, %f, %f", calibrator_torque_.getSampleCount(),
                   calib_torque(0), calib_torque(1), calib_torque(2));

          enter(State::RUNNING);
        }

        break;
      }

      case State::RUNNING: {
        // Leave RUNNING if unstable again
        const bool condition_stop = !(absElementLessEqualThan(vel, lin_vel_thresh_stop_) &&
                                      absElementLessEqualThan(ang_vel, ang_vel_thresh_stop_));

        if (condition_stop)
        {
          enter(State::STOPPED);
          break;
        }

        // update coeff to avoid the influence of too small values.
        Eigen::Vector3d ext_force = calibrator_force_.calibrate(raw_dist_force_w_);
        Eigen::Vector3d ext_torque = calibrator_torque_.calibrate(raw_dist_torque_cog_);
        for (int i = 0; i < 3; ++i)
        {
          coeff_force_[i].updateRMS(ext_force(i));
          coeff_torque_[i].updateRMS(ext_torque(i));
        }

        break;
      }
    }
  }

  // Current state getter
  State state() const
  {
    return state_;
  }

  Eigen::VectorXd calcWrenchFromActuatorMeas(const std::vector<double>& rotor_thrust,
                                             const std::vector<double>& servo_joint) const
  {
    assert(rotor_thrust.size() == robot_model_->getRotorNum());
    assert(servo_joint.size() == robot_model_->getRotorNum());

    Eigen::VectorXd z = Eigen::VectorXd::Zero(2 * robot_model_->getRotorNum());
    for (int i = 0; i < robot_model_->getRotorNum(); i++)
    {
      z(2 * i) = rotor_thrust[i] * sin(servo_joint[i]);
      z(2 * i + 1) = rotor_thrust[i] * cos(servo_joint[i]);
    }

    Eigen::VectorXd est_wrench_cog = alloc_mat_ * z;
    return est_wrench_cog;
  }

  // add offset function
  geometry_msgs::Vector3 getDistForceW() const override
  {
    geometry_msgs::Vector3 dist_force_w_ros;

    if (state_ == State::RUNNING)
    {
      Eigen::Vector3d result = calibrator_force_.calibrate(raw_dist_force_w_);

      // apply impact coefficient
      dist_force_w_ros.x = result(0) * coeff_force_[0].getValue();
      dist_force_w_ros.y = result(1) * coeff_force_[1].getValue();
      dist_force_w_ros.z = result(2) * coeff_force_[2].getValue();

      // apply external force limit
      if (abs(dist_force_w_ros.x) > ext_force_limit_[0] || abs(dist_force_w_ros.y) > ext_force_limit_[1] ||
          abs(dist_force_w_ros.z) > ext_force_limit_[2])
      {
        ROS_WARN_THROTTLE(0.5, "Disturbance force: %.4f, %.4f, %.4f exceeds limit: %.4f, %.4f, %.4f N",
                          dist_force_w_ros.x, dist_force_w_ros.y, dist_force_w_ros.z, ext_force_limit_[0],
                          ext_force_limit_[1], ext_force_limit_[2]);
      }
      dist_force_w_ros.x = std::clamp(dist_force_w_ros.x, -ext_force_limit_[0], ext_force_limit_[0]);
      dist_force_w_ros.y = std::clamp(dist_force_w_ros.y, -ext_force_limit_[1], ext_force_limit_[1]);
      dist_force_w_ros.z = std::clamp(dist_force_w_ros.z, -ext_force_limit_[2], ext_force_limit_[2]);
    }

    return dist_force_w_ros;
  }
  geometry_msgs::Vector3 getDistTorqueCOG() const override
  {
    geometry_msgs::Vector3 dist_torque_cog_ros;

    if (state_ == State::RUNNING)
    {
      Eigen::Vector3d result = calibrator_torque_.calibrate(raw_dist_torque_cog_);

      // apply impact coefficient
      dist_torque_cog_ros.x = result(0) * coeff_torque_[0].getValue();
      dist_torque_cog_ros.y = result(1) * coeff_torque_[1].getValue();
      dist_torque_cog_ros.z = result(2) * coeff_torque_[2].getValue();

      // apply external torque limit
      if (abs(dist_torque_cog_ros.x) > ext_torque_limit_[0] || abs(dist_torque_cog_ros.y) > ext_torque_limit_[1] ||
          abs(dist_torque_cog_ros.z) > ext_torque_limit_[2])
      {
        ROS_WARN_THROTTLE(0.5, "Disturbance torque: %.4f, %.4f, %.4f exceeds limit: %.4f, %.4f, %.4f Nm",
                          dist_torque_cog_ros.x, dist_torque_cog_ros.y, dist_torque_cog_ros.z, ext_torque_limit_[0],
                          ext_torque_limit_[1], ext_torque_limit_[2]);
      }

      dist_torque_cog_ros.x = std::clamp(dist_torque_cog_ros.x, -ext_torque_limit_[0], ext_torque_limit_[0]);
      dist_torque_cog_ros.y = std::clamp(dist_torque_cog_ros.y, -ext_torque_limit_[1], ext_torque_limit_[1]);
      dist_torque_cog_ros.z = std::clamp(dist_torque_cog_ros.z, -ext_torque_limit_[2], ext_torque_limit_[2]);
    }

    return dist_torque_cog_ros;
  }

protected:
  std::vector<double> thrust_meas_;
  std::vector<double> thrust_cmd_;  // for thrust cmd (use this value may be more stable than actual thrust)
  std::vector<double> joint_angles_;

private:
  // --- FSM data ---
  State state_{ State::STOPPED };
  ros::Time state_enter_time_;

  // state switch thresholds
  std::vector<double> lin_vel_thresh_calib_;  // m/s
  std::vector<double> ang_vel_thresh_calib_;  // rad/s
  std::vector<double> lin_vel_thresh_stop_;   // m/s
  std::vector<double> ang_vel_thresh_stop_;   // rad/s

  // calibration duration
  ros::Duration calib_duration_t_;

  // calibration accumulators
  AverageCalibrator calibrator_force_;
  AverageCalibrator calibrator_torque_;

  // --- limit ---
  std::vector<double> ext_force_limit_{ 0.0, 0.0, 0.0 };   // force limit in world frame
  std::vector<double> ext_torque_limit_{ 0.0, 0.0, 0.0 };  // torque limit in cog frame

  // --- threshold function ---
  // We use sigmoid function right now
  std::vector<Sigmoid> coeff_force_{ 3 };
  std::vector<Sigmoid> coeff_torque_{ 3 };

  ros::Publisher pub_disturb_wrench_coeff_;

  // for servo angles
  ros::Subscriber sub_joint_states_;

  // for actual thrust
  ros::Subscriber sub_esc_telem_;
  double krpm_square_to_thrust_ratio_;
  double krpm_square_to_thrust_bias_;

  // for thrust command
  ros::Subscriber sub_thrust_cmd_;

  // called when entering a new state.
  void enter(State next)
  {
    state_ = next;
    state_enter_time_ = ros::Time::now();
    if (state_ == State::CALIBRATING)
    {
      calibrator_force_.reset();
      calibrator_torque_.reset();
      for (int i = 0; i < 3; ++i)
      {
        coeff_force_[i].reset();
        coeff_torque_[i].reset();
      }
    }

    ROS_INFO("Wrench Estimator: Entering state %d", state_);
    ros::NodeHandle wrench_est_nh(nh_, "controller/wrench_est");
    wrench_est_nh.setParam("state", static_cast<int>(state_));
  }

  void callbackPubDistWrench(const ros::TimerEvent& event) const override
  {
    WrenchEstBase::callbackPubDistWrench(event);

    geometry_msgs::WrenchStamped dist_wrench_coeff;
    dist_wrench_coeff.header.stamp = ros::Time::now();
    dist_wrench_coeff.wrench.force.x = coeff_force_[0].getValue();
    dist_wrench_coeff.wrench.force.y = coeff_force_[1].getValue();
    dist_wrench_coeff.wrench.force.z = coeff_force_[2].getValue();
    dist_wrench_coeff.wrench.torque.x = coeff_torque_[0].getValue();
    dist_wrench_coeff.wrench.torque.y = coeff_torque_[1].getValue();
    dist_wrench_coeff.wrench.torque.z = coeff_torque_[2].getValue();
    dist_wrench_coeff.header.frame_id = nh_.getNamespace() + "/cog";
    pub_disturb_wrench_coeff_.publish(dist_wrench_coeff);
  }

  void callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
  {
    for (int i = 0; i < robot_model_->getJointNum(); i++)
      joint_angles_[i] = msg->position[i];
  }

  void callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg)
  {
    double krpm;
    krpm = (double)msg->esc_telemetry_1.rpm * 0.001;
    thrust_meas_[0] = krpm * krpm * krpm_square_to_thrust_ratio_ + krpm_square_to_thrust_bias_;

    krpm = (double)msg->esc_telemetry_2.rpm * 0.001;
    thrust_meas_[1] = krpm * krpm * krpm_square_to_thrust_ratio_ + krpm_square_to_thrust_bias_;

    krpm = (double)msg->esc_telemetry_3.rpm * 0.001;
    thrust_meas_[2] = krpm * krpm * krpm_square_to_thrust_ratio_ + krpm_square_to_thrust_bias_;

    krpm = (double)msg->esc_telemetry_4.rpm * 0.001;
    thrust_meas_[3] = krpm * krpm * krpm_square_to_thrust_ratio_ + krpm_square_to_thrust_bias_;
  }

  void callbackFourAxisCmd(const spinal::FourAxisCommandConstPtr& msg)
  {
    thrust_cmd_[0] = msg->base_thrust[0];
    thrust_cmd_[1] = msg->base_thrust[1];
    thrust_cmd_[2] = msg->base_thrust[2];
    thrust_cmd_[3] = msg->base_thrust[3];
  }

  static bool absElementLessEqualThan(const tf::Vector3& vec, const std::vector<double>& thresh)
  {
    return (abs(vec.x()) < thresh[0]) && (abs(vec.y()) < thresh[1]) && (abs(vec.z()) < thresh[2]);
  }
};

class WrenchEstNone : public WrenchEstActuatorMeasBase
{
  void update(const tf::Vector3& vel, const tf::Vector3& ang_vel) override
  {
    // do nothing
  }
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_ACTUATOR_MEAS_BASE_H

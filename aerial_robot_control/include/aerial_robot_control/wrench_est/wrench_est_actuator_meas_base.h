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

#include "sensor_msgs/JointState.h"
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
    lin_vel_thresh_.resize(3, 0.0);
    ang_vel_thresh_.resize(3, 0.0);
    wrench_est_nh.getParam("lin_vel_threshold", lin_vel_thresh_);
    wrench_est_nh.getParam("ang_vel_threshold", ang_vel_thresh_);

    double duration_t;
    getParam<double>(wrench_est_nh, "calib_duration_t", duration_t, 3.0);
    calib_duration_t_ = ros::Duration(duration_t);

    // threshold for small noise
    getParam<double>(wrench_est_nh, "thresh_force", thresh_force_, 0.1);
    getParam<double>(wrench_est_nh, "thresh_torque", thresh_torque_, 0.01);
    getParam<double>(wrench_est_nh, "steepness_force", steepness_force_, 1);
    getParam<double>(wrench_est_nh, "steepness_torque", steepness_torque_, 1);

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

    calib_offset_sample_count_ = 0;
    calib_offset_force_w_ = Eigen::Vector3d::Zero();
    calib_offset_torque_cog_ = Eigen::Vector3d::Zero();

    // Reset FSM
    state_ = State::STOPPED;
    state_enter_time_ = ros::Time::now();
  }

  virtual void update(const tf::Vector3& vel, const tf::Vector3& ang_vel)
  {
    // Simple magnitude checks for stability
    const bool stable = (vel.x() <= lin_vel_thresh_[0]) && (vel.y() <= lin_vel_thresh_[1]) &&
                        (vel.z() <= lin_vel_thresh_[2]) && (ang_vel.x() <= ang_vel_thresh_[0]) &&
                        (ang_vel.y() <= ang_vel_thresh_[1]) && (ang_vel.z() <= ang_vel_thresh_[2]);

    switch (state_)
    {
      case State::STOPPED: {
        // Transition condition: become stable
        if (stable)
        {
          enter(State::CALIBRATING);
        }
        // No output in STOPPED
        break;
      }

      case State::CALIBRATING: {
        if (!stable)
        {
          // Abort calibration if motion becomes large
          enter(State::STOPPED);
          break;
        }

        // Accumulate raw external wrench to compute offset
        calib_offset_sample_count_++;
        calib_offset_force_w_ =
            calib_offset_force_w_ + (dist_force_w_ - calib_offset_force_w_) / calib_offset_sample_count_;
        calib_offset_torque_cog_ =
            calib_offset_torque_cog_ + (dist_torque_cog_ - calib_offset_torque_cog_) / calib_offset_sample_count_;

        // After calibration duration, fix offsets and go RUNNING
        if ((ros::Time::now() - state_enter_time_) >= calib_duration_t_)
        {
          ROS_INFO("The offset for external wrench -> true, the average offset samples: %lu",
                   calib_offset_sample_count_);
          ROS_INFO("The offset force in world frame (N): %f, %f, %f", calib_offset_force_w_(0),
                   calib_offset_force_w_(1), calib_offset_force_w_(2));
          ROS_INFO("The offset torque in cog frame (Nm): %f, %f, %f", calib_offset_torque_cog_(0),
                   calib_offset_torque_cog_(1), calib_offset_torque_cog_(2));
          enter(State::RUNNING);
        }

        break;
      }

      case State::RUNNING: {
        // Leave RUNNING if unstable again
        if (!stable)
        {
          enter(State::STOPPED);
          break;
        }

        updateWrenchImpactCoeff(dist_force_w_, dist_torque_cog_);

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
      Eigen::Vector3d result = dist_force_w_ - calib_offset_force_w_;

      // apply impact coefficient
      result *= impact_coeff_force_;

      dist_force_w_ros.x = result(0);
      dist_force_w_ros.y = result(1);
      dist_force_w_ros.z = result(2);
    }

    return dist_force_w_ros;
  }
  geometry_msgs::Vector3 getDistTorqueCOG() const override
  {
    geometry_msgs::Vector3 dist_torque_cog_ros;

    if (state_ == State::RUNNING)
    {
      Eigen::Vector3d result = dist_torque_cog_ - calib_offset_torque_cog_;

      // apply impact coefficient
      result *= impact_coeff_torque_;

      dist_torque_cog_ros.x = result(0);
      dist_torque_cog_ros.y = result(1);
      dist_torque_cog_ros.z = result(2);
    }

    return dist_torque_cog_ros;
  }

protected:
  // --- FSM data ---
  State state_{ State::STOPPED };
  ros::Time state_enter_time_;

  // state switch thresholds
  std::vector<double> lin_vel_thresh_;  // m/s
  std::vector<double> ang_vel_thresh_;  // rad/s

  // Calibration duration
  ros::Duration calib_duration_t_;

  // Calibration accumulators
  size_t calib_offset_sample_count_{ 0 };
  Eigen::Vector3d calib_offset_force_w_{ Eigen::Vector3d::Zero() };     // offset estimated force in world frame
  Eigen::Vector3d calib_offset_torque_cog_{ Eigen::Vector3d::Zero() };  // offset estimated torque in cog frame

  // Called when entering a new state.
  void enter(State next)
  {
    state_ = next;
    state_enter_time_ = ros::Time::now();
    if (state_ == State::CALIBRATING)
    {
      // Reset calibration accumulation
      calib_offset_force_w_.setZero();
      calib_offset_torque_cog_.setZero();
      calib_offset_sample_count_ = 0;
    }

    ROS_INFO("Wrench Estimator: Entering state %d", state_);
  }

  // for threshold function
  double thresh_force_ = 0.0;
  double steepness_force_ = 0.0;
  double impact_coeff_force_ = 0.0;

  double thresh_torque_ = 0.0;
  double steepness_torque_ = 0.0;
  double impact_coeff_torque_ = 0.0;

  ros::Publisher pub_disturb_wrench_coeff_;

  void cbPubDistWrench(const ros::TimerEvent& event) const override
  {
    WrenchEstBase::cbPubDistWrench(event);

    geometry_msgs::WrenchStamped dist_wrench_coeff;
    dist_wrench_coeff.header.stamp = ros::Time::now();
    dist_wrench_coeff.wrench.force.x = impact_coeff_force_;
    dist_wrench_coeff.wrench.force.y = impact_coeff_force_;
    dist_wrench_coeff.wrench.force.z = impact_coeff_force_;
    dist_wrench_coeff.wrench.torque.x = impact_coeff_torque_;
    dist_wrench_coeff.wrench.torque.y = impact_coeff_torque_;
    dist_wrench_coeff.wrench.torque.z = impact_coeff_torque_;
    dist_wrench_coeff.header.frame_id = "beetle1/cog";
    pub_disturb_wrench_coeff_.publish(dist_wrench_coeff);
  }

  /* We use sigmoid function right now */
  void updateWrenchImpactCoeff(const Eigen::Vector3d& external_force_w, const Eigen::Vector3d& external_torque_cog)
  {
    const double external_force_norm =
        sqrt(external_force_w(0) * external_force_w(0) + external_force_w(1) * external_force_w(1) +
             external_force_w(2) * external_force_w(2));
    const double external_torque_norm =
        sqrt(external_torque_cog(0) * external_torque_cog(0) + external_torque_cog(1) * external_torque_cog(1) +
             external_torque_cog(2) * external_torque_cog(2));

    // use sigmoid function to limit the external force
    impact_coeff_force_ = 1.0 / (1.0 + exp(-steepness_force_ * (external_force_norm - thresh_force_)));
    impact_coeff_torque_ = 1.0 / (1.0 + exp(-steepness_torque_ * (external_torque_norm - thresh_torque_)));
  }

  // for servo angles
  std::vector<double> joint_angles_;

  ros::Subscriber sub_joint_states_;
  void callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
  {
    for (int i = 0; i < robot_model_->getJointNum(); i++)
      joint_angles_[i] = msg->position[i];
  }

  // for actual thrust
  std::vector<double> thrust_meas_;

  ros::Subscriber sub_esc_telem_;
  double krpm_square_to_thrust_ratio_;
  double krpm_square_to_thrust_bias_;
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

  // for thrust cmd (use this value may be more stable than actual thrust)
  std::vector<double> thrust_cmd_;

  ros::Subscriber sub_thrust_cmd_;
  void callbackFourAxisCmd(const spinal::FourAxisCommandConstPtr& msg)
  {
    thrust_cmd_[0] = msg->base_thrust[0];
    thrust_cmd_[1] = msg->base_thrust[1];
    thrust_cmd_[2] = msg->base_thrust[2];
    thrust_cmd_[3] = msg->base_thrust[3];
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

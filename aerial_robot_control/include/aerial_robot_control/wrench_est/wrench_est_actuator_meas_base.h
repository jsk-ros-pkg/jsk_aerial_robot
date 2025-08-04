//
// Created by jinjie on 24/11/04.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_ACTUATOR_MEAS_BASE_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_ACTUATOR_MEAS_BASE_H

#include "aerial_robot_control/wrench_est/wrench_est_base.h"
#include "aerial_robot_estimation/sensor/imu_4_wrench_est.h"

#include "sensor_msgs/JointState.h"
#include "spinal/ESCTelemetryArray.h"
#include "spinal/FourAxisCommand.h"

namespace digital_filter
{

/**
 * @brief Simple 2-pole (biquad) IIR filter, Direct-Form I.
 *        The internal state is pre-filled with the first sample (see reset())
 *        so that y[0] == x[0], avoiding the usual start-up dip.
 */
class BiquadIIR
{
public:
  BiquadIIR() = default;

  /**
   * @param b     Numerator coefficients {b0, b1, b2}
   * @param a     Denominator coefficients {a0, a1, a2}.  a0 may be != 1;
   *              the function normalises all taps internally so that a0 → 1.
   * @param gain  Optional pre-gain applied to all numerator taps.
   * @throw       std::invalid_argument if a0 is zero (or ~zero).
   */
  void setCoeffs(const std::vector<double>& b, const std::vector<double>& a, double gain = 1.0)
  {
    if (std::fabs(a[0]) < 1e-12)
      throw std::invalid_argument("a0 must not be zero");

    // check that b and a have the right size
    if (b.size() != 3 || a.size() != 3)
      throw std::invalid_argument("BiquadIIR requires 3 coefficients for b and a");

    const double inv_a0 = 1.0 / a[0];

    // Apply gain, then normalise by a0
    for (int i = 0; i < 3; ++i)
      b_[i] = b[i] * gain * inv_a0;

    a1_ = a[1] * inv_a0;
    a2_ = a[2] * inv_a0;
  }

  /**
   * @brief Initialise the delay line so that the very first output equals x0.
   * @param x0  Value used to prime the internal state.
   */
  void reset(double x0 = 0.0)
  {
    x1_ = x0;
    x2_ = x0;
    y1_ = x0;
    y2_ = x0;
    primed_ = true;
  }

  /**
   * @brief Process a single input sample.
   * @param x  New input sample.
   * @return   Filtered output sample.
   */
  double filter(double x)
  {
    if (!primed_)
      reset(x);  // One-shot priming on first call

    // Direct-Form I difference equation:
    const double y = b_[0] * x + b_[1] * x1_ + b_[2] * x2_ - a1_ * y1_ - a2_ * y2_;

    // Shift delay line
    x2_ = x1_;
    x1_ = x;
    y2_ = y1_;
    y1_ = y;

    return y;
  }

  double getLastOutput() const
  {
    return y1_;
  }

private:
  /* Normalised coefficients */
  double b_[3]{ 0.0, 0.0, 0.0 };  // b0, b1, b2
  double a1_{ 0.0 }, a2_{ 0.0 };  // a1, a2   (a0 == 1 after normalisation)

  /* Delay-line state */
  double x1_{ 0.0 }, x2_{ 0.0 };  // x[n-1], x[n-2]
  double y1_{ 0.0 }, y2_{ 0.0 };  // y[n-1], y[n-2]

  bool primed_{ false };  // true once the filter has been initialised
};

}  // namespace digital_filter

namespace aerial_robot_control
{

class WrenchEstActuatorMeasBase : public WrenchEstBase
{
public:
  WrenchEstActuatorMeasBase() = default;

  void initWrenchPub() override
  {
    pub_disturb_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("dist_w_f_cog_tq/ext", 1);
  }

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator, double ctrl_loop_du) override
  {
    WrenchEstBase::initialize(nh, robot_model, estimator, ctrl_loop_du);

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

    is_offset_ = false;
    offset_count_ = 0;
    offset_force_w_ = Eigen::Vector3d::Zero();
    offset_torque_cog_ = Eigen::Vector3d::Zero();
  }

  /* MUST be overridden in derived classes, put it after the disturbance wrench is estimated */
  virtual void update()
  {
    if (!is_offset_)
    {
      // when the is_offset is false, the offset values are updated but not specified to the result.
      offset_count_++;
      offset_force_w_ = offset_force_w_ + (dist_force_w_ - offset_force_w_) / offset_count_;
      offset_torque_cog_ = offset_torque_cog_ + (dist_torque_cog_ - offset_torque_cog_) / offset_count_;
    }
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
    Eigen::Vector3d result = is_offset_ ? (dist_force_w_ - offset_force_w_) : dist_force_w_;

    geometry_msgs::Vector3 dist_force_w_ros;
    dist_force_w_ros.x = result(0);
    dist_force_w_ros.y = result(1);
    dist_force_w_ros.z = result(2);

    return dist_force_w_ros;
  }
  geometry_msgs::Vector3 getDistTorqueCOG() const override
  {
    Eigen::Vector3d result = is_offset_ ? (dist_torque_cog_ - offset_torque_cog_) : dist_torque_cog_;

    geometry_msgs::Vector3 dist_torque_cog_ros;
    dist_torque_cog_ros.x = result(0);
    dist_torque_cog_ros.y = result(1);
    dist_torque_cog_ros.z = result(2);

    return dist_torque_cog_ros;
  }

  bool getOffsetFlag() const
  {
    return is_offset_;
  }
  void toggleOffsetFlag()
  {
    is_offset_ = !is_offset_;

    if (is_offset_)
    {
      ROS_INFO("The offset for external wrench -> true, the average offset samples: %d", offset_count_);
      ROS_INFO("The offset force in world frame (N): %f, %f, %f", offset_force_w_(0), offset_force_w_(1),
               offset_force_w_(2));
      ROS_INFO("The offset torque in cog frame (Nm): %f, %f, %f", offset_torque_cog_(0), offset_torque_cog_(1),
               offset_torque_cog_(2));
    }
    else
      ROS_INFO("The offset for external wrench -> false");
  }

protected:
  // for offset
  bool is_offset_ = false;
  int offset_count_ = 0;
  Eigen::Vector3d offset_force_w_;     // offset estimated force in world frame
  Eigen::Vector3d offset_torque_cog_;  // offset estimated torque in cog frame

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
  void update() override
  {
    // do nothing
  }
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_ACTUATOR_MEAS_BASE_H

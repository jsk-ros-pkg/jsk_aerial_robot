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

// --------------------------------------------------------------------------
//  IIRFilter — transposed DF‑I with gain, vector storage
// --------------------------------------------------------------------------

template <std::size_t M, std::size_t N>
class IIRFilter
{
  static_assert(M > 0 && N > 0, "IIRFilter must have non‑zero taps");

public:
  IIRFilter() = default;

  void setCoeffs(const std::vector<double>& b, const std::vector<double>& a, double gain = 1.0)
  {
    if (b.size() != M || a.size() != N)
      throw std::invalid_argument("IIR taps size mismatch");
    num_ = b;
    for (double& v : num_)
      v *= gain;  // apply pre‑gain only to numerator
    den_ = a;
  }

  void reset(double y0 = 0.0)
  {
    std::fill(z_.begin(), z_.end(), y0);
  }

  [[nodiscard]] constexpr std::size_t order() const noexcept
  {
    return N - 1;
  }

  double filter(double x_n)
  {
    const double a0 = den_[0];
    if (a0 == 0.0)
      throw std::runtime_error("a0 cannot be zero");

    // feedback term Σ a_k * z_{k-1}
    double fb = 0.0;
    for (std::size_t k = 1; k < N; ++k)
      fb += den_[k] * z_[k - 1];

    double w = (x_n - fb) / a0;
    double y = num_[0] * w + z_[0];

    // update states
    for (std::size_t k = 1; k < z_.size(); ++k)
      z_[k - 1] = num_[k] * w + z_[k] + (k < N ? den_[k] * y : 0.0);

    // the last state depends on which side is longer
    std::size_t last = z_.size();
    if constexpr (M > N)
      z_[last - 1] = num_[N] * w;
    else if constexpr (N > M)
      z_[last - 1] = den_[N - 1] * y;
    else
      z_[last - 1] = (num_.back() * w + den_.back() * y);

    return y;
  }

private:
  std::vector<double> num_;  // numerator taps (with gain applied)
  std::vector<double> den_;  // denominator taps (a₀ … a_{N‑1})
  std::vector<double> z_;    // delay‑line state, length max(M,N)‑1
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

//
// Created by li-jinjie on 24-3-1.
//

#ifndef AERIAL_ROBOT_CONTROL_UTILS_H
#define AERIAL_ROBOT_CONTROL_UTILS_H

#include <algorithm>

namespace aerial_robot_control
{
class ITerm
{
public:
  ITerm() = default;
  ~ITerm() = default;

  void initialize(double i_gain, double u_limit, double dt)
  {
    i_gain_ = i_gain;
    u_limit_ = u_limit;
    dt_ = dt;
    i_term_ = 0;
    error_last_round_ = 0;
  }

  void setIGain(double i_gain)
  {
    i_gain_ = i_gain;
  }

  double update(double error)  // y - y_ref
  {
    // update the integrator using trapazoidal rule
    i_term_ += (dt_ / 2) * (error + error_last_round_);

    double u = i_gain_ * i_term_;

    double u_sat = std::max(std::min(u, u_limit_), -u_limit_);

    // integral anti-windup: if u -> saturation, then i_term_ -> saturation / i_gain_
    if (i_gain_ > 0.001)
      i_term_ = u_sat / i_gain_;

    error_last_round_ = error;

    return u_sat;
  }

  void reset()
  {
    i_term_ = 0;
    error_last_round_ = 0;
  }

  /**
   * @brief change the i term by a offset control input u
   * @param u_offset the offset of the control input
   */
  void changeITerm(double u_offset)
  {
    if (i_gain_ > 0.001)
      i_term_ += u_offset / i_gain_;
  }

private:
  double i_gain_;
  double u_limit_;
  double dt_;
  double i_term_;
  double error_last_round_;
};

/**
 * @brief Simple 2-pole (biquad) IIR filter, Direct-Form I.
 *        The internal state is pre-filled with the first sample (see reset())
 *        so that y[0] == x[0], avoiding the usual start-up dip.
 */
class BiquadIIR
{
public:
  BiquadIIR() = default;
  ~BiquadIIR() = default;

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

class Sigmoid
{
public:
  Sigmoid() = default;
  ~Sigmoid() = default;

  // clang-format off
  void initialize(double steepness, double threshold) { steepness_ = steepness; threshold_ = threshold; }
  void reset() { value_ = 0.0; }
  void updateRMS(double x) { update(sqrt(x * x)); }
  void update(double x) { value_ = 1.0 / (1.0 + exp(-steepness_ * (x - threshold_))); }
  double getValue() const { return value_; }
  // clang-format on

private:
  double steepness_{ 0.0 };
  double threshold_{ 0.0 };
  double value_{ 0.0 };
};

class AverageCalibrator
{
public:
  AverageCalibrator() = default;
  ~AverageCalibrator() = default;

  void reset()
  {
    calib_offset_sample_count_ = 0;
    calib_offset_sum_value_.setZero();
  }

  void update(const Eigen::Vector3d& value)
  {
    calib_offset_sum_value_ += value;
    calib_offset_sample_count_++;
  }

  Eigen::Vector3d calibrate(const Eigen::Vector3d& value) const
  {
    return value - getOffsetValue();
  }

  // clang-format off
  int getSampleCount() const { return calib_offset_sample_count_; }
  Eigen::Vector3d getOffsetValue() const { return calib_offset_sum_value_ / calib_offset_sample_count_; }
  // clang-format on

private:
  size_t calib_offset_sample_count_{ 0 };
  Eigen::Vector3d calib_offset_sum_value_{ Eigen::Vector3d::Zero() };
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_UTILS_H

//
// Created by li-jinjie on 24-3-1.
//

#ifndef AERIAL_ROBOT_CONTROL_I_TERM_H
#define AERIAL_ROBOT_CONTROL_I_TERM_H

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

    // integral anti-windup: adjust integrator to keep u out of saturation
    if (i_gain_ != 0)
      i_term_ += (dt_ / i_gain_) * (u_sat - u);

    error_last_round_ = error;

    return u_sat;
  }

  void reset()
  {
    i_term_ = 0;
    error_last_round_ = 0;
  }

private:
  double i_gain_;
  double u_limit_;
  double dt_;
  double i_term_;
  double error_last_round_;
};
}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_I_TERM_H

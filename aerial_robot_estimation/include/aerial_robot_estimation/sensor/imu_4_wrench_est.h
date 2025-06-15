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

#pragma once

#include <aerial_robot_estimation/sensor/imu.h>
#include <geometry_msgs/AccelStamped.h>

using namespace Eigen;
using namespace std;

namespace digital_filter
{

// --------------------------------------------------------------------------
//  FIRFilter — direct‑form FIR, compile‑time length (uses std::vector)
// --------------------------------------------------------------------------

template <std::size_t N>
class FIRFilter
{
  static_assert(N > 0, "FIRFilter length must be > 0");

public:
  FIRFilter() : coeffs_(N, 0.0), hist_(N, 0.0)
  {
  }

  /// Load coefficients and optional gain.
  void setCoeffs(const std::vector<double>& b, double gain = 1.0)
  {
    if (b.size() != N)
      throw std::invalid_argument("FIR taps != N");
    for (std::size_t i = 0; i < N; ++i)
      coeffs_[i] = b[i] * gain;
  }

  void reset()
  {
    primed_ = false;  // reset the primed flag
  }

  [[nodiscard]] constexpr std::size_t order() const noexcept
  {
    return N - 1;
  }

  /// Process one sample.
  double filter(double x_n)
  {
    if (!primed_)
    {
      resetWMeas(x_n);  // if not primed, reset with the first sample
    }

    hist_[idx_] = x_n;  // overwrite oldest sample
    double acc = 0.0;
    std::size_t tap = idx_;
    for (std::size_t k = 0; k < N; ++k)
    {
      acc += coeffs_[k] * hist_[tap];
      tap = (tap == 0) ? N - 1 : tap - 1;  // circular buffer walk
    }
    idx_ = (idx_ + 1) % N;
    return acc;
  }

private:
  bool primed_{ false };        // true if filter is primed
  std::vector<double> coeffs_;  // b₀ … b_{N‑1}
  std::vector<double> hist_;    // circular buffer of past inputs
  std::size_t idx_{};           // write index

  /// Prime the internal delay line so that the first output equals `y0`.
  void resetWMeas(double y0)
  {
    std::fill(hist_.begin(), hist_.end(), y0);
    idx_ = 0;
    primed_ = true;  // primed means the filter is ready to process samples
  }
};

}  // namespace digital_filter

namespace sensor_plugin
{
class Imu4WrenchEst : public sensor_plugin::Imu
{
public:
  void initialize(ros::NodeHandle nh, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, string sensor_name,
                  int index) override;

  bool reset() override;

  void setOmegaCogInCog(const tf::Vector3& omega_cog_in_cog)
  {
    boost::lock_guard<boost::mutex> lock(omega_mutex_);
    omega_cog_in_cog_ = omega_cog_in_cog;
  }

  void setVelCogInW(const tf::Vector3& vel_cog_in_w)
  {
    boost::lock_guard<boost::mutex> lock(vel_mutex_);
    vel_cog_in_w_ = vel_cog_in_w;
  }

  void setOmegaDotCogInCog(const tf::Vector3& omega_dot_cog_in_cog)
  {
    boost::lock_guard<boost::mutex> lock(omega_mutex_);
    omega_dot_cog_in_cog_ = omega_dot_cog_in_cog;
  }

  void setAccCogInCog(const tf::Vector3& acc_cog_in_cog)
  {
    boost::lock_guard<boost::mutex> lock(vel_mutex_);
    acc_cog_in_cog_ = acc_cog_in_cog;
  }

  tf::Vector3 getOmegaCogInCog()
  {
    boost::lock_guard<boost::mutex> lock(omega_mutex_);
    return omega_cog_in_cog_;
  }

  tf::Vector3 getVelCogInW()
  {
    boost::lock_guard<boost::mutex> lock(vel_mutex_);
    return vel_cog_in_w_;
  }

  tf::Vector3 getOmegaDotCogInCog()
  {
    boost::lock_guard<boost::mutex> lock(omega_mutex_);
    return omega_dot_cog_in_cog_;
  }

  tf::Vector3 getAccCogInCog()
  {
    boost::lock_guard<boost::mutex> lock(vel_mutex_);
    return acc_cog_in_cog_;
  }

protected:
  void ImuCallback(const spinal::ImuConstPtr& imu_msg) override;

  // semaphore
  boost::mutex omega_mutex_;
  boost::mutex vel_mutex_;

  // data
  tf::Vector3 vel_cog_in_w_;    // cog point, world frame
  tf::Vector3 acc_cog_in_cog_;  // cog point, cog frame, align with Imu Raw data

  tf::Vector3 omega_cog_in_cog_;  // cog point, cog frame

  std::array<digital_filter::FIRFilter<5>, 3> omega_diff_;  // cog point, cog frame, use FIR filter to smooth the
                                                            // numerical derivative
  tf::Vector3 omega_dot_cog_in_cog_;                        // cog point, cog frame. Use dot means numerical derivative

  // publisher
  ros::Publisher pub_acc_;
};
};  // namespace sensor_plugin

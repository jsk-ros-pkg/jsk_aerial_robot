// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#include <boost/algorithm/clamp.hpp>
#include <string>

using boost::algorithm::clamp;

namespace aerial_robot_control
{
  class PID
  {
  public:

    PID(const std::string name = std::string(""),
        const double p_gain = 0, const double i_gain = 0, const double d_gain = 0,
        const double limit_sum = 1e6, const double limit_p = 1e6, const double limit_i = 1e6, const double limit_d = 1e6,
        const double limit_err_p = 1e6, const double limit_err_i = 1e6, const double limit_err_d = 1e6):
      name_(name), result_(0), err_p_(0), err_i_(0), err_i_prev_(0), err_d_(0),
      target_p_(0), target_d_(0), val_p_(0), val_d_(0),
      p_term_(0), i_term_(0), d_term_(0)
    {
      setGains(p_gain, i_gain, d_gain);
      setLimits(limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d);
    }

    virtual ~PID() = default;

    virtual void update(const double err_p, const double du, const double err_d, const double feedforward_term = 0)
    {
      err_p_ = clamp(err_p, -limit_err_p_, limit_err_p_);
      err_i_prev_ = err_i_;
      err_i_ = clamp(err_i_ + err_p_ * du, -limit_err_i_, limit_err_i_);
      err_d_ = clamp(err_d, -limit_err_d_, limit_err_d_);

      p_term_ = clamp(err_p_ * p_gain_, -limit_p_, limit_p_);
      i_term_ = clamp(err_i_ * i_gain_, -limit_i_, limit_i_);
      d_term_ = clamp(err_d_ * d_gain_, -limit_d_, limit_d_);

      result_ = clamp(p_term_ + i_term_ + d_term_ + feedforward_term, -limit_sum_, limit_sum_);
    }

    const double result() const { return result_; }

    void reset()
    {
      err_i_ = 0;
      err_i_prev_ = 0;
      result_ = 0;
    }

    const double& getPGain() const { return p_gain_; }
    const double& getIGain() const { return i_gain_; }
    const double& getDGain() const { return d_gain_; }

    void setPGain(const double p_gain) { p_gain_ = p_gain; }
    void setIGain(const double i_gain) { i_gain_ = i_gain; }
    void setDGain(const double d_gain) { d_gain_ = d_gain; }

    void setGains(const double p_gain, const double i_gain, const double d_gain)
    {
      setPGain(p_gain);
      setIGain(i_gain);
      setDGain(d_gain);
    }


    const double& getLimitSum() const { return limit_sum_; }
    const double& getLimitP() const { return limit_p_; }
    const double& getLimitI() const { return limit_i_; }
    const double& getLimitD() const { return limit_d_; }
    void setLimitSum(const double limit_sum) {limit_sum_ = limit_sum; }
    void setLimitP(const double limit_p) {limit_p_ = limit_p; }
    void setLimitI(const double limit_i) {limit_i_ = limit_i; }
    void setLimitD(const double limit_d) {limit_d_ = limit_d; }
    void setLimitErrP(const double limit_err_p) {limit_err_p_ = limit_err_p; }
    void setLimitErrI(const double limit_err_i) {limit_err_i_ = limit_err_i; }
    void setLimitErrD(const double limit_err_d) {limit_err_d_ = limit_err_d; }
    void setLimits(const double limit_sum, const double limit_p, const double limit_i, const double limit_d, const double limit_err_p, const double limit_err_i, const double limit_err_d)
    {
      setLimitSum(limit_sum);
      setLimitP(limit_p);
      setLimitI(limit_i);
      setLimitD(limit_d);
      setLimitErrP(limit_err_p);
      setLimitErrI(limit_err_i);
      setLimitErrD(limit_err_d);
    }

    const std::string getName() const { return name_;}
    const double& getErrP() const { return err_p_; }
    const double& getErrI() const { return err_i_; }
    const double& getPrevErrI() const { return err_i_prev_; }
    const double& getErrD() const { return err_d_; }
    void setErrP(const double err_p) { err_p_ = err_p; }
    void setErrI(const double err_i) { err_i_ = err_i; }

    const double& getPTerm() const { return p_term_; }
    const double& getITerm() const { return i_term_; }
    const double& getDTerm() const { return d_term_; }

  protected:

    std::string name_;
    double result_;
    double p_gain_, i_gain_, d_gain_;
    double p_term_, i_term_, d_term_;
    double err_p_, err_i_, err_i_prev_, err_d_;
    double limit_sum_, limit_p_, limit_i_, limit_d_;
    double limit_err_p_, limit_err_i_, limit_err_d_;
    double target_p_, target_d_;
    double val_p_, val_d_;
  };

};

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

/* base class */
#include <kalman_filter/kf_base_plugin.h>

/* plugin */
#include <pluginlib/class_list_macros.h>

namespace kf_plugin
{
  class KalmanFilterBaroBias : public kf_plugin::KalmanFilter
  {
  public:
    KalmanFilterBaroBias(): KalmanFilter() {}

    ~KalmanFilterBaroBias() {}

    void initialize(string name, int id);

    /* be sure that the first parma should be timestamp */
    void getPredictModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& state_transition_model, MatrixXd& control_input_model) const
    {
      state_transition_model = const_state_transition_model_;
      control_input_model = const_control_input_model_;
    }
    void getCorrectModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& observation_model) const
    {
      observation_model = const_observation_model_;
    }
  private:
    MatrixXd const_state_transition_model_;
    MatrixXd const_control_input_model_;
    MatrixXd const_observation_model_;

  };
};

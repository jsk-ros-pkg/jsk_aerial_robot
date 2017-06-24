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
#include <aerial_robot_base/kf/baro_bias_plugin.h>


namespace kf_plugin
{
  void KalmanFilterBaroBias::initialize(ros::NodeHandle nh, string suffix, int id)
  {
    KalmanFilter::initialize(nh, suffix, id);

    /* cfg init */
    server_ = new dynamic_reconfigure::Server<aerial_robot_base::KalmanFilterBaroBiasConfig>(nhp_);
    dynamic_reconf_func_ = boost::bind(&KalmanFilterBaroBias::cfgCallback, this, _1, _2);
    server_->setCallback(dynamic_reconf_func_);

    Matrix<double, 1, 1> state_transition_model;
    state_transition_model << 1;
    setStateTransitionModel(state_transition_model);

    Matrix<double, 1, 1> control_input_model;
    control_input_model << 1;
    setControlInputModel(control_input_model);

    Matrix<double, 1, 1> observation_model;
    observation_model << 1;
    setObservationModel(observation_model);
  }

  void KalmanFilterBaroBias::cfgCallback(aerial_robot_base::KalmanFilterBaroBiasConfig &config, uint32_t level)
  {
    if(config.kalman_filter_flag == true)
      {
        printf("cfg update, node: %s ", (nhp_.getNamespace()).c_str());

        switch(level)
          {
          case 1:  // INPUT_SIGMA = 1
            input_sigma_(0) = config.input_sigma;
            setPredictionNoiseCovariance();
            printf("change the input sigma\n");
            break;
          case 2:  // MEASURE_SIGMA = 2
            measure_sigma_(0)  = config.measure_sigma;
            setMeasurementNoiseCovariance();
            printf("change the measure sigma\n");
            break;
          default :
            printf("\n");
            break;
          }
      }
  }


};

PLUGINLIB_EXPORT_CLASS(kf_plugin::KalmanFilterBaroBias, kf_plugin::KalmanFilter);

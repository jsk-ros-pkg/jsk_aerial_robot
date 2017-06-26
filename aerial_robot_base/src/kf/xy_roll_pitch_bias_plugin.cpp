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
#include <kalman_filter/kf_pos_vel_acc_plugin.h>

/* plugin */
#include <pluginlib/class_list_macros.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_base/KalmanFilterXYBiasConfig.h>

namespace kf_plugin
{

  class KalmanFilterXYBias : public kf_plugin::KalmanFilter
  {
  public:

    /*
      state_dim_ = 6 : p_x, v_x, p_y, v_y, b_roll, b_pitch
      input_dim_ = 5 : a_xb, a_yb, a_zb, d_b_roll(0), d_b_pitch(0)
      measure_dim_ = 2:  p_x + p_y or v_x + v_y
    */

    KalmanFilterXYBias():KalmanFilter(6,5,2) {}
    ~KalmanFilterXYBias() {}

    void initialize(ros::NodeHandle nh, string suffix, int id)
    {

      KalmanFilter::initialize(nh, suffix, id);

      //cfg init
      server_ = new dynamic_reconfigure::Server<aerial_robot_base::KalmanFilterXYBiasConfig>(nhp_);
      dynamic_reconf_func_ = boost::bind(&KalmanFilterXYBias::cfgCallback, this, _1, _2);
      server_->setCallback(dynamic_reconf_func_);
    }

    /* be sure that the first parameter should be timestamp */
    /* core */
    /* params:
       0: dt
       1: roll
       2: pitch
       3: yaw
       4: a_xb
       5: a_yb
       6: a_zb
     */
    void updatePredictModel(const vector<double>& params)
    {
      if(params.size() != 7)
        ROS_INFO("params.size is: %d", params.size());
      assert(params.size() == 7);

      float dt = params[0];

      /* roll + b_roll */
      double S_phy_b = sin(params[1] + estimate_state_[4]);
      double C_phy_b = cos(params[1] + estimate_state_[4]);
      /* pitch + b_pitch */
      double S_theta_b = sin(params[2] + estimate_state_[5]);
      double C_theta_b = cos(params[2] + estimate_state_[5]);
      /* yaw */
      double S_psi = sin(params[3]); double C_psi = cos(params[3]);

      /* acc */
      Vector3d acc; acc << params[4], params[5], params[6];

      /* R && dR */
      double R1 = C_psi * C_theta_b; /* R1 */
      double R2 = C_psi * S_theta_b * S_phy_b - S_psi * C_phy_b; /* R2 */
      double R3 = C_psi * S_theta_b * C_phy_b + S_psi * S_phy_b; /* R3 */
      double R4 = S_psi * C_theta_b; /* R4 */
      double R5 = S_psi * S_theta_b * S_phy_b + C_psi * C_phy_b; /* R5 */
      double R6 = S_psi * S_theta_b * C_phy_b - C_psi * S_phy_b; /* R6 */

      Vector3d dR123_dr;
      dR123_dr <<
        0, /* dR1_dbp */
        C_psi * S_theta_b * C_phy_b - S_psi * (-S_phy_b), /* dR2_dbr */
        C_psi * S_theta_b * (-S_phy_b) + S_psi * C_phy_b; /* dR3_dbr */

      Vector3d dR123_dp;
      dR123_dp <<
        C_psi * (-S_theta_b), /* dR1_dbp */
        C_psi * C_theta_b * S_phy_b, /* dR2_dbp */
        C_psi * C_theta_b * C_phy_b; /* dR3_dbp */

      Vector3d dR456_dr;
      dR456_dr <<
        0, /* dR4_dbr */
        S_psi * S_theta_b * C_phy_b + C_psi * (-S_phy_b), /* dR5_dbr */
        S_psi * S_theta_b * (-S_phy_b) - C_psi * C_phy_b; /* dR6_dbr */

      Vector3d dR456_dp;
      dR456_dp <<
        S_psi * (-S_theta_b), /* dR4_dbp */
        S_psi * C_theta_b * S_phy_b, /* dR5_dbp */
        S_psi * C_theta_b * C_phy_b; /* dR6_dbp */

      Matrix<double, 6, 6> state_transition_model = MatrixXd::Identity(6, 6);
      state_transition_model(0,1) = dt;
      state_transition_model(2,3) = dt;

      state_transition_model(0,4) = 0.5 * dt * dt  * dR123_dr.dot(acc);
      state_transition_model(0,5) = 0.5 * dt * dt  * dR123_dp.dot(acc);

      state_transition_model(1,4) = dt  * dR123_dr.dot(acc);
      state_transition_model(1,5) = dt  * dR123_dp.dot(acc);

      state_transition_model(2,4) = 0.5 * dt * dt  * dR456_dr.dot(acc);
      state_transition_model(2,5) = 0.5 * dt * dt  * dR456_dp.dot(acc);

      state_transition_model(3,4) = dt  * dR456_dr.dot(acc);
      state_transition_model(3,5) = dt  * dR456_dp.dot(acc);

      setStateTransitionModel(state_transition_model);

      Matrix<double, 6, 5> control_input_model = MatrixXd::Zero(6, 5);
      control_input_model(0, 0) = 0.5 * dt * dt  * R1;
      control_input_model(0, 1) = 0.5 * dt * dt  * R2;
      control_input_model(0, 2) = 0.5 * dt * dt  * R3;
      control_input_model(1, 0) = dt  * R1;
      control_input_model(1, 1) = dt  * R2;
      control_input_model(1, 2) = dt  * R3;

      control_input_model(2, 0) = 0.5 * dt * dt  * R4;
      control_input_model(2, 1) = 0.5 * dt * dt  * R5;
      control_input_model(2, 2) = 0.5 * dt * dt  * R6;
      control_input_model(3, 0) = dt  * R4;
      control_input_model(3, 1) = dt  * R5;
      control_input_model(3, 2) = dt  * R6;

      control_input_model(4, 3) = 1;
      control_input_model(5, 4) = 1;

      setControlInputModel(control_input_model);
    }

    /* be sure that the first parma should be timestamp */
    void updateCorrectModel(const vector<double>& params)
    {
      /* params: correct mode */
      assert(params.size() == 1);
      assert((int)params[0] <= VEL);

      Matrix<double, 2, 6> observation_model;
      switch((int)params[0])
        {
        case POS:
          {
            observation_model <<
              1, 0, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0;
            break;
          }
        case VEL:
          {
            observation_model <<
              0, 1, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0;
            break;
          }
        default:
          {
            ROS_ERROR("xy bias ekf: wrong mode");
            return;
            break;
          }
        }

      setObservationModel(observation_model);

      /*
      Matrix<double, 2, 1> meas; meas << val_x, val_y;
      */
    }

  private:
    //dynamic reconfigure
    dynamic_reconfigure::Server<aerial_robot_base::KalmanFilterXYBiasConfig>* server_;
    dynamic_reconfigure::Server<aerial_robot_base::KalmanFilterXYBiasConfig>::CallbackType dynamic_reconf_func_;

    void cfgCallback(aerial_robot_base::KalmanFilterXYBiasConfig &config, uint32_t level)
    {
      if(config.kalman_filter_flag == true)
        {
          printf("cfg update, node: %s ", (nhp_.getNamespace()).c_str());

          switch(level)
            {
            case 1:  // INPUT_SIGMA_ACC = 1
              input_sigma_(0) = config.acc_sigma;
              input_sigma_(1) = config.acc_sigma;
              input_sigma_(2) = config.acc_sigma;
              setPredictionNoiseCovariance();
              printf("change the input(acc) sigma\n");
              break;
            case 2:  // BIAS_SIGMA_ACC = 1
              input_sigma_(3) = config.bias_sigma;
              input_sigma_(4) = config.bias_sigma;
              setPredictionNoiseCovariance();
              printf("change the input(roll pitch bais) sigma\n");
              break;
            case 3:  // MEASURE_SIGMA = 3
              measure_sigma_(0)  = config.measure_sigma;
              measure_sigma_(1)  = config.measure_sigma;
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
};

PLUGINLIB_EXPORT_CLASS(kf_plugin::KalmanFilterXYBias, kf_plugin::KalmanFilter);

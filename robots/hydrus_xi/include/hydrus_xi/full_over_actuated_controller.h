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

#ifndef OVER_FULL_ACTUATED_CONTROLLER
#define OVER_FULL_ACTUATED_CONTROLLER

#include <ros/ros.h>
#include <hydrus/transform_control.h>
#include <hydrus_xi/QMatrixPseudoInverseInertia.h>

#include <memory>

/* basic control class */
#include <aerial_robot_base/control/flatness_pid_controller.h>

namespace control_plugin
{
  class FullOverActuatedController: public control_plugin::FlatnessPid
  {
  public:
    FullOverActuatedController();
    ~FullOverActuatedController();

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    BasicEstimator* estimator, Navigator* navigator,
                    double ctrl_loop_rate) override;
    void pidUpdate() override;
    void sendCmd() override;
  private:
    std::shared_ptr<TransformController> transform_controller_;
    tf::Vector3 target_linear_acc_;
    Eigen::MatrixXd Q_pseudo_inv_; //propeller force to wrench in CoG projection matrix
    void calcQPseudoInv();
    void calcForceVector(std::vector<float>& force);

    std::string q_matrix_pseudo_inverse_inertia_pub_topic_name_;
    ros::Publisher q_matrix_pseudo_inverse_inertia_pub_;

    int msg_pub_prescaler_;
    int msg_pub_cnt_;

    /* psuedo inverse */
    /* https://gist.github.com/javidcf/25066cf85e71105d57b6 */
    template <class MatT>
    Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
    pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
    {
      typedef typename MatT::Scalar Scalar;
      auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
      const auto &singularValues = svd.singularValues();
      Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
      singularValuesInv.setZero();
      for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
          {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
          }
        else
          {
            singularValuesInv(i, i) = Scalar{0};
          }
      }
      return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
    }
  };
} //namespace control_plugin

#endif //ifndef OVER_FULL_ACTUATED_CONTROLLER

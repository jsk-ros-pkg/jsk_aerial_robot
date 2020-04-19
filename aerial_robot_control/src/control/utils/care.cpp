// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, JSK Lab
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

#include <aerial_robot_control/control/utils/care.h>

namespace control_utils
{
  /* Continuous-time Algebraic Riccati Equation */
  bool care(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q, Eigen::MatrixXd& P, Eigen::MatrixXd& K)
  {
    /* hamiltonMatrixSolver */
    int state_dim = A.rows();
    int input_dim = B.cols();
    Eigen::MatrixXd R_inv = R.inverse();

    Eigen::MatrixXcd H = Eigen::MatrixXcd::Zero(2 * state_dim, 2 * state_dim);
    H.block(0,0, state_dim, state_dim) = A.cast<std::complex<double> >();
    H.block(state_dim, 0, state_dim, state_dim) = -(Q.cast<std::complex<double> >());
    H.block(0, state_dim, state_dim, state_dim) = - (B * R_inv * B.transpose()).cast<std::complex<double> >();
    H.block(state_dim, state_dim, state_dim, state_dim) = - (A.transpose()).cast<std::complex<double> >();

    Eigen::ComplexEigenSolver<Eigen::MatrixXcd> ces;
    ces.compute(H);

    Eigen::MatrixXcd phy = Eigen::MatrixXcd::Zero(2 * state_dim, state_dim);
    int j = 0;

    for(int i = 0; i < 2 * state_dim; i++)
      {
        if(ces.eigenvalues()[i].real() < 0)
          {
            if(j >= state_dim)
              {
                std::cout << RED_MESSAGE << "Error in care: nagative sigular amount is larger" << RESET_COLOR << std::endl;
                return false;
              }

            phy.col(j) = ces.eigenvectors().col(i);
            j++;
          }
      }

    if(j != state_dim)
      {
        std::cout << RED_MESSAGE << "Error in care: nagative sigular value amount is not enough" << RESET_COLOR << std::endl;
        return false;
      }

    Eigen::MatrixXcd f = phy.block(0, 0, state_dim, state_dim);
    Eigen::MatrixXcd g = phy.block(state_dim, 0, state_dim, state_dim);

    Eigen::MatrixXcd f_inv  = f.inverse();
    P = (g * f_inv).real();
    //K
    K = -R_inv * B.transpose() * P;

    return true;
  }
}


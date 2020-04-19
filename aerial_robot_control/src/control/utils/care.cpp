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
  bool care(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q, Eigen::MatrixXd& P, Eigen::MatrixXd& K, const bool iterative_solution, const double converge_thresh, const int max_iteration)
  {

    Eigen::MatrixXd R_inv = R.inverse();
    const int state_dim = A.rows();
    const int input_dim = B.cols();

    if(iterative_solution)
      {
        /* Kleinman method */
        if (K.cols() != state_dim || K.rows() != input_dim)
          {
            std::cout << RED_MESSAGE << "Error in care: the init K has wrong size ( " << K.rows() << ", " << K.cols() << "),  can not run Kleinman method" << RESET_COLOR << std::endl;
            return false;
          }

        double max_diff;
        for(int i = 0; i < max_iteration; i++)
          {
            /* Lyapunov equation */
            Eigen::MatrixXd A_BK = A + B * K;

            Eigen::ComplexSchur<Eigen::MatrixXd> SchurA_BK(A_BK);
            Eigen::MatrixXcd A_BK_T = SchurA_BK.matrixT();
            Eigen::MatrixXcd A_BK_U = SchurA_BK.matrixU();

            Eigen::ComplexSchur<Eigen::MatrixXd> SchurA_BKt(A_BK.transpose());
            Eigen::MatrixXcd A_BKt_T = SchurA_BKt.matrixT();
            Eigen::MatrixXcd A_BKt_U = SchurA_BKt.matrixU();

            Eigen::MatrixXd Q_KtNK = Q + K.transpose() * R * K;
            Eigen::MatrixXcd F = (A_BKt_U.adjoint() * (-Q_KtNK)) * A_BK_U;
            Eigen::MatrixXcd Y = Eigen::internal::matrix_function_solve_triangular_sylvester(A_BKt_T, A_BK_T, F);

            Eigen::MatrixXd K_prev = K;
            P = ((A_BKt_U * Y) * A_BK_U.adjoint()).real();
            K = -R_inv * B.transpose() * P;

            Eigen::MatrixXd delta_K = K - K_prev;
            max_diff = fabs(delta_K.maxCoeff()) > fabs(delta_K.minCoeff())?fabs(delta_K.maxCoeff()):fabs(delta_K.minCoeff());

            // std::cout << "Care in Kleinman method, iteration: " << i << ", max diff: " << max_diff << std::endl;

            if(max_diff < converge_thresh)
              {
                // std::cout << BLUE_MESSAGE << "Care in Kleinman method, iteration: " << i << ", converge" << RESET_COLOR << std::endl;

#if 0 //debug
                /* compare with hamiltonMatrixSolver */
                Eigen::MatrixXd K_temp;
                Eigen::MatrixXd P_temp;
                care(A, B, R, Q, P_temp, K_temp);
                delta_K = K - K_temp;
                max_diff = fabs(delta_K.maxCoeff()) > fabs(delta_K.minCoeff())?fabs(delta_K.maxCoeff()):fabs(delta_K.minCoeff());
                std::cout << "Care in Kleinman method,  compared with hamiltonMatrixSolver, max diff: ";
                if(max_diff > converge_thresh)
                  std::cout << YELLOW_MESSAGE << max_diff << RESET_COLOR << std::endl;
                else
                  std::cout << max_diff << std::endl;
#endif
                return true;
              }
          }

        std::cout << YELLOW_MESSAGE << "Warning in care: K does not converge in Kleinman method, max matrix element diff is " << max_diff << ", use hamiltonMatrixSolver" << RESET_COLOR << std::endl;

        return care(A, B, R, Q, P, K);
      }
    else
      {
        /* hamiltonMatrixSolver */

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
      }
    return true;
  }
}


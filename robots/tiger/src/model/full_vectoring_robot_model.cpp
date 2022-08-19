// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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

#include <tiger/model/full_vectoring_robot_model.h>
#include <OsqpEigen/OsqpEigen.h>

using namespace Tiger;


FullVectoringRobotModel::FullVectoringRobotModel(bool init_with_rosparam, bool verbose, double edf_radius, double edf_max_tilt) :
  Dragon::FullVectoringRobotModel(init_with_rosparam, verbose, edf_radius, edf_max_tilt)
{
  ros::NodeHandle nh;

  nh.param("joint_torque_limit", joint_torque_limit_, 3.0);
  nh.param("init_untouch_leg", untouch_leg_, -1);

  static_vectoring_f_ = Eigen::VectorXd::Zero(0);
  static_joint_t_ = Eigen::VectorXd::Zero(0);
}

void FullVectoringRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  Dragon::FullVectoringRobotModel::updateRobotModelImpl(joint_positions);

  KDL::JntArray gimbal_processed_joint = getGimbalProcessedJoint<KDL::JntArray>();
  std::vector<KDL::Rotation> links_rotation_from_cog = getLinksRotationFromCog<KDL::Rotation>();

  calcBasicKinematicsJacobian();
  const auto& thrust_coord_jacobians = getThrustCoordJacobians();
  const int joint_num = getJointNum();
  const int link_joint_num = getLinkJointIndices().size();
  const int rotor_num = getRotorNum();
  const int fr_ndof = 3 * rotor_num;

  Eigen::MatrixXd A1_fr_all = Eigen::MatrixXd::Zero(joint_num, fr_ndof);
  Eigen::MatrixXd A2_fr = Eigen::MatrixXd::Zero(6, fr_ndof);

  for (int i = 0; i < rotor_num; i++) {
    Eigen::MatrixXd a = -thrust_coord_jacobians.at(i).topRows(3).rightCols(joint_num).transpose();
    Eigen::MatrixXd r = aerial_robot_model::kdlToEigen(links_rotation_from_cog.at(i));
      // describe force w.r.t. local (link) frame
      A1_fr_all.middleCols(3 * i, 3) = a * r;
      A2_fr.middleCols(3 * i, 3) = thrust_coord_jacobians.at(i).topRows(3).leftCols(6).transpose() * r;
  }

  const int leg_num = rotor_num / 2;
  if (untouch_leg_ >= leg_num)
    {
      ROS_WARN_ONCE("The untouch leg ID exceeds the total leg number, reset to -1");
      untouch_leg_ = -1;
    }
  const int fe_num =  leg_num - (int)(untouch_leg_ >= 0);
  const int fe_ndof = 3 * fe_num;

  Eigen::MatrixXd A1_fe_all = Eigen::MatrixXd::Zero(joint_num, fe_ndof);
  Eigen::MatrixXd A2_fe = Eigen::MatrixXd::Zero(6, fe_ndof);
  std::vector<Eigen::MatrixXd> ee_coord_jacobians;
  int cnt = 0;
  for (int i = 0; i < leg_num; i++) {

    if (untouch_leg_ == i) continue;

    std::string name = std::string("link") + std::to_string((i + 1) *2) + std::string("_foot");

    Eigen::MatrixXd jac = RobotModel::getJacobian(gimbal_processed_joint, name);
    ee_coord_jacobians.push_back(jac);

    A1_fe_all.middleCols(3 * cnt, 3) = -jac.topRows(3).rightCols(joint_num).transpose();
    A2_fe.middleCols(3 * cnt, 3) = jac.topRows(3).leftCols(6).transpose();

    cnt ++;
  }


  Eigen::VectorXd b1_all = Eigen::VectorXd::Zero(joint_num);
  Eigen::VectorXd b2 = Eigen::VectorXd::Zero(6);
  for(const auto& inertia : getInertiaMap()) {

    Eigen::MatrixXd cog_coord_jacobian
      = RobotModel::getJacobian(gimbal_processed_joint, inertia.first, inertia.second.getCOG());

    b1_all
      -= cog_coord_jacobian.rightCols(joint_num).transpose()
      * inertia.second.getMass() * (-getGravity());

    b2
      += cog_coord_jacobian.leftCols(6).transpose()
      * inertia.second.getMass() * (-getGravity());
  }

  // only consider link joint
  Eigen::MatrixXd A1_fe = Eigen::MatrixXd::Zero(link_joint_num, fe_ndof);
  Eigen::MatrixXd A1_fr = Eigen::MatrixXd::Zero(link_joint_num, fr_ndof);
  Eigen::VectorXd b1 = Eigen::VectorXd::Zero(link_joint_num);

  cnt = 0;
  for(int i = 0; i < joint_num; i++) {
    if(getJointNames().at(i) == getLinkJointNames().at(cnt))
      {
        A1_fe.row(cnt) = A1_fe_all.row(i);
        A1_fr.row(cnt) = A1_fr_all.row(i);
        b1(cnt) = b1_all(i);
        cnt++;
      }
    if(cnt == link_joint_num) break;
  }


  if (untouch_leg_ == -1) {
      // 1. only use joint torque for stand
      OsqpEigen::Solver qp_solver;
      qp_solver.settings()->setVerbosity(false);
      qp_solver.settings()->setWarmStart(true);
      /*** set the initial data of the QP solver ***/
      qp_solver.data()->setNumberOfVariables(fe_ndof);
      qp_solver.data()->setNumberOfConstraints(6);

      /*
        cost function:
        (A1_fe fe + b1)^T (A1_fe fe + b1)
        = f^T A1_fe^T A1_fe f + 2 b1^T A1_fe f + cons
      */
      Eigen::MatrixXd hessian = A1_fe.transpose() * A1_fe;
      Eigen::SparseMatrix<double> hessian_sparse = hessian.sparseView();
      qp_solver.data()->setHessianMatrix(hessian_sparse);

      Eigen::VectorXd gradient = b1.transpose() * A1_fe;
      qp_solver.data()->setGradient(gradient);

      /* equality constraint: zero total wnrech */
      Eigen::SparseMatrix<double> constraint_sparse = (-A2_fe).sparseView();
      qp_solver.data()->setLinearConstraintsMatrix(constraint_sparse);
      qp_solver.data()->setLowerBound(b2);
      qp_solver.data()->setUpperBound(b2);

      if(!qp_solver.initSolver()) {
        ROS_ERROR("can not initialize qp solver");
      }

      bool res = qp_solver.solve();
      if(!res) {
        ROS_ERROR("can not solve QP");
      }
      else {
        Eigen::VectorXd fe = qp_solver.getSolution();
        Eigen::VectorXd tor = b1 + A1_fe * fe;
        ROS_INFO_STREAM_ONCE("[QP1] Fe: " << fe.transpose());
        ROS_INFO_STREAM_ONCE("[QP1] Joint Torque: " << tor.transpose());
        ROS_INFO_STREAM_ONCE("[QP1] Wrench: " << (A2_fe * fe + b2).transpose());
      }
  }

  // 2. use thrust force and joint torque for standing, cost func for joint torque
  OsqpEigen::Solver qp_solver2;
  qp_solver2.settings()->setVerbosity(false);
  qp_solver2.settings()->setWarmStart(true);
  qp_solver2.data()->setNumberOfVariables(fr_ndof + fe_ndof);
  qp_solver2.data()->setNumberOfConstraints(6);

  /*
    cost function:
    f_all^T W1 f_all + (A1 f_all + b1)^T W2 (A1 f_all + b1)
    = f^T (W1 + A1^T W2 A1) f + 2 b1^T W2 A1 f + cons
  */
  Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(link_joint_num, fr_ndof + fe_ndof);
  A1.leftCols(fr_ndof) = A1_fr;
  A1.rightCols(fe_ndof) = A1_fe;

  Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(6, fr_ndof + fe_ndof);
  A2.leftCols(fr_ndof) = A2_fr;
  A2.rightCols(fe_ndof) = A2_fe;

  Eigen::MatrixXd W1 = Eigen::MatrixXd::Zero(fr_ndof + fe_ndof, fr_ndof + fe_ndof);
  W1.topLeftCorner(fr_ndof, fr_ndof) = thrust_force_weight_ * Eigen::MatrixXd::Identity(fr_ndof, fr_ndof);
  Eigen::MatrixXd W2 = joint_torque_weight_ * Eigen::MatrixXd::Identity(link_joint_num, link_joint_num);

  Eigen::MatrixXd hessian2 = W1 + A1.transpose() * W2 * A1;
  Eigen::SparseMatrix<double> hessian_sparse2 = hessian2.sparseView();
  qp_solver2.data()->setHessianMatrix(hessian_sparse2);

  Eigen::VectorXd gradient2 = b1.transpose() * W2 * A1;
  qp_solver2.data()->setGradient(gradient2);

  /* equality constraint: zero total wnrech */
  Eigen::SparseMatrix<double> constraint_sparse2 = (-A2).sparseView();
  qp_solver2.data()->setLinearConstraintsMatrix(constraint_sparse2);
  qp_solver2.data()->setLowerBound(b2);
  qp_solver2.data()->setUpperBound(b2);

  if(!qp_solver2.initSolver()) {
    ROS_ERROR("can not initialize qp solver");
  }

  double s_t = ros::Time::now().toSec();
  bool res = qp_solver2.solve();
  ROS_INFO_STREAM_ONCE("QP2 solve: " << ros::Time::now().toSec() - s_t);
  if(!res) {
    ROS_ERROR("can not solve QP");
  }
  else {

    Eigen::VectorXd f_all = qp_solver2.getSolution();

    ROS_INFO_STREAM_ONCE("[QP2] Thrust force for stand: " << f_all.head(fr_ndof).transpose());
    ROS_INFO_STREAM_ONCE("[QP2] Contact force for stand: " << f_all.tail(fe_ndof).transpose());
    ROS_INFO_STREAM_ONCE("[QP2] Joint Torque: " << (A1 * f_all + b1).transpose());
    ROS_INFO_STREAM_ONCE("[QP2] Wrench: " << (A2 * f_all + b2).transpose());
  }

  // 3. use thrust force and joint torque, constraint for joint torque
  OsqpEigen::Solver qp_solver3;
  qp_solver3.settings()->setVerbosity(false);
  qp_solver3.settings()->setWarmStart(true);
  qp_solver3.data()->setNumberOfVariables(fr_ndof + fe_ndof);
  qp_solver3.data()->setNumberOfConstraints(6 + link_joint_num);

  /*
    cost function: f_all^T W1 f_all
  */
  Eigen::MatrixXd hessian3 = W1;
  Eigen::SparseMatrix<double> hessian_sparse3 = hessian3.sparseView();
  qp_solver3.data()->setHessianMatrix(hessian_sparse3);

  Eigen::VectorXd gradient3 = Eigen::VectorXd::Zero(fr_ndof + fe_ndof);
  qp_solver3.data()->setGradient(gradient3);

  /* equality constraint: zero total wnrech */
  Eigen::MatrixXd constraints = Eigen::MatrixXd::Zero(6 + link_joint_num, fr_ndof + fe_ndof);
  constraints.topRows(6) = A2;
  constraints.bottomRows(link_joint_num) = A1;
  Eigen::SparseMatrix<double> constraint_sparse3 = constraints.sparseView();
  qp_solver3.data()->setLinearConstraintsMatrix(constraint_sparse3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(6 + link_joint_num);
  b.head(6) = -b2;
  b.tail(link_joint_num) = -b1;
  Eigen::VectorXd max_torque = Eigen::VectorXd::Zero(6 + link_joint_num);
  max_torque.tail(link_joint_num) = joint_torque_limit_ * Eigen::VectorXd::Ones(link_joint_num);

  Eigen::VectorXd lower_bound = b - max_torque;
  Eigen::VectorXd upper_bound = b + max_torque;
  qp_solver3.data()->setLowerBound(lower_bound);
  qp_solver3.data()->setUpperBound(upper_bound);

  if(!qp_solver3.initSolver()) {
    ROS_ERROR("can not initialize qp solver");
  }

  s_t = ros::Time::now().toSec();
  res = qp_solver3.solve();
  ROS_INFO_STREAM_ONCE("QP3 solve: " << ros::Time::now().toSec() - s_t);
  if(!res) {
    ROS_ERROR("can not solve QP");
  }
  else {

    Eigen::VectorXd f_all_neq = qp_solver3.getSolution();
    Eigen::VectorXd fr = f_all_neq.head(fr_ndof);
    Eigen::VectorXd fe = f_all_neq.tail(fe_ndof);
    Eigen::VectorXd tor = A1 * f_all_neq + b1;

    ROS_INFO_STREAM_ONCE("[QP3] Thrust force for stand: " << fr.transpose());
    ROS_INFO_STREAM_ONCE("[QP3] Contact force for stand: " << fe.transpose());
    ROS_INFO_STREAM_ONCE("[QP3] Joint Torque: " << tor.transpose());
    ROS_INFO_STREAM_ONCE("[QP3] Wrench: " << (A2 * f_all_neq + b2).transpose());

    // ROS_INFO_STREAM_THROTTLE(1.0, "[QP3] Thrust force for stand: " << fr.transpose());
    // ROS_INFO_STREAM_THROTTLE(1.0, "[QP3] Contact force for stand: " << fe.transpose());
    // ROS_INFO_STREAM_THROTTLE(1.0, "[QP3] Joint Torque: " << (A1 * f_all_neq + b1).transpose());
    // ROS_INFO_STREAM_THROTTLE(1.0, "[QP3] Wrench: " << (A2 * f_all_neq + b2).transpose());

    Eigen::VectorXd lambda = Eigen::VectorXd::Zero(rotor_num);
    for(int i = 0; i < rotor_num; i++) {
      lambda(i) = fr.segment(3 * i, 3).norm();
    }

    ROS_INFO_STREAM_ONCE("[QP3] Thrust force lambda: " << lambda.transpose());
    // ROS_INFO_STREAM_THROTTLE(1.0, "[QP3] Thrust force lambda: " << lambda.transpose());

    // setStaticVectoringF(fr);
    // setStaticJointT(tor);
  }

  // 4. use thrust force and joint torque, cost and constraint for joint torque
  OsqpEigen::Solver qp_solver4;
  qp_solver4.settings()->setVerbosity(false);
  qp_solver4.settings()->setWarmStart(true);
  qp_solver4.data()->setNumberOfVariables(fr_ndof + fe_ndof);
  qp_solver4.data()->setNumberOfConstraints(6 + link_joint_num);

  /*
    cost function:
    f_all^T W1 f_all + (A1 f_all + b1)^T W2 (A1 f_all + b1)
    = f^T (W1 + A1^T W2 A1) f + 2 b1^T A1 f + cons
  */
  qp_solver4.data()->setHessianMatrix(hessian_sparse2);
  qp_solver4.data()->setGradient(gradient2);

  /* equality constraint: zero total wnrech */
  constraints = Eigen::MatrixXd::Zero(6 + link_joint_num, fr_ndof + fe_ndof);
  constraints.topRows(6) = A2;
  constraints.bottomRows(link_joint_num) = A1;
  Eigen::SparseMatrix<double> constraint_sparse4 = constraints.sparseView();
  qp_solver4.data()->setLinearConstraintsMatrix(constraint_sparse4);
  qp_solver4.data()->setLowerBound(lower_bound);
  qp_solver4.data()->setUpperBound(upper_bound);

  if(!qp_solver4.initSolver()) {
    ROS_ERROR("can not initialize qp solver");
  }

  s_t = ros::Time::now().toSec();
  res = qp_solver4.solve();
  ROS_INFO_STREAM_ONCE("QP4 solve: " << ros::Time::now().toSec() - s_t);
  if(!res) {
    ROS_ERROR("can not solve QP4");
  }
  else {

    Eigen::VectorXd f_all_neq = qp_solver4.getSolution();
    Eigen::VectorXd fr = f_all_neq.head(fr_ndof);
    Eigen::VectorXd fe = f_all_neq.tail(fe_ndof);
    Eigen::VectorXd tor = A1 * f_all_neq + b1;

    ROS_INFO_STREAM_ONCE("[QP4] Thrust force for stand: " << fr.transpose());
    ROS_INFO_STREAM_ONCE("[QP4] Contact force for stand: " << fe.transpose());
    ROS_INFO_STREAM_ONCE("[QP4] Joint Torque: " << tor.transpose());
    ROS_INFO_STREAM_ONCE("[QP4] Wrench: " << (A2 * f_all_neq + b2).transpose());

    ROS_INFO_STREAM_THROTTLE(1.0, "[QP4] Thrust force for stand: " << fr.transpose());
    ROS_INFO_STREAM_THROTTLE(1.0, "[QP4] Contact force for stand: " << fe.transpose());
    ROS_INFO_STREAM_THROTTLE(1.0, "[QP4] Joint Torque: " << (A1 * f_all_neq + b1).transpose());
    //ROS_INFO_STREAM_THROTTLE(1.0, "[QP4] Wrench: " << (A2 * f_all_neq + b2).transpose());

    Eigen::VectorXd lambda = Eigen::VectorXd::Zero(rotor_num);
    for(int i = 0; i < rotor_num; i++) {
      lambda(i) = fr.segment(3 * i, 3).norm();
    }

    ROS_INFO_STREAM_ONCE("[QP4] Thrust force lambda: " << lambda.transpose());
    ROS_INFO_STREAM_THROTTLE(1.0, "[QP4] Thrust force lambda: " << lambda.transpose());

    setStaticVectoringF(fr);
    setStaticJointT(tor);
  }

}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Tiger::FullVectoringRobotModel, aerial_robot_model::RobotModel);

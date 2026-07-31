// -*- mode: c++ -*-
#pragma once

#include <pinocchio/fwd.hpp>

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/multibody/fwd.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>

namespace aerial_robot_dynamics
{
class PinocchioRobotModel;
}

namespace aerial_robot_control
{

class FwddynMpcControlProblem
{
public:
  using Scalar = float;

  enum SolverType
  {
    FDDP = 0,
    BOX_FDDP,
    INTRO,
    HPIPM_SQP
  };

  struct Parameters
  {
    int num_nodes = 50;
    int max_iter = 1;
    int max_init_iter = 100;
    double dt = 0.02;
    int num_threads = 1;
    SolverType solver_type = SolverType::FDDP;

    Eigen::VectorXd q_ref;
    Eigen::Vector3d com_track_weight = Eigen::Vector3d::Constant(1e4);
    Eigen::VectorXd centroidal_momentum_weight = Eigen::VectorXd::Constant(6, 1e1);
    Eigen::VectorXd x_state_weight;
    double control_weight = 1e-1;
    double thrust_reg_weight = 1e-1;
    double thrust_barrier_weight = 1e1;
    double delta_thrust_max = Scalar(1e5);
    std::vector<std::string> locked_joint_names = {};

    void print()
    {
      std::cout << "MPC Parameters:" << std::endl;
      std::cout << "  num_nodes: " << num_nodes << std::endl;
      std::cout << "  max_iter: " << max_iter << std::endl;
      std::cout << "  max_init_iter: " << max_init_iter << std::endl;
      std::cout << "  dt: " << dt << std::endl;
      std::cout << "  num_threads: " << num_threads << std::endl;
      std::cout << "  solver_type: "
                << (solver_type == SolverType::FDDP ? "FDDP" :
                                                      (solver_type == SolverType::BOX_FDDP ?
                                                           "BOX_FDDP" :
                                                           (solver_type == SolverType::INTRO ? "INTRO" : "HPIPM_SQP")))
                << std::endl;
      std::cout << "  q_ref: " << q_ref.transpose() << std::endl;
      std::cout << "  com_track_weight: " << com_track_weight.transpose() << std::endl;
      std::cout << "  centroidal_momentum_weight: " << centroidal_momentum_weight.transpose() << std::endl;
      std::cout << "  control_weight: " << control_weight << std::endl;
      std::cout << "  thrust_barrier_weight: " << thrust_barrier_weight << std::endl;
      std::cout << "  thrust_reg_weight: " << thrust_reg_weight << std::endl;
      if (x_state_weight.size() > 0)
        std::cout << "  x_state_weight: " << x_state_weight.transpose() << std::endl;
      std::cout << "  delta_thrust_max: " << delta_thrust_max << std::endl;
      if (!locked_joint_names.empty())
      {
        std::cout << "  locked_joint_names: ";
        for (const auto& name : locked_joint_names)
          std::cout << name << " ";
        std::cout << std::endl;
      }
    }
  };

  FwddynMpcControlProblem() = default;

  void initialize(const std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel>& pinocchio_robot_model,
                  const Parameters& parameters);
  void reset();

  void buildMPCProblem(const Eigen::VectorXd& x0, const Eigen::Vector3d& com_target, const Eigen::VectorXd& x_ref);
  void setInitialState(const Eigen::VectorXd& x0);
  void setReferences(const std::vector<Eigen::Vector3d>& com_traj, const std::vector<Eigen::VectorXd>& x_ref_traj);
  void slideHorizon();
  bool solveMPC(int max_iter, bool verbose = false, bool is_feasible = false);

  bool isInitialized() const
  {
    return initialized_;
  }
  bool hasSolution() const
  {
    return !xs_.empty() && !us_.empty();
  }

  const Parameters& parameters() const
  {
    return parameters_;
  }
  // reduced pinocchio model actually used by the OCP (== full model when no joints are locked)
  const std::shared_ptr<pinocchio::Model>& reducedModel() const
  {
    return pin_model_;
  }
  Eigen::VectorXd expandState(const Eigen::VectorXd& x_reduced) const;
  const std::vector<Eigen::VectorXd>& xs() const
  {
    return xs_;
  }
  const std::vector<Eigen::VectorXd>& us() const
  {
    return us_;
  }

private:
  std::shared_ptr<crocoddyl::IntegratedActionModelEulerWithThrustsTpl<Scalar>>
  createMPCNode(const Eigen::Matrix<Scalar, 3, 1>& com_target, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x_ref);

  // reduce full-model vectors to the locked-joint (reduced) model layout
  Eigen::VectorXd reduceStateRef(const Eigen::VectorXd& x_ref_full) const;      // [q; v]         -> reduced
  Eigen::VectorXd reduceState(const Eigen::VectorXd& x_full) const;             // [q; v; thrust] -> reduced
  Eigen::VectorXd reduceStateWeight(const Eigen::VectorXd& weight_full) const;  // [dq; dv]       -> reduced

  bool initialized_ = false;

  // full-model dimensions and the q/v indices removed by joint locking (sorted ascending)
  int nq_full_ = 0;
  int nv_full_ = 0;
  std::vector<int> locked_q_idx_;
  std::vector<int> locked_v_idx_;

  Parameters parameters_;

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pin_model_;
  std::shared_ptr<pinocchio::ModelTpl<Scalar>> pin_model_f_;

  std::shared_ptr<crocoddyl::StateMultibodyTpl<Scalar>> state_mb_;
  std::shared_ptr<crocoddyl::StateMultibodyWithThrustsTpl<Scalar>> state_with_thrusts_;
  std::shared_ptr<crocoddyl::ActuationModelFloatingBaseThrusterRatesTpl<Scalar>> actuation_;

  std::shared_ptr<crocoddyl::ShootingProblemTpl<Scalar>> shooting_problem_;
  std::shared_ptr<crocoddyl::SolverAbstractTpl<Scalar>> solver_;

  std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> xs_f_;
  std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> us_f_;
  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> us_;

  std::vector<std::shared_ptr<crocoddyl::ResidualModelCoMPositionTpl<Scalar>>> com_residuals_;
  std::vector<std::shared_ptr<crocoddyl::ResidualModelStateTpl<Scalar>>> state_residuals_;
};

}  // namespace aerial_robot_control

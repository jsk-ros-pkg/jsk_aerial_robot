// -*- mode: c++ -*-
#include <aerial_robot_control/control/fwddyn_mpc_control_problem.hpp>

#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solvers/box-fddp.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/solvers/intro.hpp>
#ifdef CROCODDYL_WITH_HPIPM
#include <crocoddyl/core/solvers/hpipm-sqp.hpp>
#endif
#include <crocoddyl/core/utils/timer.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn-with-thrusts.hpp>
#include <crocoddyl/multibody/actuations/floating-base-thrust-rates.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/residuals/centroidal-momentum.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/states/multibody-with-thrusts.hpp>

#include <aerial_robot_dynamics/robot_model.h>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <algorithm>
#include <cmath>

namespace aerial_robot_control
{

namespace
{
std::vector<Eigen::VectorXd>
toDoubleTrajectory(const std::vector<Eigen::Matrix<FwddynMpcControlProblem::Scalar, Eigen::Dynamic, 1>>& trajectory)
{
  std::vector<Eigen::VectorXd> result;
  result.reserve(trajectory.size());
  for (const auto& value : trajectory)
    result.push_back(value.template cast<double>());
  return result;
}

// remove the entries at the given (sorted, ascending) indices from v
Eigen::VectorXd eraseRows(const Eigen::VectorXd& v, const std::vector<int>& remove)
{
  if (remove.empty())
    return v;
  Eigen::VectorXd out(v.size() - static_cast<Eigen::Index>(remove.size()));
  std::size_t r = 0;
  Eigen::Index o = 0;
  for (Eigen::Index i = 0; i < v.size(); ++i)
  {
    if (r < remove.size() && i == remove[r])
    {
      ++r;
      continue;
    }
    out(o++) = v(i);
  }
  return out;
}

Eigen::VectorXd insertRows(const Eigen::VectorXd& reduced, const std::vector<int>& insert_at,
                           const Eigen::VectorXd& source_full)
{
  if (insert_at.empty())
    return reduced;
  Eigen::VectorXd out(source_full.size());
  std::size_t k = 0;
  Eigen::Index r = 0;
  for (Eigen::Index i = 0; i < out.size(); ++i)
  {
    if (k < insert_at.size() && i == insert_at[k])
    {
      out(i) = source_full(i);
      ++k;
    }
    else
    {
      out(i) = reduced(r++);
    }
  }
  return out;
}

}  // namespace

void FwddynMpcControlProblem::initialize(
    const std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel>& pinocchio_robot_model,
    const Parameters& parameters)
{
  pinocchio_robot_model_ = pinocchio_robot_model;
  pin_model_ = pinocchio_robot_model_->getModel();

  // full-model dimensions (used to reduce x_ref / x_state_weight / x0 to the locked-joint layout)
  nq_full_ = pin_model_->nq;
  nv_full_ = pin_model_->nv;
  locked_q_idx_.clear();
  locked_v_idx_.clear();

  // convert pinocchio model to reduced model if there are locked joints
  if (!parameters.locked_joint_names.empty())
  {
    pin_model_ = std::make_shared<pinocchio::Model>(*pin_model_);

    // resolve duplication of fixed root_joint and pinocchio's free-flyer joint
    for (auto& f : pin_model_->frames)
      if (f.name == "root_joint" && f.type == pinocchio::FIXED_JOINT)
        f.name = "root_joint_fixed";

    std::vector<pinocchio::JointIndex> locked_joint_ids;
    std::cout << "locked joint id: ";
    for (const auto& joint_name : parameters.locked_joint_names)
    {
      if (!pin_model_->existJointName(joint_name))
        throw std::runtime_error("Locked joint name " + joint_name + " not found");

      pinocchio::JointIndex joint_id = pin_model_->getJointId(joint_name);
      locked_joint_ids.push_back(joint_id);
      std::cout << joint_id << " ";

      // record the q/v indices occupied by this joint in the full model
      const auto& jm = pin_model_->joints[joint_id];
      for (int k = 0; k < jm.nq(); ++k)
        locked_q_idx_.push_back(jm.idx_q() + k);
      for (int k = 0; k < jm.nv(); ++k)
        locked_v_idx_.push_back(jm.idx_v() + k);
    }
    std::cout << std::endl;
    std::sort(locked_q_idx_.begin(), locked_q_idx_.end());
    std::sort(locked_v_idx_.begin(), locked_v_idx_.end());
    pin_model_ = std::make_shared<pinocchio::Model>(
        pinocchio::buildReducedModel(*pin_model_, locked_joint_ids, parameters.q_ref));
  }

  pin_model_f_ = std::make_shared<pinocchio::ModelTpl<Scalar>>(pin_model_->cast<Scalar>());
  parameters_ = parameters;

  // reduce the state regularisation weight ([dq; dv], length 2*nv) to the locked-joint layout
  if (!locked_v_idx_.empty())
    parameters_.x_state_weight = reduceStateWeight(parameters_.x_state_weight);

  const int rotor_num = pinocchio_robot_model_->getRotorNum();

  state_mb_ = std::make_shared<crocoddyl::StateMultibodyTpl<Scalar>>(pin_model_f_);

  const auto& rotor_names = pinocchio_robot_model_->getRotorNames();
  pinocchio::Data pin_data(*pin_model_);
  const Eigen::VectorXd q_neutral = pinocchio::neutral(*pin_model_);
  pinocchio::framesForwardKinematics(*pin_model_, pin_data, q_neutral);

  std::vector<crocoddyl::DistributedThrusterTpl<Scalar>> thrusters;
  const Scalar m_f_rate = static_cast<Scalar>(std::abs(pinocchio_robot_model_->getMFRate()));
  for (int i = 0; i < rotor_num; ++i)
  {
    const std::string& rotor_name = rotor_names.at(i);
    const pinocchio::FrameIndex rotor_fid = pin_model_->getFrameId(rotor_name);
    const pinocchio::JointIndex parent_joint = pin_model_->frames[rotor_fid].parentJoint;
    const pinocchio::SE3 joint_M_rotor = pin_data.oMi[parent_joint].inverse() * pin_data.oMf[rotor_fid];

    const int dir = pinocchio_robot_model_->getRotorDirection(i);
    thrusters.emplace_back(static_cast<int>(rotor_fid), joint_M_rotor.cast<Scalar>(), m_f_rate,
                           (dir == 1) ? crocoddyl::DT_CCW : crocoddyl::DT_CW,
                           static_cast<Scalar>(pinocchio_robot_model_->getThrustLowerLimits()(i)),
                           static_cast<Scalar>(pinocchio_robot_model_->getThrustUpperLimits()(i)),
                           static_cast<Scalar>(parameters_.delta_thrust_max));
  }

  state_with_thrusts_ = std::make_shared<crocoddyl::StateMultibodyWithThrustsTpl<Scalar>>(state_mb_, rotor_num);
  actuation_ =
      std::make_shared<crocoddyl::ActuationModelFloatingBaseThrusterRatesTpl<Scalar>>(state_with_thrusts_, thrusters);

  initialized_ = true;
  reset();
}

void FwddynMpcControlProblem::reset()
{
  shooting_problem_.reset();
  solver_.reset();
  xs_f_.clear();
  us_f_.clear();
  xs_.clear();
  us_.clear();
  com_residuals_.clear();
  state_residuals_.clear();
}

Eigen::VectorXd FwddynMpcControlProblem::reduceStateRef(const Eigen::VectorXd& x_ref_full) const
{
  // x_ref layout: [q (nq_full), v (nv_full)]
  if (locked_q_idx_.empty() && locked_v_idx_.empty())
    return x_ref_full;
  const Eigen::VectorXd q = eraseRows(x_ref_full.head(nq_full_), locked_q_idx_);
  const Eigen::VectorXd v = eraseRows(x_ref_full.segment(nq_full_, nv_full_), locked_v_idx_);
  Eigen::VectorXd out(q.size() + v.size());
  out << q, v;
  return out;
}

Eigen::VectorXd FwddynMpcControlProblem::reduceState(const Eigen::VectorXd& x_full) const
{
  // x0 layout: [q (nq_full), v (nv_full), thrust (rotor_num)] -- the thrust tail is kept as is
  if (locked_q_idx_.empty() && locked_v_idx_.empty())
    return x_full;
  const Eigen::VectorXd q = eraseRows(x_full.head(nq_full_), locked_q_idx_);
  const Eigen::VectorXd v = eraseRows(x_full.segment(nq_full_, nv_full_), locked_v_idx_);
  const Eigen::VectorXd thrust = x_full.tail(x_full.size() - nq_full_ - nv_full_);
  Eigen::VectorXd out(q.size() + v.size() + thrust.size());
  out << q, v, thrust;
  return out;
}

Eigen::VectorXd FwddynMpcControlProblem::expandState(const Eigen::VectorXd& x_reduced) const
{
  // inverse of reduceState: reduced [q; v; thrust] -> full [q; v; thrust]
  if (locked_q_idx_.empty() && locked_v_idx_.empty())
    return x_reduced;
  const int nq_r = nq_full_ - static_cast<int>(locked_q_idx_.size());
  const int nv_r = nv_full_ - static_cast<int>(locked_v_idx_.size());
  const Eigen::VectorXd q_full = insertRows(x_reduced.head(nq_r), locked_q_idx_, parameters_.q_ref);
  const Eigen::VectorXd v_full =
      insertRows(x_reduced.segment(nq_r, nv_r), locked_v_idx_, Eigen::VectorXd::Zero(nv_full_));
  const Eigen::VectorXd thrust = x_reduced.tail(x_reduced.size() - nq_r - nv_r);
  Eigen::VectorXd out(nq_full_ + nv_full_ + thrust.size());
  out << q_full, v_full, thrust;
  return out;
}

Eigen::VectorXd FwddynMpcControlProblem::reduceStateWeight(const Eigen::VectorXd& weight_full) const
{
  // x_state_weight layout: [dq (nv_full), dv (nv_full)] -- both blocks use the v indices
  if (locked_v_idx_.empty())
    return weight_full;
  const Eigen::VectorXd dq = eraseRows(weight_full.head(nv_full_), locked_v_idx_);
  const Eigen::VectorXd dv = eraseRows(weight_full.segment(nv_full_, nv_full_), locked_v_idx_);
  Eigen::VectorXd out(dq.size() + dv.size());
  out << dq, dv;
  return out;
}

std::shared_ptr<crocoddyl::IntegratedActionModelEulerWithThrustsTpl<FwddynMpcControlProblem::Scalar>>
FwddynMpcControlProblem::createMPCNode(const Eigen::Matrix<Scalar, 3, 1>& com_target,
                                       const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x_ref)
{
  const std::size_t nu = actuation_->get_nu();

  // contact
  auto contacts = std::make_shared<crocoddyl::ContactModelMultipleTpl<Scalar>>(state_mb_, nu);
  auto costs = std::make_shared<crocoddyl::CostModelSumTpl<Scalar>>(state_mb_, nu);

  // CoM tracking
  auto com_res = std::make_shared<crocoddyl::ResidualModelCoMPositionTpl<Scalar>>(state_mb_, com_target, nu);
  com_residuals_.push_back(com_res);
  auto com_act =
      std::make_shared<crocoddyl::ActivationModelWeightedQuadTpl<Scalar>>(parameters_.com_track_weight.cast<Scalar>());
  costs->addCost("comTrack", std::make_shared<crocoddyl::CostModelResidualTpl<Scalar>>(state_mb_, com_act, com_res),
                 Scalar(1.0));

  // State regularisation
  auto state_res = std::make_shared<crocoddyl::ResidualModelStateTpl<Scalar>>(state_mb_, x_ref, nu);
  state_residuals_.push_back(state_res);
  auto state_act =
      std::make_shared<crocoddyl::ActivationModelWeightedQuadTpl<Scalar>>(parameters_.x_state_weight.cast<Scalar>());
  costs->addCost("stateReg", std::make_shared<crocoddyl::CostModelResidualTpl<Scalar>>(state_mb_, state_act, state_res),
                 Scalar(1.0));

  // Centroidal momentum regularisation
  auto cm_res = std::make_shared<crocoddyl::ResidualModelCentroidalMomentumTpl<Scalar>>(
      state_mb_, Eigen::Matrix<Scalar, 6, 1>::Zero(), nu);
  auto cm_act = std::make_shared<crocoddyl::ActivationModelWeightedQuadTpl<Scalar>>(
      parameters_.centroidal_momentum_weight.cast<Scalar>());
  costs->addCost("centroidalMomentumReg",
                 std::make_shared<crocoddyl::CostModelResidualTpl<Scalar>>(state_mb_, cm_act, cm_res), Scalar(1.0));

  // Control regularisation
  auto ctrl_res = std::make_shared<crocoddyl::ResidualModelControlTpl<Scalar>>(state_mb_, nu);
  costs->addCost("ctrlReg", std::make_shared<crocoddyl::CostModelResidualTpl<Scalar>>(state_mb_, ctrl_res),
                 static_cast<Scalar>(parameters_.control_weight));

  auto dam = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamicsWithThrustsTpl<Scalar>>(
      state_with_thrusts_, actuation_, contacts, costs, Scalar(1e-6), false);

  // Thurst regularization
  dam->set_thrust_reg_weight(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Constant(
      pinocchio_robot_model_->getRotorNum(), static_cast<Scalar>(parameters_.thrust_reg_weight)));

  // Thrust saturation barrier
  dam->set_thrust_barrier(
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Constant(pinocchio_robot_model_->getRotorNum(),
                                                         static_cast<Scalar>(parameters_.thrust_barrier_weight)),
      pinocchio_robot_model_->getThrustLowerLimits().cast<Scalar>(),
      pinocchio_robot_model_->getThrustUpperLimits().cast<Scalar>());

  return std::make_shared<crocoddyl::IntegratedActionModelEulerWithThrustsTpl<Scalar>>(
      dam, static_cast<Scalar>(parameters_.dt));
}

void FwddynMpcControlProblem::buildMPCProblem(const Eigen::VectorXd& x0, const Eigen::Vector3d& com_target,
                                              const Eigen::VectorXd& x_ref)
{
  com_residuals_.clear();
  state_residuals_.clear();

  const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x0_f = reduceState(x0).cast<Scalar>();
  const Eigen::Matrix<Scalar, 3, 1> com_target_f = com_target.cast<Scalar>();
  const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x_ref_f = reduceStateRef(x_ref).cast<Scalar>();

  // create running and terminal models
  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstractTpl<Scalar>>> running_models;
  running_models.reserve(parameters_.num_nodes);
  for (int i = 0; i < parameters_.num_nodes; ++i)
    running_models.push_back(createMPCNode(com_target_f, x_ref_f));

  auto terminal = createMPCNode(com_target_f, x_ref_f);

  // create OCP and solver
  shooting_problem_ = std::make_shared<crocoddyl::ShootingProblemTpl<Scalar>>(x0_f, running_models, terminal);
  shooting_problem_->set_nthreads(parameters_.num_threads);

  if (parameters_.solver_type == SolverType::FDDP)
    solver_ = std::make_shared<crocoddyl::SolverFDDPTpl<Scalar>>(shooting_problem_);
  else if (parameters_.solver_type == SolverType::BOX_FDDP)
    solver_ = std::make_shared<crocoddyl::SolverBoxFDDPTpl<Scalar>>(shooting_problem_);
  else if (parameters_.solver_type == SolverType::INTRO)
    solver_ = std::make_shared<crocoddyl::SolverIntroTpl<Scalar>>(shooting_problem_);
  else if (parameters_.solver_type == SolverType::HPIPM_SQP)
#ifdef CROCODDYL_WITH_HPIPM
    solver_ = std::make_shared<crocoddyl::SolverHpipmSQPTpl<Scalar>>(shooting_problem_);
#else
    throw std::runtime_error(
        "HPIPM_SQP solver is selected but crocoddyl was built without HPIPM support "
        "(CROCODDYL_WITH_HPIPM is not defined). Rebuild crocoddyl with BUILD_WITH_HPIPM=ON and hpipm-cpp available.");
#endif
  else
    throw std::runtime_error("Invalid solver type");

  xs_f_.assign(parameters_.num_nodes + 1, x0_f);
  us_f_.resize(parameters_.num_nodes);
  for (int i = 0; i < parameters_.num_nodes; ++i)
  {
    us_f_[i] = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(actuation_->get_nu());
    running_models[i]->quasiStatic(running_models[i]->createData(), us_f_[i], x0_f);
  }

  xs_ = toDoubleTrajectory(xs_f_);
  us_ = toDoubleTrajectory(us_f_);
}

void FwddynMpcControlProblem::setInitialState(const Eigen::VectorXd& x0)
{
  if (!shooting_problem_ || xs_.empty())
    return;

  const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x0_f = reduceState(x0).cast<Scalar>();
  shooting_problem_->set_x0(x0_f);
  xs_f_[0] = x0_f;
  xs_[0] = x0_f.cast<double>();
}

void FwddynMpcControlProblem::setReferences(const std::vector<Eigen::Vector3d>& com_traj,
                                            const std::vector<Eigen::VectorXd>& x_ref_traj)
{
  if (com_traj.size() != com_residuals_.size() || x_ref_traj.size() != state_residuals_.size())
  {
    std::cerr << "[FwddynMpcControlProblem] setReferences: size mismatch (expected " << com_residuals_.size()
              << " nodes, got com_traj=" << com_traj.size() << ", x_ref_traj=" << x_ref_traj.size() << ")" << std::endl;
    return;
  }

  for (std::size_t i = 0; i < com_residuals_.size(); ++i)
    com_residuals_[i]->set_reference(com_traj[i].cast<Scalar>());
  for (std::size_t i = 0; i < state_residuals_.size(); ++i)
    state_residuals_[i]->set_reference(reduceStateRef(x_ref_traj[i]).cast<Scalar>());
}

void FwddynMpcControlProblem::slideHorizon()
{
  // retreive the first running node and data (which will become the new terminal node)
  std::shared_ptr<crocoddyl::ActionModelAbstractTpl<Scalar>> new_terminal =
      shooting_problem_->get_runningModels().front();
  std::shared_ptr<crocoddyl::ActionDataAbstractTpl<Scalar>> new_terminal_data =
      shooting_problem_->get_runningDatas().front();

  // append a terminal node to the end of running nodes, and remove the first node
  shooting_problem_->circularAppend(shooting_problem_->get_terminalModel(), shooting_problem_->get_terminalData());

  // update the terminal node with the first running node
  shooting_problem_->updateNode(shooting_problem_->get_T(), new_terminal, new_terminal_data);

  // shift the references for the new terminal node
  std::shared_ptr<crocoddyl::ResidualModelCoMPositionTpl<Scalar>> new_com_res = com_residuals_.front();
  std::shared_ptr<crocoddyl::ResidualModelStateTpl<Scalar>> new_state_res = state_residuals_.front();

  com_residuals_.erase(com_residuals_.begin());
  state_residuals_.erase(state_residuals_.begin());

  com_residuals_.push_back(new_com_res);
  state_residuals_.push_back(new_state_res);

  // update the initial guess by shifting the previous solution
  xs_f_.erase(xs_f_.begin());
  xs_f_.push_back(xs_f_.back());
  us_f_.erase(us_f_.begin());
  us_f_.push_back(us_f_.back());

  // rollout terminal node to close the dynamics gap introduced by the shift
  const auto& last_model = shooting_problem_->get_runningModels().back();
  const auto& last_data = shooting_problem_->get_runningDatas().back();
  last_model->calc(last_data, xs_f_[xs_f_.size() - 2], us_f_.back());
  xs_f_.back() = last_data->xnext;

  xs_ = toDoubleTrajectory(xs_f_);
  us_ = toDoubleTrajectory(us_f_);
}

bool FwddynMpcControlProblem::solveMPC(int max_iter, bool verbose, bool is_feasible)
{
  if (!solver_)
    return false;

  crocoddyl::Timer timer;
  const bool solved = solver_->solve(xs_f_, us_f_, max_iter, is_feasible);
  double solve_time = timer.get_duration();

  if (verbose)
  {
    std::cout << "MPC solved: " << solved << std::endl;
    std::cout << "total calculation time:" << solve_time << std::endl;
    std::cout << "Number of iterations: " << solver_->get_iter() << std::endl;
    std::cout << "time per iterate:" << solve_time / solver_->get_iter() << std::endl;
    std::cout << "Total cost: " << solver_->get_cost() << std::endl;
    std::cout << "Gradient norm: " << solver_->get_stop() << std::endl;
    std::cout << std::endl;
  }

  xs_f_ = solver_->get_xs();
  us_f_ = solver_->get_us();
  xs_ = toDoubleTrajectory(xs_f_);
  us_ = toDoubleTrajectory(us_f_);
  return solved;
}

}  // namespace aerial_robot_control

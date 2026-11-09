#include <aerial_robot_dynamics/robot_model.h>

#include <boost/noncopyable.hpp>
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <stdexcept>

namespace bp = boost::python;

namespace
{
using aerial_robot_dynamics::PinocchioRobotModel;

pinocchio::Model& getModel(PinocchioRobotModel& self)
{
  return *self.getModel();
}

pinocchio::Data& getData(PinocchioRobotModel& self)
{
  return *self.getData();
}

Eigen::VectorXd forwardDynamics(PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q,
                                const Eigen::Ref<const Eigen::VectorXd>& v,
                                const Eigen::Ref<const Eigen::VectorXd>& tau,
                                const Eigen::Ref<const Eigen::VectorXd>& thrust)
{
  Eigen::VectorXd thrust_copy = thrust;
  return self.forwardDynamics(q, v, tau, thrust_copy);
}

Eigen::MatrixXd forwardDynamicsDerivatives(PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q,
                                           const Eigen::Ref<const Eigen::VectorXd>& v,
                                           const Eigen::Ref<const Eigen::VectorXd>& tau,
                                           const Eigen::Ref<const Eigen::VectorXd>& thrust)
{
  Eigen::VectorXd thrust_copy = thrust;
  return self.forwardDynamicsDerivatives(q, v, tau, thrust_copy);
}

void validateOutputSize(const Eigen::Ref<Eigen::VectorXd>& out, Eigen::Index expected_size, const char* name)
{
  if (out.size() != expected_size)
    throw std::invalid_argument(std::string(name) + " must have size " + std::to_string(expected_size) + ".");
}

bool inverseDynamicsOsqp(PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q,
                         const Eigen::Ref<const Eigen::VectorXd>& v, const Eigen::Ref<const Eigen::VectorXd>& a,
                         Eigen::Ref<Eigen::VectorXd> tau, const Eigen::VectorXd& base_residual_lower,
                         const Eigen::VectorXd& base_residual_upper)
{
  validateOutputSize(tau, self.getModel()->nv + self.getRotorNum(), "tau");

  Eigen::VectorXd tau_result;
  const bool solved = self.inverseDynamicsOsqp(q, v, a, tau_result, base_residual_lower, base_residual_upper);
  tau = tau_result;
  return solved;
}

bool inverseDynamicsProxqp(PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q,
                           const Eigen::Ref<const Eigen::VectorXd>& v, const Eigen::Ref<const Eigen::VectorXd>& a,
                           Eigen::Ref<Eigen::VectorXd> tau, const Eigen::VectorXd& base_residual_lower,
                           const Eigen::VectorXd& base_residual_upper)
{
  validateOutputSize(tau, self.getModel()->nv + self.getRotorNum(), "tau");

  Eigen::VectorXd tau_result;
  const bool solved = self.inverseDynamicsProxqp(q, v, a, tau_result, base_residual_lower, base_residual_upper);
  tau = tau_result;
  return solved;
}

bool inverseDynamics(PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q,
                     const Eigen::Ref<const Eigen::VectorXd>& v, const Eigen::Ref<const Eigen::VectorXd>& a,
                     Eigen::Ref<Eigen::VectorXd> tau)
{
  validateOutputSize(tau, self.getModel()->nv + self.getRotorNum(), "tau");

  Eigen::VectorXd tau_result;
  const bool solved = self.inverseDynamics(q, v, a, tau_result);
  tau = tau_result;
  return solved;
}

int getRotorDirection(PinocchioRobotModel& self, int index)
{
  return self.getRotorDirection(index);
}

std::string getRobotDescription(PinocchioRobotModel& self)
{
  return self.getRobotDescription();
}

std::string getPinocchioRobotDescription(PinocchioRobotModel& self)
{
  return self.getPinocchioRobotDescription();
}

bool getIsFloatingBase(PinocchioRobotModel& self)
{
  return self.getIsFloatingBase();
}

int getRotorNum(PinocchioRobotModel& self)
{
  return self.getRotorNum();
}

double getMFRate(PinocchioRobotModel& self)
{
  return self.getMFRate();
}

double getLatestIdSolveTime(PinocchioRobotModel& self)
{
  return self.getLatestIdSolveTime();
}

double getThrustHessianWeight(PinocchioRobotModel& self)
{
  return self.getThrustHessianWeight();
}

int getNq(PinocchioRobotModel& self)
{
  return self.getModel()->nq;
}

int getNv(PinocchioRobotModel& self)
{
  return self.getModel()->nv;
}

int getNjoints(PinocchioRobotModel& self)
{
  return self.getModel()->njoints;
}

int getNframes(PinocchioRobotModel& self)
{
  return self.getModel()->nframes;
}
}  // namespace

BOOST_PYTHON_MODULE(_robot_model)
{
  eigenpy::enableEigenPy();

  // Importing pinocchio registers Boost.Python/eigenpy converters for
  // pinocchio::Model, Data, SE3, Force, and their std::vector containers.
  bp::import("pinocchio");

  bp::class_<PinocchioRobotModel::Config>("Config")
      .def_readwrite("thrust_hessian_weight", &PinocchioRobotModel::Config::thrust_hessian_weight)
      .def_readwrite("verbose", &PinocchioRobotModel::Config::verbose)
      .def_readwrite("warm_start_inverse_dynamics", &PinocchioRobotModel::Config::warm_start_inverse_dynamics);

  bp::class_<PinocchioRobotModel, boost::noncopyable>("PinocchioRobotModel", bp::no_init)
      .def(bp::init<std::string, std::string, bool>(
          (bp::arg("robot_description"), bp::arg("pinocchio_robot_description"), bp::arg("is_floating_base"))))
      .def(bp::init<std::string, std::string, bool, const PinocchioRobotModel::Config&>(
          (bp::arg("robot_description"), bp::arg("pinocchio_robot_description"), bp::arg("is_floating_base"),
           bp::arg("config"))))

      .def("getModel", &getModel, bp::return_internal_reference<>())
      .def("get_model", &getModel, bp::return_internal_reference<>())
      .def("getData", &getData, bp::return_internal_reference<>())
      .def("get_data", &getData, bp::return_internal_reference<>())

      .def("forwardDynamics", &forwardDynamics,
           (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("tau"), bp::arg("thrust")))
      .def("forward_dynamics", &forwardDynamics,
           (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("tau"), bp::arg("thrust")))
      .def("forwardDynamicsDerivatives", &forwardDynamicsDerivatives,
           (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("tau"), bp::arg("thrust")))
      .def("forward_dynamics_derivatives", &forwardDynamicsDerivatives,
           (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("tau"), bp::arg("thrust")))
      .def("inverseDynamics", &inverseDynamics,
           (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("a"), bp::arg("tau")))
      .def("inverse_dynamics", &inverseDynamics,
           (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("a"), bp::arg("tau")))
      .def("inverseDynamicsOsqp", &inverseDynamicsOsqp,
           (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("a"), bp::arg("tau"),
            bp::arg("base_residual_lower") = Eigen::VectorXd(Eigen::VectorXd::Zero(6)),
            bp::arg("base_residual_upper") = Eigen::VectorXd(Eigen::VectorXd::Zero(6))))
      .def("inverse_dynamics_osqp", &inverseDynamicsOsqp,
           (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("a"), bp::arg("tau"),
            bp::arg("base_residual_lower") = Eigen::VectorXd(Eigen::VectorXd::Zero(6)),
            bp::arg("base_residual_upper") = Eigen::VectorXd(Eigen::VectorXd::Zero(6))))
      .def("inverseDynamicsProxqp", &inverseDynamicsProxqp,
           (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("a"), bp::arg("tau"),
            bp::arg("base_residual_lower") = Eigen::VectorXd(Eigen::VectorXd::Zero(6)),
            bp::arg("base_residual_upper") = Eigen::VectorXd(Eigen::VectorXd::Zero(6))))
      .def("inverse_dynamics_proxqp", &inverseDynamicsProxqp,
           (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("a"), bp::arg("tau"),
            bp::arg("base_residual_lower") = Eigen::VectorXd(Eigen::VectorXd::Zero(6)),
            bp::arg("base_residual_upper") = Eigen::VectorXd(Eigen::VectorXd::Zero(6))))

      .def("computeFExtByThrust", &PinocchioRobotModel::computeFExtByThrust, (bp::arg("self"), bp::arg("thrust")))
      .def("compute_f_ext_by_thrust", &PinocchioRobotModel::computeFExtByThrust, (bp::arg("self"), bp::arg("thrust")))
      .def("computeTauExtByThrustDerivativeQDerivatives",
           &PinocchioRobotModel::computeTauExtByThrustDerivativeQDerivatives, (bp::arg("self"), bp::arg("q")))
      .def("compute_tau_ext_by_thrust_derivative_q_derivatives",
           &PinocchioRobotModel::computeTauExtByThrustDerivativeQDerivatives, (bp::arg("self"), bp::arg("q")))
      .def("computeTauExtByThrustDerivativeQDerivativesNum",
           &PinocchioRobotModel::computeTauExtByThrustDerivativeQDerivativesNum, (bp::arg("self"), bp::arg("q")))
      .def("compute_tau_ext_by_thrust_derivative_q_derivatives_num",
           &PinocchioRobotModel::computeTauExtByThrustDerivativeQDerivativesNum, (bp::arg("self"), bp::arg("q")))
      .def("computeTauExtByThrust", &PinocchioRobotModel::computeTauExtByThrust,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("compute_tau_ext_by_thrust", &PinocchioRobotModel::computeTauExtByThrust,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("computeTauExtByThrustDerivative", &PinocchioRobotModel::computeTauExtByThrustDerivative,
           (bp::arg("self"), bp::arg("q")))
      .def("compute_tau_ext_by_thrust_derivative", &PinocchioRobotModel::computeTauExtByThrustDerivative,
           (bp::arg("self"), bp::arg("q")))
      .def("computeTauExtByThrustQDerivative", &PinocchioRobotModel::computeTauExtByThrustQDerivative,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("compute_tau_ext_by_thrust_q_derivative", &PinocchioRobotModel::computeTauExtByThrustQDerivative,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("computeTauExtByThrustQDerivativeRnea", &PinocchioRobotModel::computeTauExtByThrustQDerivativeRnea,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("compute_tau_ext_by_thrust_q_derivative_rnea", &PinocchioRobotModel::computeTauExtByThrustQDerivativeRnea,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("computeTauExtByThrustQDerivativeStaticTorque",
           &PinocchioRobotModel::computeTauExtByThrustQDerivativeStaticTorque,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("compute_tau_ext_by_thrust_q_derivative_static_torque",
           &PinocchioRobotModel::computeTauExtByThrustQDerivativeStaticTorque,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("computeTauExtByThrustQDerivativeHessian", &PinocchioRobotModel::computeTauExtByThrustQDerivativeHessian,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("compute_tau_ext_by_thrust_q_derivative_hessian",
           &PinocchioRobotModel::computeTauExtByThrustQDerivativeHessian,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("computeTauExtByThrustQDerivativeNum", &PinocchioRobotModel::computeTauExtByThrustQDerivativeNum,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))
      .def("compute_tau_ext_by_thrust_q_derivative_num", &PinocchioRobotModel::computeTauExtByThrustQDerivativeNum,
           (bp::arg("self"), bp::arg("q"), bp::arg("thrust")))

      .def("getResetConfiguration", &PinocchioRobotModel::getResetConfiguration)
      .def("get_reset_configuration", &PinocchioRobotModel::getResetConfiguration)
      .def("getRotorDirection", &getRotorDirection, (bp::arg("self"), bp::arg("index")))
      .def("get_rotor_direction", &getRotorDirection, (bp::arg("self"), bp::arg("index")))

      .add_property("model", bp::make_function(&getModel, bp::return_internal_reference<>()))
      .add_property("data", bp::make_function(&getData, bp::return_internal_reference<>()))
      .add_property("robot_description", &getRobotDescription)
      .add_property("pinocchio_robot_description", &getPinocchioRobotDescription)
      .add_property("is_floating_base", &getIsFloatingBase)
      .add_property("rotor_num", &getRotorNum)
      .add_property("m_f_rate", &getMFRate)
      .add_property("latest_id_solve_time", &getLatestIdSolveTime)
      .add_property("joint_m_rotors",
                    bp::make_function(&PinocchioRobotModel::getJointMRotors, bp::return_internal_reference<>()))
      .add_property("rotor_frame_indices",
                    bp::make_function(&PinocchioRobotModel::getRotorFrameIndices, bp::return_internal_reference<>()))
      .add_property("rotor_names",
                    bp::make_function(&PinocchioRobotModel::getRotorNames, bp::return_internal_reference<>()))
      .add_property("joint_torque_limits", bp::make_function(&PinocchioRobotModel::getJointTorqueLimits,
                                                             bp::return_value_policy<bp::copy_const_reference>()))
      .add_property("thrust_upper_limits", bp::make_function(&PinocchioRobotModel::getThrustUpperLimits,
                                                             bp::return_value_policy<bp::copy_const_reference>()))
      .add_property("thrust_lower_limits", bp::make_function(&PinocchioRobotModel::getThrustLowerLimits,
                                                             bp::return_value_policy<bp::copy_const_reference>()))
      .add_property("config", bp::make_function(&PinocchioRobotModel::getConfig, bp::return_internal_reference<>()))
      .add_property("thrust_hessian_weight", &getThrustHessianWeight)
      .add_property("nq", &getNq)
      .add_property("nv", &getNv)
      .add_property("njoints", &getNjoints)
      .add_property("nframes", &getNframes);
}

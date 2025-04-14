#pragma once

#include <pinocchio/fwd.hpp>  // should be included before any other pinocchio headers
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/force.hpp>

#include <ros/ros.h>
#include <tinyxml.h>
#include <iostream>
#include <memory>

namespace aerial_robot_dynamics
{
  class PinocchioRobotModel
  {
  public:
    PinocchioRobotModel();
    ~PinocchioRobotModel() = default;

    std::shared_ptr<pinocchio::Model> getModel() const {return model_;}
    std::shared_ptr<pinocchio::Data> getData() const {return data_;}

    Eigen::VectorXd forwardDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& tau, Eigen::VectorXd& thrust);
    bool forwardDynamicsTest(bool verbose = false);
    const int& getRotorNum() const {return rotor_num_;}

  private:
    std::shared_ptr<pinocchio::Model> model_;
    std::shared_ptr<pinocchio::Data> data_;

    int rotor_num_;
    double m_f_rate_;
    double max_thrust_;
    double min_thrust_;
    double joint_torque_limit_;

    std::string getRobotModelXml(const std::string& param_name, ros::NodeHandle nh = ros::NodeHandle());
    Eigen::VectorXd getResetConfiguration();
  };
} // namespace aerial_robot_dynamics

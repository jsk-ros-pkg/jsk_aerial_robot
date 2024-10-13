#pragma once

#include <pinocchio/fwd.hpp>

#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/center-of-mass.hxx>
#include <pinocchio/algorithm/centroidal.hxx>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hxx>
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3-tpl.hpp>
#include <pinocchio/spatial/inertia.hpp>


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// #include <casadi/casadi.hpp>
#include <CasadiEigen/CasadiEigen.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <iostream>
#include <mutex>
#include <tinyxml.h>


namespace aerial_robot_model {
  class PinocchioRobotModel
  {
  public:
    PinocchioRobotModel();
    ~PinocchioRobotModel() = default;

    pinocchio::Model getModelDbl() {return model_dbl_;}
    pinocchio::Data getDataDbl() {return data_dbl_;}
    pinocchio::ModelTpl<casadi::SX> getModel() {return model_;}
    pinocchio::DataTpl<casadi::SX> getData() {return data_;}

    std::vector<casadi::SX> getRotorsOriginFromRoot() {return rotors_origin_from_root_;}
    std::vector<casadi::SX> getRotorsOriginFromCog() {return rotors_origin_from_cog_;}
    std::vector<casadi::SX> getRotorsNormalFromRoot() {return rotors_normal_from_root_;}
    std::vector<casadi::SX> getRotorsNormalFromCog() {return rotors_normal_from_cog_;}

    casadi::SX getQCs() {return q_cs_;}
    Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> getQ() {return q_;}
    casadi::DM getQDbl() {return q_dbl_;}

    casadi::SX getMass() {return mass_;}
    casadi::SX getInertia() {return inertia_;}
    pinocchio::SE3Tpl<casadi::SX> getoMcog() {return oMcog_;}

    std::map<std::string, int> getJointIndexMap() {return joint_index_map_;}

    void updateRobotModel();
    void updateRobotModel(casadi::SX q_cs);
    void updateRobotModel(Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> q);
    void updateRobotModelImpl(Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> q);

  private:
    std::string getRobotModelXml(const std::string param, ros::NodeHandle nh = ros::NodeHandle());

    void modelInit();
    void kinematicsInit();
    void inertialInit();
    void inertialUpdate();
    void rotorInit();
    void rotorUpdate();

    pinocchio::Model model_dbl_;
    pinocchio::Data data_dbl_;
    pinocchio::ModelTpl<casadi::SX> model_;
    pinocchio::DataTpl<casadi::SX> data_;

    std::mutex mutex_cog_pos_;
    std::mutex mutex_cog_rot_;
    std::mutex mutex_inertia_;
    std::mutex mutex_rotor_origin_;
    std::mutex mutex_rotor_normal_;

    std::vector<casadi::SX> rotors_origin_from_root_;
    std::vector<casadi::SX> rotors_origin_from_cog_;
    std::vector<casadi::SX> rotors_normal_from_root_;
    std::vector<casadi::SX> rotors_normal_from_cog_;

    casadi::SX q_cs_;
    Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> q_;
    casadi::DM q_dbl_;

    casadi::SX mass_;
    casadi::SX inertia_;
    pinocchio::SE3Tpl<casadi::SX> oMcog_;

    std::map<std::string, int> joint_index_map_;

    int rotor_num_;
    std::string baselink_;

  protected:
    void setCogPos(const Eigen::Matrix<casadi::SX, 3, 1> cog_pos)
    {
      std::lock_guard<std::mutex> lock(mutex_cog_pos_);
      oMcog_.translation() = cog_pos;
    }
    void setCogRot(const Eigen::Matrix<casadi::SX, 3, 3> cog_rot)
    {
      std::lock_guard<std::mutex> lock(mutex_cog_rot_);
      oMcog_.rotation() = cog_rot;
    }
    void setInertia(const Eigen::Matrix<casadi::SX, 3, 3> inertia)
    {
      std::lock_guard<std::mutex> lock(mutex_inertia_);
      pinocchio::casadi::copy(inertia, inertia_);
    }
    void setRotorsOriginFromRoot(const std::vector<casadi::SX> rotors_origin_from_root)
    {
      std::lock_guard<std::mutex> lock(mutex_rotor_origin_);
      rotors_origin_from_root_ = rotors_origin_from_root;
    }
    void setRotorsOriginFromCog(const std::vector<casadi::SX> rotors_origin_from_cog)
    {
      std::lock_guard<std::mutex> lock(mutex_rotor_origin_);
      rotors_origin_from_cog_ = rotors_origin_from_cog;
    }
    void setRotorsNormalFromRoot(const std::vector<casadi::SX> rotors_normal_from_root)
    {
      std::lock_guard<std::mutex> lock(mutex_rotor_normal_);
      rotors_normal_from_root_ = rotors_normal_from_root;
    }
    void setRotorsNormalFromCog(const std::vector<casadi::SX> rotors_normal_from_cog)
    {
      std::lock_guard<std::mutex> lock(mutex_rotor_normal_);
      rotors_normal_from_cog_ = rotors_normal_from_cog;
    }
  };
}

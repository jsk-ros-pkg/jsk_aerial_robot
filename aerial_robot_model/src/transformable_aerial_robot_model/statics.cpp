#include <aerial_robot_model/transformable_aerial_robot_model.h>

namespace aerial_robot_model {

  void RobotModel::calcStaticThrust()
  {
    calcWrenchMatrixOnRoot(); // update Q matrix
    Eigen::VectorXd wrench_g = calcGravityWrenchOnRoot();
    static_thrust_ = aerial_robot_model::pseudoinverse(q_mat_) * (-wrench_g);
  }

  void RobotModel::calcJointTorque(const bool update_jacobian)
  {
    const auto& sigma = getRotorDirection();
    const auto& joint_positions = getJointPositions();
    const auto& inertia_map = getInertiaMap();
    const int joint_num = getJointNum();
    const int rotor_num = getRotorNum();
    const double m_f_rate = getMFRate();

    if(update_jacobian)
      calcBasicKinematicsJacobian(); // update thrust_coord_jacobians_

    joint_torque_ = Eigen::VectorXd::Zero(joint_num);

    // update coord jacobians for cog point and convert to joint torque
    int seg_index = 0;
    for(const auto& inertia : inertia_map)
      {
        cog_coord_jacobians_.at(seg_index) = RobotModel::getJacobian(joint_positions, inertia.first, inertia.second.getCOG());
        joint_torque_ -= cog_coord_jacobians_.at(seg_index).rightCols(joint_num).transpose() * inertia.second.getMass() * (-gravity_);
        seg_index ++;
      }

    // thrust
    for (int i = 0; i < rotor_num; ++i) {
      Eigen::VectorXd wrench = thrust_wrench_units_.at(i) * static_thrust_(i);
      joint_torque_ -= thrust_coord_jacobians_.at(i).rightCols(joint_num).transpose() * wrench;
    }
  }

  void RobotModel::calcLambdaJacobian()
  {
    // w.r.t root
    const auto& inertia_map = getInertiaMap();
    const auto& rotor_direction = getRotorDirection();
    const auto& joint_positions = getJointPositions();
    const int rotor_num = getRotorNum();
    const int joint_num = getJointNum();
    const int ndof = thrust_coord_jacobians_.at(0).cols();
    const double m_f_rate = getMFRate();
    const int wrench_dof = q_mat_.rows(); // default: 6, under-actuated: 4
    Eigen::MatrixXd q_pseudo_inv = aerial_robot_model::pseudoinverse(q_mat_);

    /* derivative for gravity jacobian */
    Eigen::MatrixXd wrench_gravity_jacobian = Eigen::MatrixXd::Zero(6, ndof);
    for(const auto& inertia : inertia_map){
      wrench_gravity_jacobian.bottomRows(3) -= aerial_robot_model::skew(-inertia.second.getMass() * gravity_3d_) * getSecondDerivativeRoot(inertia.first, inertia.second.getCOG());
    }

    ROS_DEBUG_STREAM("wrench_gravity_jacobian w.r.t. root : \n" << wrench_gravity_jacobian);

    if(wrench_dof == 6) // fully-actuated
      lambda_jacobian_ = -q_pseudo_inv * wrench_gravity_jacobian; // trans, rot
    else // under-actuated
      lambda_jacobian_ = -q_pseudo_inv * (wrench_gravity_jacobian).middleRows(2, wrench_dof); // z, rot

    /* derivative for thrust jacobian */
    std::vector<Eigen::MatrixXd> q_mat_jacobians;
    Eigen::MatrixXd q_inv_jacobian = Eigen::MatrixXd::Zero(6, ndof);
    for (int i = 0; i < rotor_num; ++i) {
      std::string thrust_name = std::string("thrust") + std::to_string(i + 1);
      Eigen::MatrixXd q_mat_jacobian = Eigen::MatrixXd::Zero(6, ndof);

      q_mat_jacobian.bottomRows(3) -= aerial_robot_model::skew(thrust_wrench_units_.at(i).head(3)) * getSecondDerivativeRoot(thrust_name);

      Eigen::MatrixXd wrench_unit_jacobian = Eigen::MatrixXd::Zero(6, ndof);
      wrench_unit_jacobian.topRows(3) = -skew(thrust_wrench_units_.at(i).head(3)) * thrust_coord_jacobians_.at(i).bottomRows(3);
      wrench_unit_jacobian.bottomRows(3) = -skew(thrust_wrench_units_.at(i).tail(3)) * thrust_coord_jacobians_.at(i).bottomRows(3);
      q_mat_jacobian += (thrust_wrench_allocations_.at(i) * wrench_unit_jacobian);

      q_inv_jacobian += (q_mat_jacobian * static_thrust_(i));
      q_mat_jacobians.push_back(q_mat_jacobian);
    }

     if(wrench_dof == 6) // fully-actuated
      lambda_jacobian_ += -q_pseudo_inv * q_inv_jacobian; // trans, rot
    else // under-actuated
      lambda_jacobian_ += -q_pseudo_inv * (q_inv_jacobian).middleRows(2, wrench_dof); // z, rot

    // https://mathoverflow.net/questions/25778/analytical-formula-for-numerical-derivative-of-the-matrix-pseudo-inverse, the third tmer
    Eigen::MatrixXd q_pseudo_inv_jacobian = Eigen::MatrixXd::Zero(rotor_num, ndof);
    Eigen::VectorXd pseudo_wrech = q_pseudo_inv.transpose() * static_thrust_;
    for(int i = 0; i < rotor_num; i++)
      {
        if(wrench_dof == 6) // fully-actuated
          q_pseudo_inv_jacobian.row(i) = pseudo_wrech.transpose() * q_mat_jacobians.at(i);
        else // under-actuated
          q_pseudo_inv_jacobian.row(i) = pseudo_wrech.transpose() * q_mat_jacobians.at(i).middleRows(2, wrench_dof);
      }
    lambda_jacobian_ += (Eigen::MatrixXd::Identity(rotor_num, rotor_num) - q_pseudo_inv * q_mat_) * q_pseudo_inv_jacobian;

    ROS_DEBUG_STREAM("lambda_jacobian: \n" << lambda_jacobian_);
  }

  void RobotModel::calcJointTorqueJacobian()
  {
    const auto& sigma = getRotorDirection();
    const auto& inertia_map = getInertiaMap();
    const double m_f_rate = getMFRate();
    const int rotor_num = getRotorNum();
    const int joint_num = getJointNum();
    const int ndof = lambda_jacobian_.cols();

    joint_torque_jacobian_ = Eigen::MatrixXd::Zero(joint_num, ndof);

    // gravity
    for(const auto& inertia : inertia_map)
      {
        for (int j = 0; j < joint_num; ++j) {
          joint_torque_jacobian_.row(j) += inertia.second.getMass() * (-gravity_.transpose()) * getSecondDerivative(inertia.first, j, inertia.second.getCOG());
        }
      }

    // thrust
    for (int i = 0; i < rotor_num; ++i) {
      Eigen::VectorXd wrench = thrust_wrench_units_.at(i) * static_thrust_(i);
      std::string thrust_name = std::string("thrust") + std::to_string(i + 1);

      for (int j = 0; j < joint_num; ++j) {
        joint_torque_jacobian_.row(j) += wrench.transpose() * getSecondDerivative(thrust_name, j);
      }

      Eigen::MatrixXd wrench_unit_jacobian = Eigen::MatrixXd::Zero(6, ndof);
      wrench_unit_jacobian.topRows(3) = -skew(thrust_wrench_units_.at(i).head(3)) * thrust_coord_jacobians_.at(i).bottomRows(3);
      wrench_unit_jacobian.bottomRows(3) = -skew(thrust_wrench_units_.at(i).tail(3)) * thrust_coord_jacobians_.at(i).bottomRows(3);

      joint_torque_jacobian_ += thrust_coord_jacobians_.at(i).rightCols(joint_num).transpose() * (wrench_unit_jacobian * static_thrust_(i) + thrust_wrench_units_.at(i) * lambda_jacobian_.row(i));
    }
    joint_torque_jacobian_ *= -1;
  }

  Eigen::VectorXd RobotModel::calcGravityWrenchOnRoot()
  {
    const auto& seg_frames = getSegmentsTf();
    const auto& inertia_map = getInertiaMap();

    Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink_).M.Inverse());
    Eigen::VectorXd wrench_g = Eigen::VectorXd::Zero(6);
    KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
    for(const auto& inertia : inertia_map)
      {
        Eigen::MatrixXd jacobi_root = Eigen::MatrixXd::Identity(3, 6);
        Eigen::Vector3d p = root_rot * aerial_robot_model::kdlToEigen(seg_frames.at(inertia.first).p + seg_frames.at(inertia.first).M * inertia.second.getCOG());
        jacobi_root.rightCols(3) = - aerial_robot_model::skew(p);
        wrench_g += jacobi_root.transpose() *  inertia.second.getMass() * (-gravity_);
      }
    return wrench_g;
  }

  Eigen::MatrixXd RobotModel::calcWrenchMatrixOnCoG()
  {
    const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
    const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
    const auto& sigma = getRotorDirection();
    const auto& rotor_direction = getRotorDirection();
    const int rotor_num = getRotorNum();
    const double m_f_rate = getMFRate();

    //Q : WrenchAllocationMatrix
    Eigen::MatrixXd Q(6, rotor_num);
    for (unsigned int i = 0; i < rotor_num; ++i) {
      Q.block(0, i, 3, 1) = u.at(i);
      Q.block(3, i, 3, 1) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
    }
    return Q;
  }

  void RobotModel::calcWrenchMatrixOnRoot()
  {
    const auto& seg_frames = getSegmentsTf();
    const std::vector<Eigen::Vector3d>& u = getRotorsNormalFromCog<Eigen::Vector3d>();
    const auto& sigma = getRotorDirection();
    const int rotor_num = getRotorNum();
    const double m_f_rate = getMFRate();

    Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink_).M.Inverse());

    q_mat_ = Eigen::MatrixXd::Zero(6, rotor_num);
    for (unsigned int i = 0; i < rotor_num; ++i) {
      std::string rotor = "thrust" + std::to_string(i + 1);
      Eigen::MatrixXd q_i = Eigen::MatrixXd::Identity(6, 6);
      Eigen::Vector3d p = root_rot * aerial_robot_model::kdlToEigen(seg_frames.at(rotor).p);
      q_i.bottomLeftCorner(3,3) = aerial_robot_model::skew(p);

      Eigen::VectorXd wrench_unit = Eigen::VectorXd::Zero(6);
      wrench_unit.head(3) = u.at(i);
      wrench_unit.tail(3) = m_f_rate * sigma.at(i + 1) * u.at(i);

      thrust_wrench_units_.at(i) = wrench_unit;
      thrust_wrench_allocations_.at(i) = q_i;
      q_mat_.col(i) = q_i * wrench_unit;
    }
  }

  void RobotModel::staticsInit()
  {
    /* get baselink and thrust_link from robot model */
    auto robot_model_xml = getRobotModelXml("robot_description");

    /* set rotor property */
    TiXmlElement* m_f_rate_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("m_f_rate");
    if(!m_f_rate_attr)
      ROS_ERROR("Can not get m_f_rate attribute from urdf model");
    else
      m_f_rate_attr->Attribute("value", &m_f_rate_);

    std::vector<urdf::LinkSharedPtr> urdf_links;
    model_.getLinks(urdf_links);
    for(const auto& link: urdf_links)
      {
        if(link->parent_joint)
          {
            if(link->parent_joint->name == "rotor1")
              {
                thrust_max_ = link->parent_joint->limits->upper;
                thrust_min_ = link->parent_joint->limits->lower;
                break;
              }
          }
      }

    const int full_body_dof = 6 + joint_num_;
    q_mat_.resize(6, rotor_num_);
    gravity_.resize(6);
    gravity_ <<  0, 0, 9.80665, 0, 0, 0;
    gravity_3d_.resize(3);
    gravity_3d_ << 0, 0, 9.80665;
    lambda_jacobian_.resize(rotor_num_, full_body_dof);
    joint_torque_.resize(joint_num_);
    joint_torque_jacobian_.resize(joint_num_, full_body_dof);
    static_thrust_.resize(rotor_num_);
    thrust_wrench_units_.resize(rotor_num_);
    thrust_wrench_allocations_.resize(rotor_num_);
  }

} //namespace aerial_robot_model

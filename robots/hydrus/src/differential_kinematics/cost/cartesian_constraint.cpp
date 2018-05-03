#include <hydrus/differential_kinematics/cost/cartesian_constraint.h>

namespace differential_kinematics
{
  namespace cost
  {
    template <class motion_planner>
    void CartersianConstraint<motion_planner>::initialize(ros::NodeHandle nh,
                                                          ros::NodeHandle nhp,
                                                          boost::shared_ptr<motion_planner> planner,
                                                          std::string cost_name,
                                                          bool orientation, bool full_body)
    {
      Base<motion_planner>::initialize(nh, nhp, planner, cost_name, orientation, full_body);

      Base<motion_planner>::nhp_.param ("pos_convergence_thre", pos_convergence_thre_, 0.001); //[m]
      if(Base<motion_planner>::verbose_) std::cout << "pos_convergence_thre: " << std::setprecision(3) << pos_convergence_thre_ << std::endl;
      Base<motion_planner>::nhp_.param ("rot_convergence_thre", rot_convergence_thre_, 0.017); //[rad]
      if(Base<motion_planner>::verbose_) std::cout << "rot_convergence_thre: " << std::setprecision(3) << rot_convergence_thre_ << std::endl;
      Base<motion_planner>::nhp_.param ("pos_err_max", pos_err_max_, 0.1); //[m]
      if(Base<motion_planner>::verbose_) std::cout << "pos_err_max: " << std::setprecision(3) << pos_err_max_ << std::endl;
      Base<motion_planner>::nhp_.param ("rot_err_max", rot_err_max_, 0.17); //[rad]
      if(Base<motion_planner>::verbose_) std::cout << "rot_err_max: " << std::setprecision(3) << rot_err_max_ << std::endl;
      Base<motion_planner>::nhp_.param ("w_cartesian_err_constraint", w_cartesian_err_constraint_, 1.0);
      if(Base<motion_planner>::verbose_) std::cout << "w_cartesian_err_constraint: " << std::setprecision(3) << w_cartesian_err_constraint_ << std::endl;

      W_cartesian_err_constraint_ = Eigen::MatrixXd::Identity(6, 6) * w_cartesian_err_constraint_;
      if(!Base<motion_planner>::orientation_) W_cartesian_err_constraint_.block(3, 3, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
    }

    template <class motion_planner>
    bool CartersianConstraint<motion_planner>::getHessianGradient(bool& convergence, Eigen::MatrixXd& H, Eigen::VectorXd& g, bool debug)
    {
      convergence = false;

      KDL::Frame end_frame;
      KDL::ChainFkSolverPos_recursive fk_solver(chain_);
      KDL::JntArray joint_positions(chain_.getNrOfJoints());
      joint_positions.data = Base<motion_planner>::planner_->getTargetJointVector();
      fk_solver.JntToCart(joint_positions, end_frame);

      tf::Transform end_tf;
      tf::transformKDLToTF(end_frame, end_tf);
      if(debug)
        ROS_WARN("end x: %f,  y: %f, yaw: %f, target end  [%f, %f, %f], yaw: %f", end_tf.getOrigin().x(), end_tf.getOrigin().y(), tf::getYaw(end_tf.getRotation()), target_reference_frame_.getOrigin().x(), target_reference_frame_.getOrigin().y(), target_reference_frame_.getOrigin().z(), tf::getYaw(target_reference_frame_.getRotation()));
      tf::Transform tf_err =  (Base<motion_planner>::planner_->getTargetRootPose() * end_tf).inverse() * target_reference_frame_;
      //tf::Transform tf_err =  end_tf.inverse() * target_reference_frame_;
      double pos_err = tf_err.getOrigin().length();
      double rot_err = fabs(tf_err.getRotation().getAngleShortestPath());

      if(debug)
        ROS_INFO("pos err, rot(angle): %f[m], %f[rad], err pos vec from end tf : [%f, %f, %f]", pos_err, rot_err, tf_err.getOrigin().x(), tf_err.getOrigin().y(), tf_err.getOrigin().z());

      /* check convergence */
      if(pos_err < pos_convergence_thre_ ) /* position convergence */
        {
          if(!Base<motion_planner>::orientation_)  convergence = true;
          else
            if(rot_err < rot_convergence_thre_) convergence = true;
        }

      /* transform the cartesian err to the root link frame */
      if(pos_err > pos_err_max_) pos_err = pos_err_max_;
      if(rot_err > rot_err_max_) rot_err = rot_err_max_;

      tf::Vector3 target_pos_err_root_link = end_tf.getBasis() * tf_err.getOrigin().normalize() * pos_err;
      tf::Vector3 target_rot_err_root_link = end_tf.getBasis() * tf_err.getRotation().getAxis() * rot_err;

      Eigen::VectorXd delta_cartesian = Eigen::VectorXd::Zero(6);
      Eigen::Vector3d temp_vec;
      tf::vectorTFToEigen(target_pos_err_root_link, temp_vec);
      delta_cartesian.head(3) = temp_vec;
      tf::vectorTFToEigen(target_rot_err_root_link, temp_vec);
      delta_cartesian.tail(3) = temp_vec;

#if 1
      if(Base<motion_planner>::planner_->getMultilinkType() == motion_planner::MULTILINK_TYPE_SE2)
        {
          if(!Base<motion_planner>::orientation_) delta_cartesian.segment(2, 4) = Eigen::VectorXd::Zero(4);
          else delta_cartesian.segment(2, 3) = Eigen::VectorXd::Zero(3);
        }
      else
        {
          if(!Base<motion_planner>::orientation_) delta_cartesian.segment(3, 3) = Eigen::VectorXd::Zero(3);
        }
#endif

      if(debug) std::cout << "delta cartesian: \n" << delta_cartesian.transpose() << std::endl;

      Eigen::MatrixXd jacobian;
      if(!calcJointJacobian(jacobian, debug)) return false;

      /*
        QP:
        - cost function : (J dq - dx)^T W1 (J dq - dx) + dq^T W2 dq  :  The least square error term for equality conditions(e.g. IK)  + vel (i.e. joint, root) penality
        - dq^T (J^T W1 J + W2) dq - 2 dx^T W1 J dq + hoge
        - since:-2 dx^T W1 J = -2 (J^T W1^T dx)^T = -2 (J^T W1 dx)^T
      */
      H = jacobian.transpose() * W_cartesian_err_constraint_ * jacobian;
      g = - delta_cartesian.transpose()  * W_cartesian_err_constraint_ * jacobian;

      if(debug)
        {
          std::cout << "the Weight Matrix of cartesian_err constraint: \n" << W_cartesian_err_constraint_ << std::endl;
          std::cout << "cost name: " << Base<motion_planner>::cost_name_ << ", H: \n" << H << std::endl;
          std::cout << "cost name: " << Base<motion_planner>::cost_name_ << ", g: \n" << g.transpose() << std::endl;
        }

      return true;
    }

    template <class motion_planner>
    bool CartersianConstraint<motion_planner>::calcJointJacobian(Eigen::MatrixXd& jacobian, bool debug)
    {
      jacobian = Eigen::MatrixXd::Zero(6, chain_.getNrOfJoints() + 6);

      /* fill the joint state */
      KDL::JntArray joint_positions(chain_.getNrOfJoints());
      joint_positions.data = Base<motion_planner>::planner_->getTargetJointVector();

      /* calculate the jacobian */
      KDL::ChainJntToJacSolver jac_solver(chain_);
      KDL::Jacobian jac(chain_.getNrOfJoints());
      if(jac_solver.JntToJac(joint_positions, jac) == KDL::SolverI::E_NOERROR)
        {
#if 0 
          if(1)
#else
            if(Base<motion_planner>::full_body_)
#endif
              {
                /* consider root is attached with a 6Dof free joint */

                /* get end frame position from root */
                KDL::Frame end_frame;
                KDL::ChainFkSolverPos_recursive fk_solver(chain_);
                fk_solver.JntToCart(joint_positions, end_frame);

                if(debug)
                  ROS_INFO("end pose: [%f %f %f]", end_frame.p.x(), end_frame.p.y(), end_frame.p.z());

                /* root link */
                jacobian.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
                jacobian.block(0, 3, 3, 1) = (Eigen::Vector3d(1, 0, 0)).cross(Eigen::Vector3d(end_frame.p.data));
                jacobian.block(0, 4, 3, 1) = (Eigen::Vector3d(0, 1, 0)).cross(Eigen::Vector3d(end_frame.p.data));
                jacobian.block(0, 5, 3, 1) = (Eigen::Vector3d(0, 0, 1)).cross(Eigen::Vector3d(end_frame.p.data));
              }

          /* joint part */
          jacobian.block(0, 6, 6, jac.data.cols()) = jac.data;

#if 1
          /* special */
          if(Base<motion_planner>::planner_->getMultilinkType() == motion_planner::MULTILINK_TYPE_SE2)
            {
              if(!Base<motion_planner>::orientation_)
                jacobian.block(2, 0, 4, jacobian.cols()) = Eigen::MatrixXd::Zero(4, jacobian.cols());
              else
                jacobian.block(2, 0, 3, jacobian.cols()) = Eigen::MatrixXd::Zero(3, jacobian.cols());
            }
          else
            {
              if(!Base<motion_planner>::orientation_)
                jacobian.block(3, 0, 3, jacobian.cols()) = Eigen::MatrixXd::Zero(3, jacobian.cols());
            }
#endif

          if(debug) std::cout << "jacobian: \n" << jacobian << std::endl;
          return true;
        }

      ROS_WARN("cost (%s) can not calculate the jacobian", Base<motion_planner>::cost_name_.c_str());
      return false;
    }
  };
};

#include <pluginlib/class_list_macros.h>
#include <hydrus/differential_kinematics/planner_core.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::cost::CartersianConstraint<differential_kinematics::Planner>, differential_kinematics::cost::Base<differential_kinematics::Planner>);

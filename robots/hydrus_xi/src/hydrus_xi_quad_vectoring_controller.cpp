#include <hydrus_xi/hydrus_xi_quad_vectoring_controller.h>

namespace
{
  int cnt = 0;
  int invalid_cnt = 0;

  double minYawMoment(const std::vector<double> &x, std::vector<double> &grad, void *controller_ptr)
  {
    cnt++;
    QuadVectoringController *controller = reinterpret_cast<QuadVectoringController*>(controller_ptr);
    assert(grad.empty());

    /* calculate the vectoring */
    //ROS_INFO("obj func process, [%f, %f]", x.at(0), x.at(1)); //debug, to confirm the x is same between objective and constarint

    /* update robot model */
    for(int i = 0; i < x.size(); i++)
      {
        //ROS_INFO("debug: gimbal_name: %s, angle: %f", joint_state_.name.at(gimbal_index_.at(i)).c_str(), x.at(i));
        controller->joint_state_.position.at(controller->gimbal_index_.at(i)) = x.at(i);
      }

    assert(x.size() == controller->gimbal_names_.size());
    controller->getRobotModelPtr()->updateRobotModel(controller->joint_state_);



    if(!controller->getRobotModelPtr()->modelling())
      {
        invalid_cnt ++;
        if(controller->verbose_) ROS_WARN("invliad modelling, %d", invalid_cnt);
      }
    else
      {
        invalid_cnt = 0;

        /* 1. calculate the max and min yaw torque by LP */

        Eigen::VectorXd gradient = controller->getRobotModelPtr()->getQtau().row(2).transpose();
        Eigen::VectorXd max_u, min_u;
        double max_yaw, min_yaw;

        /* get min u and min yaw */
        controller->getYawRangeLPSolver().updateGradient(gradient);
        if(!controller->getYawRangeLPSolver().solve())
          {
            ROS_ERROR("cat not calcualte the min u by LP");
            controller->max_min_yaw_ = 0;
          }
        else
          {
            min_u = controller->getYawRangeLPSolver().getSolution();
            //std::cout << "min_u: " << min_u.transpose() << std::endl;
            min_yaw = (gradient.transpose() * min_u)(0);
            if(min_yaw > 0)
              {
                ROS_WARN("the min yaw is positive: %f", min_yaw);
                min_yaw = 0;
              }
          }

        /* get max u and max yaw */
        //std::cout << "yaw moment map: " << gradient.transpose() << std::endl;
        Eigen::VectorXd reverse_gradient = -1 * gradient;
        controller->getYawRangeLPSolver().updateGradient(reverse_gradient);
        if(!controller->getYawRangeLPSolver().solve())
          {
            ROS_ERROR("cat not calcualte the max u by LP");
            max_yaw = 0;
          }
        else
          {
            max_u = controller->getYawRangeLPSolver().getSolution();
            //std::cout << "max_u: " << max_u.transpose() << std::endl;
            max_yaw = (gradient.transpose() * max_u)(0);
            assert(max_yaw > 0);
          }

        //ROS_INFO("LP: max: %f, min: %f", max_yaw, min_yaw); //debug
        controller->max_min_yaw_ = std::min(max_yaw, -min_yaw);
      }

    Eigen::VectorXd force_v = controller->getRobotModelPtr()->getOptimalHoveringThrust();
    double average_force = force_v.sum()/ force_v.size();
    double variant = 0;

    for(int i = 0; i < force_v.size(); i++)
      variant += ((force_v(i) - average_force) * (force_v(i) - average_force));

    variant = sqrt(variant / force_v.size());
    //ROS_INFO("force norm: %f, var: %f", force_v.norm(), variant);
    //return force_v.norm() + 2 * variant; // only force
    //return max_yaw; // only yaw moment
    return controller->force_rate_ * (controller->getRobotModelPtr()->getMass() * 2 / force_v.norm() + 0.01 / variant) + controller->yaw_rate_ * controller->max_min_yaw_;
  }

  double attConstraint(const std::vector<double> &x, std::vector<double> &grad, void *controller_ptr)
  {
    //if(!valid_pose_) return 1; //invliad

    //ROS_INFO("constraint func process, [%f, %f]", x.at(0), x.at(1)); //debug

    QuadVectoringController *controller = reinterpret_cast<QuadVectoringController*>(controller_ptr);
    assert(grad.empty());

    double ez_x = controller->getRobotModelPtr()->getCogDesireOrientation<Eigen::Matrix3d>()(0,2);
    double ez_y = controller->getRobotModelPtr()->getCogDesireOrientation<Eigen::Matrix3d>()(1,2);
    double ez_z = controller->getRobotModelPtr()->getCogDesireOrientation<Eigen::Matrix3d>()(2,2);
    double attitude = atan2(sqrt(ez_x* ez_x + ez_y * ez_y), fabs(ez_z));
    return attitude - controller->attitude_thresh_; //hard-coding: approxi 0.02 [rad]
  }
};

QuadVectoringController::QuadVectoringController(ros::NodeHandle nh, ros::NodeHandle nhp):
    nh_(nh), nhp_(nhp),
    opt_gimbal_angles_(0),
    prev_opt_gimbal_angles_(0),
    max_min_yaw_(0)
{
    robot_model_ptr_ = std::make_shared<HydrusRobotModel>(true);

    nhp_.param("verbose", verbose_, false);
    nhp_.param("simulation", simulation_, false);
    nhp_.param("real_machine", real_machine_, false);
    nhp_.param("delta_angle", delta_angle_, 0.2);
    nhp_.param("force_rate", force_rate_, 1.0);
    nhp_.param("yaw_rate", yaw_rate_, 1.0);
    nhp_.param("attitude_thresh", attitude_thresh_, 0.02);

    actuator_state_sub_ = nh_.subscribe("joint_states", 1, &QuadVectoringController::actuatorStateCallback, this);

    if((simulation_ && !real_machine_) || (!simulation_ && real_machine_))
      gimbal_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);

    gimbal_names_ = {"gimbal1", "gimbal4"}; //ends gimbals. TODO: configurable, using v-string
    gimbal_names_ = {"gimbal1", "gimbal2", "gimbal3", "gimbal4"}; //whole gimbals. TODO: configurable
    gimbal_index_.resize(0),

    /* nonlinear optimization for vectoring angles planner */
    vectoring_nl_solver_ = std::make_shared<nlopt::opt>(nlopt::LN_COBYLA, gimbal_names_.size());
    vectoring_nl_solver_->set_max_objective(minYawMoment, this);

    vectoring_nl_solver_->add_inequality_constraint(attConstraint, this, 1e-8);
    vectoring_nl_solver_->set_xtol_rel(1e-4); //1e-4
    vectoring_nl_solver_->set_maxeval(1000); // 1000 times
    /* linear optimization for yaw range */
    double rotor_num = robot_model_ptr_->getRotorNum();

    // settings the LP solver
    yaw_range_lp_solver_.settings()->setVerbosity(false);
    yaw_range_lp_solver_.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    yaw_range_lp_solver_.data()->setNumberOfVariables(rotor_num);
    yaw_range_lp_solver_.data()->setNumberOfConstraints(rotor_num);

    // allocate LP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    hessian.resize(rotor_num, rotor_num);

    Eigen::VectorXd gradient = Eigen::VectorXd::Ones(rotor_num);
    Eigen::SparseMatrix<double> linear_cons;
    linear_cons.resize(rotor_num, rotor_num);
    for(int i = 0; i < linear_cons.cols(); i++) linear_cons.insert(i,i) = 1;

    Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(rotor_num);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Ones(rotor_num) * robot_model_ptr_->getFMax();

    yaw_range_lp_solver_.data()->setHessianMatrix(hessian);
    yaw_range_lp_solver_.data()->setGradient(gradient);
    yaw_range_lp_solver_.data()->setLinearConstraintsMatrix(linear_cons);
    yaw_range_lp_solver_.data()->setLowerBound(lower_bound);
    yaw_range_lp_solver_.data()->setUpperBound(upper_bound);

    // instantiate the yaw_range_lp_solver
    if(!yaw_range_lp_solver_.initSolver())
      throw std::runtime_error("can not init LP solver based on osqp");
}

void QuadVectoringController::actuatorStateCallback(const sensor_msgs::JointStateConstPtr& state)
{
  joint_state_ = *state;

  if(robot_model_ptr_->getActuatorJointMap().empty()) robot_model_ptr_->setActuatorJointMap(joint_state_);

  if(gimbal_index_.size() == 0)
    {
      for(auto name: gimbal_names_)
        {
          auto itr = std::find(joint_state_.name.begin(), joint_state_.name.end(), name);
          if(itr != joint_state_.name.end())
            gimbal_index_.push_back(std::distance(joint_state_.name.begin(), itr));
          else
            {
              ROS_ERROR("cannot find %s in sensor_msgs::JointState", name.c_str());
              return;
            }
        }

      /* confirm the joint pose is valid in simulation mode */
      if(simulation_ && !real_machine_)
        {
          for(int i = 0; i < joint_state_.name.size(); i++)
            {
              if(joint_state_.name.at(i).find("joint") != std::string::npos)
                {
                  if(fabs(joint_state_.position.at(i) - 2 * M_PI / robot_model_ptr_->getRotorNum()) > 0.1)
                    {
                      gimbal_index_.resize(0);
                      return;
                    }
                }
            }
          ROS_WARN("hydrus_xi vectoring controller: the joint pose is valid");
        }
    }

  /* find the best gimbal vectoring angles by nlopt */
  std::vector<double> lb(gimbal_names_.size(), - 2 * M_PI);
  std::vector<double> ub(gimbal_names_.size(), 2 * M_PI);

  /* update the range by using the last optimization result with the assumption that the motion is cotinuous */
  if(opt_gimbal_angles_.size() != 0)
    {
      for(int i = 0; i < opt_gimbal_angles_.size(); i++)
        {
          lb.at(i) = opt_gimbal_angles_.at(i) - delta_angle_;
          ub.at(i) = opt_gimbal_angles_.at(i) + delta_angle_;
        }
    }
  else
    {
      opt_gimbal_angles_.resize(gimbal_names_.size(), 0); // all angles  are zero
      /* heuristic assigment for the init state of vectoring angles */

      if(gimbal_index_.size() == robot_model_ptr_->getRotorNum())
        {
          for(int i = 0; i < gimbal_index_.size(); i++)
            {
              if(i%2 == 1) opt_gimbal_angles_.at(i) = M_PI;
            }
        }

    }
  assert(gimbal_index_.size() != 0);

  //for(int i = 0; i < opt_gimbal_angles_.size(); i++) ROS_INFO("[%f, %f]", lb.at(i), ub.at(i));
  vectoring_nl_solver_->set_lower_bounds(lb);
  vectoring_nl_solver_->set_upper_bounds(ub);

  double start_time = ros::Time::now().toSec();
  double max_f = 0;
  try
    {
      nlopt::result result = vectoring_nl_solver_->optimize(opt_gimbal_angles_, max_f);

      double roll,pitch,yaw;
      robot_model_ptr_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

      std::for_each(opt_gimbal_angles_.begin(), opt_gimbal_angles_.end(),
                    [this](double& x) mutable {
                      if(x < -M_PI - delta_angle_) x += (2 * M_PI);
                      else if (x > M_PI + delta_angle_) x -= (2 * M_PI);});

      if(prev_opt_gimbal_angles_.size() == 0) prev_opt_gimbal_angles_ = opt_gimbal_angles_;

      if(verbose_)
        {
          std::cout << "nlopt: " << std::setprecision(7)
                    << ros::Time::now().toSec() - start_time  <<  "[sec], cnt: " << cnt;
          std::cout << ", found max min yaw at gimbal: [";
          for(auto it: opt_gimbal_angles_) std::cout << std::setprecision(5) << it << " ";
          std::cout << "], atttidue: [" << roll << ", " << pitch;
          std::cout << "], force: [" << robot_model_ptr_->getOptimalHoveringThrust().transpose();
          std::cout << "] = " << std::setprecision(10) << max_min_yaw_ << std::endl;
        }

      cnt = 0;
    }
  catch(std::exception &e)
    {
      std::cout << "nlopt failed: " << e.what() << std::endl;
    }

  // lb = vectoring_nl_solver_->get_lower_bounds();
  // ub = vectoring_nl_solver_->get_upper_bounds();
  // for(int i = 0; i < opt_gimbal_angles_.size(); i++) ROS_INFO("[%f, %f]", lb.at(i), ub.at(i));

  /* publish the gimbal angles if necessary */
  if((simulation_ && !real_machine_) || (!simulation_ && real_machine_))
    {
      sensor_msgs::JointState ctrl_pub;
      ctrl_pub.header = joint_state_.header;

      for(int i = 0; i < gimbal_names_.size(); i++)
        {
          ctrl_pub.name.push_back(gimbal_names_.at(i));
          if(simulation_)
            {
              if(opt_gimbal_angles_.at(i) - prev_opt_gimbal_angles_.at(i) > M_PI)
                ctrl_pub.position.push_back(opt_gimbal_angles_.at(i) - 2 * M_PI);
              else if(opt_gimbal_angles_.at(i) - prev_opt_gimbal_angles_.at(i) < -M_PI)
                ctrl_pub.position.push_back(opt_gimbal_angles_.at(i) + 2 * M_PI);
              else ctrl_pub.position.push_back(opt_gimbal_angles_.at(i));
            }
          else
            ctrl_pub.position.push_back(opt_gimbal_angles_.at(i));
        }
      gimbal_ctrl_pub_.publish(ctrl_pub);
    }

  prev_opt_gimbal_angles_ = opt_gimbal_angles_;
}

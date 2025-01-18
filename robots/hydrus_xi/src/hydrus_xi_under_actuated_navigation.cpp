#include <hydrus_xi/hydrus_xi_under_actuated_navigation.h>

using namespace aerial_robot_navigation;

namespace
{
  int cnt = 0;
  int invalid_cnt = 0;

  double maximizeFCTMin(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    cnt++;
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();
    /* update robot model */
    KDL::JntArray joint_positions = planner->getJointPositionsForPlan();
    for(int i = 0; i < x.size(); i++)
      joint_positions(planner->getControlIndices().at(i)) = x.at(i);

    robot_model->updateRobotModel(joint_positions);

    if(!robot_model->stabilityCheck(planner->getPlanVerbose()))
      {
        invalid_cnt ++;
        std::stringstream ss;
        for(const auto& angle: x) ss << angle << ", ";
        if(planner->getPlanVerbose()) ROS_WARN_STREAM("nlopt, robot stability is invalid with gimbals: " << ss.str() << " (cnt: " << invalid_cnt << ")");
        return 0;
      }

    invalid_cnt = 0;

    Eigen::VectorXd force_v = robot_model->getStaticThrust();
    double average_force = force_v.sum() / force_v.size();
    double variant = 0;

    for(int i = 0; i < force_v.size(); i++)
      variant += ((force_v(i) - average_force) * (force_v(i) - average_force));

    variant = sqrt(variant / force_v.size());

    return planner->getForceNormWeight() * robot_model->getMass() / force_v.norm()  + planner->getForceVariantWeight() / variant + planner->getFCTMinWeight() * robot_model->getFeasibleControlTMin();
  }

  double maximizeMinYawTorque(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    cnt++;
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();

    /* update robot model */
    KDL::JntArray joint_positions = planner->getJointPositionsForPlan();
    for(int i = 0; i < x.size(); i++)
      joint_positions(planner->getControlIndices().at(i)) = x.at(i);

    robot_model->updateRobotModel(joint_positions);

    if(!robot_model->stabilityCheck(planner->getPlanVerbose()))
      {
        invalid_cnt ++;
        if(planner->getPlanVerbose()) ROS_WARN("nlopt, robot stability is invalid (cnt: %d)", invalid_cnt);
        return 0;
      }
    else
      {
        invalid_cnt = 0;

        /* 1. calculate the max and min yaw torque by LP */
        Eigen::VectorXd gradient = robot_model->calcWrenchMatrixOnCoG().row(5).transpose();
        Eigen::VectorXd max_u, min_u;
        double max_yaw, min_yaw;

        //std::cout << "yaw torque map: " << gradient.transpose() << std::endl;

        /* get min u and min yaw */
        planner->getYawRangeLPSolver().updateGradient(gradient);
        if(!planner->getYawRangeLPSolver().solve())
          {
            ROS_ERROR("cat not calcualte the min u by LP");
            planner->setMaxMinYaw(0);
          }
        else
          {
            min_u = planner->getYawRangeLPSolver().getSolution();
            //std::cout << "min_u: " << min_u.transpose() << std::endl;
            min_yaw = (gradient.transpose() * min_u)(0);
            if(min_yaw > 0)
              {
                ROS_WARN("the min yaw is positive: %f", min_yaw);
                min_yaw = 0;
              }
          }

        /* get max u and max yaw */
        Eigen::VectorXd reverse_gradient = - gradient;
        planner->getYawRangeLPSolver().updateGradient(reverse_gradient);
        if(!planner->getYawRangeLPSolver().solve())
          {
            ROS_ERROR("cat not calcualte the max u by LP");
            planner->setMaxMinYaw(0);
          }
        else
          {
            max_u = planner->getYawRangeLPSolver().getSolution();
            max_yaw = (gradient.transpose() * max_u)(0);
          }

        //ROS_INFO("LP: max: %f, min: %f", max_yaw, min_yaw); //debug
        planner->setMaxMinYaw(std::min(max_yaw, -min_yaw));
      }

    Eigen::VectorXd force_v = robot_model->getStaticThrust();
    double average_force = force_v.sum() / force_v.size();
    double variant = 0;

    for(int i = 0; i < force_v.size(); i++)
      variant += ((force_v(i) - average_force) * (force_v(i) - average_force));

    variant = sqrt(variant / force_v.size());

    return planner->getForceNormWeight() * robot_model->getMass() / force_v.norm()  + planner->getForceVariantWeight() / variant + planner->getYawTorqueWeight() * planner->getMaxMinYaw();
  }

  double baselinkRotConstraint(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto baselink_rot = planner->getRobotModelForPlan()->getCogDesireOrientation<Eigen::Matrix3d>();

    double ez_x = baselink_rot(0,2);
    double ez_y = baselink_rot(1,2);
    double ez_z = baselink_rot(2,2);
    double angle = atan2(sqrt(ez_x* ez_x + ez_y * ez_y), fabs(ez_z));

    return angle - planner->getBaselinkRotThresh();
  }


  double fcTMinConstraint(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    return planner->getFCTMinThresh() - planner->getRobotModelForPlan()->getFeasibleControlTMin();
  }

};

HydrusXiUnderActuatedNavigator::HydrusXiUnderActuatedNavigator():
    opt_gimbal_angles_(0),
    prev_opt_gimbal_angles_(0),
    max_min_yaw_(0),
    control_gimbal_names_(0),
    control_gimbal_indices_(0)
{
}

HydrusXiUnderActuatedNavigator::~HydrusXiUnderActuatedNavigator()
{
  plan_thread_.join();
}

void HydrusXiUnderActuatedNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                double loop_du)
{
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  robot_model_for_plan_ = boost::make_shared<HydrusTiltedRobotModel>(); // for planning, not the real robot model

  rosParamInit();

  gimbal_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);

  if(nh.hasParam("control_gimbal_names"))
    {
      nh.getParam("control_gimbal_names", control_gimbal_names_);
    }
  else
    {
      ROS_INFO("load control gimbal list from robot model");
      for(const auto& name: robot_model->getJointNames())
        {
          if(name.find("gimbal") != std::string::npos)
            {
              control_gimbal_names_.push_back(name);
              ROS_INFO_STREAM("add " << name);
            }
        }
    }

  /* nonlinear optimization for vectoring angles planner */
  vectoring_nl_solver_ = boost::make_shared<nlopt::opt>(nlopt::LN_COBYLA, control_gimbal_names_.size());
  if(maximize_yaw_)
    {
      vectoring_nl_solver_->set_max_objective(maximizeMinYawTorque, this);
      vectoring_nl_solver_->add_inequality_constraint(fcTMinConstraint, this, 1e-8);
    }
  else
    vectoring_nl_solver_->set_max_objective(maximizeFCTMin, this);

  vectoring_nl_solver_->add_inequality_constraint(baselinkRotConstraint, this, 1e-8);

  vectoring_nl_solver_->set_xtol_rel(1e-4); //1e-4
  vectoring_nl_solver_->set_maxeval(1000); // 1000 times
  /* linear optimization for yaw range */
  double rotor_num = robot_model->getRotorNum();

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

  Eigen::VectorXd lower_bound = Eigen::VectorXd::Ones(rotor_num) * robot_model->getThrustLowerLimit();
  Eigen::VectorXd upper_bound = Eigen::VectorXd::Ones(rotor_num) * robot_model->getThrustUpperLimit();

  yaw_range_lp_solver_.data()->setHessianMatrix(hessian);
  yaw_range_lp_solver_.data()->setGradient(gradient);
  yaw_range_lp_solver_.data()->setLinearConstraintsMatrix(linear_cons);
  yaw_range_lp_solver_.data()->setLowerBound(lower_bound);
  yaw_range_lp_solver_.data()->setUpperBound(upper_bound);

  // instantiate the yaw_range_lp_solver
  if(!yaw_range_lp_solver_.initSolver())
    throw std::runtime_error("can not init LP solver based on osqp");

  plan_thread_ = std::thread(boost::bind(&HydrusXiUnderActuatedNavigator::threadFunc, this));
}

void HydrusXiUnderActuatedNavigator::threadFunc()
{
  double plan_freq;
  ros::NodeHandle navi_nh(nh_, "navigation");
  getParam<double>(navi_nh, "plan_freq", plan_freq, 20.0);
  ros::Rate loop_rate(plan_freq);

  // sleep for initialization
  double plan_init_sleep;
  getParam<double>(navi_nh, "plan_init_sleep", plan_init_sleep, 2.0);
  ros::Duration(plan_init_sleep).sleep();

  while(ros::ok())
    {
      plan();
      loop_rate.sleep();
    }
}

bool HydrusXiUnderActuatedNavigator::plan()
{
  joint_positions_for_plan_ = robot_model_->getJointPositions(); // real

  if(joint_positions_for_plan_.rows() == 0) return false;

  // initialize from the normal shape
  bool singular_form = true;
  if(control_gimbal_indices_.size() == 0)
    {
      const auto& joint_names = robot_model_->getJointNames();
      const auto& joint_indices = robot_model_->getJointIndices();

      for(int i = 0; i < joint_names.size(); i++)
        {
          if(joint_names.at(i).find("joint") != std::string::npos)
            {
              if(fabs(joint_positions_for_plan_(joint_indices.at(i))) > 0.2) singular_form = false;
            }
        }

      for(const auto& name: control_gimbal_names_)
        control_gimbal_indices_.push_back(robot_model_->getJointIndexMap().at(name));
    }

  /* find the optimal gimbal vectoring angles from nlopt */
  std::vector<double> lb(control_gimbal_indices_.size(), - M_PI);
  std::vector<double> ub(control_gimbal_indices_.size(), M_PI);

  /* update the range by using the last optimization result with the assumption that the motion is cotinuous */
  if(opt_gimbal_angles_.size() != 0)
    {
      double delta_angle = gimbal_delta_angle_;

      if(!robot_model_for_plan_->stabilityCheck(false))
        {
          delta_angle = M_PI; // reset
        }

      for(int i = 0; i < opt_gimbal_angles_.size(); i++)
         {
           lb.at(i) = opt_gimbal_angles_.at(i) - delta_angle;
           ub.at(i) = opt_gimbal_angles_.at(i) + delta_angle;
         }
    }
  else
    {
      /* heuristic assigment for the init state of vectoring angles */

      opt_gimbal_angles_.resize(control_gimbal_indices_.size(), 0); // all angles  are zero

      // contraint bound is relaxed to perform global search

      // if control all gimbals:
      if(control_gimbal_indices_.size() == robot_model_->getRotorNum())
        {
          for(int i = 0; i < control_gimbal_indices_.size(); i++)
            {
              if(i%2 == 0) opt_gimbal_angles_.at(i) = M_PI;
            }

          // hard-coding: singular line form
          if(singular_form && robot_model_->getRotorNum() == 4)
            {
              opt_gimbal_angles_.at(0) = M_PI / 2;
              opt_gimbal_angles_.at(1) = - M_PI / 2;
              opt_gimbal_angles_.at(2) = - M_PI / 2;
              opt_gimbal_angles_.at(3) = M_PI / 2;
            }
        }
    }

  vectoring_nl_solver_->set_lower_bounds(lb);
  vectoring_nl_solver_->set_upper_bounds(ub);

  double start_time = ros::Time::now().toSec();
  double max_f = 0;
  try
    {
      nlopt::result result = vectoring_nl_solver_->optimize(opt_gimbal_angles_, max_f);

      double roll,pitch,yaw;
      robot_model_for_plan_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

      if(prev_opt_gimbal_angles_.size() == 0) prev_opt_gimbal_angles_ = opt_gimbal_angles_;

      if(plan_verbose_)
        {
          std::cout << "nlopt: " << std::setprecision(7)
                    << ros::Time::now().toSec() - start_time  <<  "[sec], cnt: " << cnt;
          std::cout << ", found optimal gimbal angles: ";
          for(auto it: opt_gimbal_angles_) std::cout << std::setprecision(5) << it << " ";
          std::cout << ", max min yaw: " << max_min_yaw_;
          std::cout << ", fc t min: " << robot_model_for_plan_->getFeasibleControlTMin();
          std::cout << ", atttidue: [" << roll << ", " << pitch;
          std::cout << "], force: [" << robot_model_for_plan_->getStaticThrust().transpose();
          std::cout << "]" << std::endl;
        }


      cnt = 0;
      invalid_cnt = 0;
    }
  catch(std::exception &e)
    {
      std::cout << "nlopt failed: " << e.what() << std::endl;
    }

  /* publish the gimbal angles if necessary */
  sensor_msgs::JointState gimbal_msg;
  gimbal_msg.header.stamp = ros::Time::now();

  for(int i = 0; i < control_gimbal_indices_.size(); i++)
    {
      gimbal_msg.name.push_back(control_gimbal_names_.at(i));
      gimbal_msg.position.push_back(opt_gimbal_angles_.at(i));
    }
  gimbal_ctrl_pub_.publish(gimbal_msg);

  prev_opt_gimbal_angles_ = opt_gimbal_angles_;

  return true;
}

void HydrusXiUnderActuatedNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();
  ros::NodeHandle navi_nh(nh_, "navigation");
  getParam<bool>(navi_nh, "plan_verbose", plan_verbose_, false);
  getParam<bool>(navi_nh, "maximize_yaw", maximize_yaw_, false);
  getParam<double>(navi_nh, "gimbal_delta_angle", gimbal_delta_angle_, 0.2);
  getParam<double>(navi_nh, "force_norm_rate", force_norm_weight_, 2.0);
  getParam<double>(navi_nh, "force_variant_rate", force_variant_weight_, 0.01);
  getParam<double>(navi_nh, "yaw_torque_weight", yaw_torque_weight_, 1.0);
  getParam<double>(navi_nh, "fc_t_min_weight", fc_t_min_weight_, 1.0);
  getParam<double>(navi_nh, "baselink_rot_thresh", baselink_rot_thresh_, 0.02);
  getParam<double>(navi_nh, "fc_t_min_thresh", fc_t_min_thresh_, 2.0);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::HydrusXiUnderActuatedNavigator, aerial_robot_navigation::BaseNavigator);

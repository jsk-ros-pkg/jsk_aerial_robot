#include <rolling/control/rolling_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

RollingController::RollingController():
  PoseLinearController(),
  torque_allocation_matrix_inv_pub_stamp_(0)
{
}

void RollingController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                   double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  rolling_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::RollingNavigator>(navigator_);
  rolling_robot_model_ = boost::dynamic_pointer_cast<RollingRobotModel>(robot_model_);
  rolling_robot_model_for_opt_ = boost::make_shared<RollingRobotModel>();
  robot_model_for_control_ = boost::make_shared<aerial_robot_model::RobotModel>();

  rotor_tilt_.resize(motor_num_);
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_, 0);
  prev_target_gimbal_angles_.resize(motor_num_, 0);
  current_gimbal_angles_.resize(motor_num_, 0);

  target_wrench_acc_cog_.resize(6);
  controlled_wrench_acc_cog_.resize(6);

  target_acc_cog_.resize(6);
  target_acc_dash_.resize(6);

  full_lambda_trans_.resize(2 * motor_num_);
  full_lambda_rot_.resize(2 * motor_num_);
  full_lambda_all_.resize(2 * motor_num_);

  full_q_mat_.resize(6, 2 * motor_num_);

  rosParamInit();

  target_roll_ = 0.0;
  target_pitch_ = 0.0;
  target_thrust_z_term_.resize(2 * motor_num_);

  rolling_control_timestamp_ = -1;

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  desire_coordinate_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  target_wrench_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_wrench_acc_cog", 1);
  wrench_allocation_matrix_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix", 1);
  full_q_mat_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/full_q_mat", 1);
  operability_pub_ = nh_.advertise<std_msgs::Float32>("debug/operability", 1);
  target_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_acc_cog", 1);
  target_acc_dash_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_acc_dash", 1);
  exerted_wrench_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/exerted_wrench_cog", 1);

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &RollingController::jointStateCallback, this);

  ground_navigation_mode_ = aerial_robot_navigation::FLYING_STATE;

  control_dof_ = std::accumulate(controlled_axis_.begin(), controlled_axis_.end(), 0);

  controlled_q_mat_.resize(control_dof_, 2 * motor_num_);
  controlled_q_mat_inv_.resize(2 * motor_num_, control_dof_);
  q_mat_.resize(control_dof_, motor_num_);
  q_mat_inv_.resize(motor_num_, control_dof_);

}

void RollingController::reset()
{
  PoseLinearController::reset();

  setControllerParams("controller");
  setControlAxisWithNameSpace("controller");
  setAttitudeGains();

  standing_target_phi_ = 0.0;
  standing_baselink_ref_pitch_last_update_time_ = -1;
  standing_target_baselink_pitch_ = 0.0;
  rolling_control_timestamp_ = -1;

  setControllerParams("standing_controller");

  ROS_INFO_STREAM("[control] reset controller");
}

void RollingController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle base_nh(nh_, "aerial_robot_base_node");
  ros::NodeHandle z_nh(control_nh, "z");

  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
  getParam<bool>(control_nh, "use_sr_inv", use_sr_inv_, false);
  getParam<double>(control_nh, "sr_inv_weight", sr_inv_weight_, 0.0);
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<double>(control_nh, "gimbal_lpf_factor",gimbal_lpf_factor_, 1.0);
  getParam<double>(nh_, "circle_radius", circle_radius_, 0.5);
  rolling_robot_model_->setCircleRadius(circle_radius_);
  rolling_robot_model_for_opt_->setCircleRadius(circle_radius_);

  getParam<double>(z_nh, "i_gain", default_z_i_gain_, 0.0);

  getParam<double>(control_nh, "standing_converged_baselink_roll_thresh", standing_converged_baselink_roll_thresh_, 0.0);
  getParam<double>(control_nh, "standing_converged_z_i_term_min", standing_converged_z_i_term_min_, 0.0);
  getParam<double>(control_nh, "standing_converged_z_i_term_descend_ratio", standing_converged_z_i_term_descend_ratio_, 0.0);
  getParam<double>(control_nh, "standing_baselink_roll_restart_z_i_control_thresh", standing_baselink_roll_restart_z_i_control_thresh_, 0.0);
  getParam<double>(control_nh, "standing_baselink_ref_pitch_update_thresh", standing_baselink_ref_pitch_update_thresh_, 1.0);
  getParam<double>(control_nh, "standing_minimum_z_i_term", standing_minimum_z_i_term_, 0.0);
  getParam<double>(control_nh, "standing_feed_forward_z_compensate_roll_thresh", standing_feed_forward_z_compensate_roll_thresh_, 0.0);

  getParam<double>(control_nh, "steering_z_acc_min", steering_z_acc_min_, 0.0);
  getParam<double>(control_nh, "steering_mu", steering_mu_, 0.0);

  getParam<string>(base_nh, "tf_prefix", tf_prefix_, std::string(""));

  double rotor_tilt1;
  double rotor_tilt2;
  double rotor_tilt3;
  getParam<double>(control_nh, "rotor_tilt1", rotor_tilt1, 0.0);
  rotor_tilt_.at(0) = rotor_tilt1;
  getParam<double>(control_nh, "rotor_tilt2", rotor_tilt2, 0.0);
  rotor_tilt_.at(1) = rotor_tilt2;
  getParam<double>(control_nh, "rotor_tilt3", rotor_tilt3, 0.0);
  rotor_tilt_.at(2) = rotor_tilt3;

  rosoutControlParams("controller");
  rosoutControlParams("ground_controller");
  rosoutControlParams("steering_controller");

  controlled_axis_.resize(6);
  rosoutControlAxis("controller");
  rosoutControlAxis("ground_controller");
  rosoutControlAxis("steering_controller");
}

void RollingController::controlCore()
{
  PoseLinearController::controlCore();

  ground_navigation_mode_ = rolling_navigator_->getCurrentGroundNavigationMode();

  setAttitudeGains();

  control_dof_ = std::accumulate(controlled_axis_.begin(), controlled_axis_.end(), 0);

  /* for stand */
  rolling_robot_model_->calcRobotModelFromFrame("cp");
  standingPlanning();
  calcWrenchAllocationMatrixFromTargetFrame();
  calcStandingFullLambda();
  /* for stand */


  // slsqpSolve();


  // targetStatePlan();
  // if(ground_navigation_mode_ == aerial_robot_navigation::FLYING_STATE || ground_navigation_mode_ == aerial_robot_navigation::STANDING_STATE)
  //   {

  /* for flight */
  // setControllerParams("controller");
  // calcAccFromCog();
  // calcFullLambda();
  /* for flight */


  calcWrenchAllocationMatrix();
  wrenchAllocation();

  // nonlinearQP();

  // std::cout << "wrench allocation" << std::endl;
  //   }
  // else if(ground_navigation_mode_ == aerial_robot_navigation::STEERING_STATE)
  //   {
  // }
  calcYawTerm();

  rolling_navigator_->setPrevGroundNavigationMode(ground_navigation_mode_);
}

void RollingController::targetStatePlan()
{
  switch(ground_navigation_mode_)
    {
    case aerial_robot_navigation::FLYING_STATE:
      {
        if(rolling_navigator_->getPrevGroundNavigationMode() != aerial_robot_navigation::FLYING_STATE)
          {
            ROS_WARN("[control] flying state");
            ROS_ERROR("[control] set control params for flying state");
            setControllerParams("controller");
            rosoutControlParams("controller");
            setControlAxisWithNameSpace("controller");
            setAttitudeGains();
          }
        break;
      }

    case aerial_robot_navigation::STANDING_STATE:
      {
        if(rolling_navigator_->getPrevGroundNavigationMode() != aerial_robot_navigation::STANDING_STATE)
          {
            ROS_WARN("[control] standing state");
            ROS_ERROR("[control] set control params for standing state");
            setControllerParams("ground_controller");
            rosoutControlParams("ground_controller");
            setControlAxisWithNameSpace("ground_controller");
            setAttitudeGains();
          }

        /* set target roll of baselink */
        tf::Vector3 cog_pos = estimator_->getPos(Frame::COG, estimate_mode_);
        if(std::abs(cog_pos.z() / circle_radius_) < 1.0 && std::asin(cog_pos.z() / circle_radius_) > standing_target_phi_)
          {
            standing_target_phi_ = std::asin(cog_pos.z() / circle_radius_);
            if(standing_target_phi_ > 1.57)
              {
                standing_target_phi_ = M_PI / 2.0;
              }
          }

        /* set target z as same as the radius */
        navigator_->setTargetPosZ(circle_radius_);

        /* set z feed forward term */
        if(pid_controllers_.at(Z).getITerm() < standing_minimum_z_i_term_ && standing_target_phi_ < standing_feed_forward_z_compensate_roll_thresh_)
          {
            pid_controllers_.at(Z).setErrIUpdateFlag(true);
            pid_controllers_.at(Z).setITerm(standing_minimum_z_i_term_);
            ROS_WARN_STREAM("[control] set z i term " << standing_minimum_z_i_term_ << " not to vibrate");
          }

        /* set desired coorinate of baselink */
        double baselink_roll = estimator_->getEuler(Frame::BASELINK, estimate_mode_).x();
        double baselink_pitch = estimator_->getEuler(Frame::BASELINK, estimate_mode_).y();

        spinal::DesireCoord desire_coordinate_msg;
        desire_coordinate_msg.roll = standing_target_phi_;
        desire_coordinate_msg.pitch = standing_target_baselink_pitch_;
        if(ros::Time::now().toSec() - standing_baselink_ref_pitch_last_update_time_ > standing_baselink_ref_pitch_update_thresh_)
          {
            // ROS_WARN_STREAM("[control] update target baselink pitch angle to " << baselink_pitch);
            standing_baselink_ref_pitch_last_update_time_ = ros::Time::now().toSec();
            standing_target_baselink_pitch_ = baselink_pitch;
          }
        desire_coordinate_pub_.publish(desire_coordinate_msg);

        /* set target xy of cog based on initial state */
        tf::Vector3 standing_initial_pos = rolling_navigator_->getStandingInitialPos();
        tf::Vector3 standing_initial_euler = rolling_navigator_->getStandingInitialEuler();
        navigator_->setTargetPosX(standing_initial_pos.x() - circle_radius_ * (1 - cos(standing_target_phi_)) * cos(standing_initial_euler.z() + M_PI / 2.0));
        navigator_->setTargetPosY(standing_initial_pos.y() - circle_radius_ * (1 - cos(standing_target_phi_)) * sin(standing_initial_euler.z() + M_PI / 2.0));

        /*  decrease throttle if roll angle is converged */
        if(fabs(baselink_roll - M_PI / 2.0) < standing_converged_baselink_roll_thresh_)
          {
            ROS_WARN_STREAM("[control] baselink roll is converged " << baselink_roll << " thresh is " << standing_converged_baselink_roll_thresh_);
            double z_i_term = pid_controllers_.at(Z).getITerm();
            pid_controllers_.at(Z).setErrIUpdateFlag(true);
            double target_z_i_term = std::max(z_i_term -  standing_converged_z_i_term_descend_ratio_, standing_converged_z_i_term_min_);
            pid_controllers_.at(Z).setITerm(target_z_i_term);
            pid_controllers_.at(Z).setIGainHoldingTerm(default_z_i_gain_);
            pid_controllers_.at(Z).setErrIUpdateFlag(false);
            ROS_WARN_STREAM("[control] set z i term to " << pid_controllers_.at(Z).getITerm());
          }

        /* restart i control and set feed forward i term again */
        else
          {
            if(!pid_controllers_.at(Z).getErrIUpdateFlag())
              {
                pid_controllers_.at(Z).setErrIUpdateFlag(true);
                ROS_WARN_STREAM("[control] restart z i control because roll error is larger than " << standing_baselink_roll_restart_z_i_control_thresh_);
                pid_controllers_.at(Z).setITerm(standing_minimum_z_i_term_);
                pid_controllers_.at(Z).setIGainHoldingTerm(3.0);
                ROS_WARN_STREAM("[control] set z i term " << standing_minimum_z_i_term_ << " not to fall down");
              }
          }
        break;
      }

    case aerial_robot_navigation::STEERING_STATE:
      {
        if(rolling_navigator_->getPrevGroundNavigationMode() != aerial_robot_navigation::STEERING_STATE)
          {
            ROS_ERROR("[control] set control params for steering mode");

            // set parameter and controlled axis
            setControllerParams("steering_controller");
            rosoutControlParams("steering_controller");
            setControlAxisWithNameSpace("steering_controller");
            setAttitudeGains();
            ROS_ERROR("[control] set control target for steering mode");

            // set target position from current position
            tf::Vector3 initial_cog_pos = estimator_->getPos(Frame::COG, estimate_mode_);
            tf::Vector3 initial_baselink_euler = estimator_->getEuler(Frame::BASELINK, estimate_mode_);
            tf::Vector3 initial_cog_euler = estimator_->getEuler(Frame::COG, estimate_mode_);
            std::cout << "initial cog pos " << initial_cog_pos.x() << " " << initial_cog_pos.y() << " " << initial_cog_pos.z() << std::endl;
            std::cout << "initial baselink euler " << initial_baselink_euler.x() << " " << initial_baselink_euler.y() << " " << initial_baselink_euler.z() << std::endl;
            std::cout << "initial cog euler " << initial_cog_euler.x() << " " << initial_cog_euler.y() << " " << initial_cog_euler.z() << std::endl;
            navigator_->setTargetPosX(initial_cog_pos.x());
            navigator_->setTargetPosY(initial_cog_pos.y());
            pid_controllers_.at(YAW).reset();
            navigator_->setTargetYaw(initial_cog_euler.z());
            ROS_WARN_STREAM("[steering] set target yaw in navigator to " << initial_cog_euler.z());

            // set target baselink pose roll: pi/2, pitch: current statex
            spinal::DesireCoord desire_coordinate_msg;
            desire_coordinate_msg.roll = M_PI / 2.0;
            desire_coordinate_msg.pitch = initial_baselink_euler.y();
            desire_coordinate_pub_.publish(desire_coordinate_msg);
            ROS_WARN_STREAM("[steering] send desire coordinate: " << desire_coordinate_msg.roll << " " << desire_coordinate_msg.pitch);
            // std::cout << "[steering] baselink pitch: " << initial_baselink_euler.y() << std::endl;
            // pid_controllers_.at(YAW).setTargetP(initial_cog_euler.z());
          }

        // transition to recovery state when roll angle is large
        double baselink_roll = estimator_->getEuler(Frame::BASELINK, estimate_mode_).x();
        if(fabs(baselink_roll - M_PI / 2.0) > 0.3)
          {
            rolling_navigator_->setGroundNavigationMode(aerial_robot_navigation::RECOVERING_STATE);
          }

        break;
      }

    case aerial_robot_navigation::RECOVERING_STATE:
      {
        if(rolling_navigator_->getPrevGroundNavigationMode() != aerial_robot_navigation::RECOVERING_STATE)
          {
            setControllerParams("ground_controller");
            rosoutControlParams("ground_controller");
            setControlAxisWithNameSpace("ground_controller");
            setAttitudeGains();
            ROS_ERROR_STREAM("[control] changed to recovery state");
            tf::Vector3 initial_cog_pos = estimator_->getPos(Frame::COG, estimate_mode_);
            tf::Vector3 initial_cog_euler = estimator_->getEuler(Frame::COG, estimate_mode_);
            tf::Vector3 initial_baselink_euler = estimator_->getEuler(Frame::COG, estimate_mode_);
            navigator_->setTargetPosX(initial_cog_pos.x());
            navigator_->setTargetPosY(initial_cog_pos.y());
            navigator_->setTargetYaw(initial_cog_euler.z());
          }

        pid_controllers_.at(Z).setErrIUpdateFlag(true);
        pid_controllers_.at(Z).setITerm(standing_minimum_z_i_term_);

        double baselink_roll = estimator_->getEuler(Frame::BASELINK, estimate_mode_).x();
        if(fabs(baselink_roll - M_PI / 2.0)  < fabs(standing_converged_baselink_roll_thresh_))
          {
            rolling_navigator_->setGroundNavigationMode(aerial_robot_navigation::STANDING_STATE);
          }
      }

    }
}

void RollingController::calcAccFromCog()
{
      ROS_WARN_ONCE("[control] calc acc for flying state");
      control_dof_ = std::accumulate(controlled_axis_.begin(), controlled_axis_.end(), 0);

      tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
      tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                               pid_controllers_.at(Y).result(),
                               pid_controllers_.at(Z).result());
      tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
      tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

      target_acc_cog_.at(0) = target_acc_cog.x();
      target_acc_cog_.at(1) = target_acc_cog.y();
      target_acc_cog_.at(2) = target_acc_cog.z();

      target_acc_dash_.at(0) = target_acc_dash.x();
      target_acc_dash_.at(1) = target_acc_dash.y();
      target_acc_dash_.at(2) = target_acc_dash.z();

      Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
      if(controlled_axis_.at(0) == 0 || controlled_axis_.at(1) == 0)
        {
          target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());
        }
      else
        {
          target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
        }

      double target_ang_acc_x = pid_controllers_.at(ROLL).result();
      double target_ang_acc_y = pid_controllers_.at(PITCH).result();
      double target_ang_acc_z = pid_controllers_.at(YAW).result();

      target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
      target_wrench_acc_cog_ = target_wrench_acc_cog;

      pid_msg_.roll.total.at(0) = target_ang_acc_x;
      pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
      pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
      pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
      pid_msg_.roll.target_p = target_rpy_.x();
      pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
      pid_msg_.roll.target_d = target_omega_.x();
      pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
      pid_msg_.pitch.total.at(0) = target_ang_acc_y;
      pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
      pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
      pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
      pid_msg_.pitch.target_p = target_rpy_.y();
      pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
      pid_msg_.pitch.target_d = target_omega_.y();
      pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

      if(navigator_->getForceLandingFlag() && target_acc_w.z() < 5.0) // heuristic measures to avoid to large gimbal angles after force land
        start_rp_integration_ = false;

      if(!controlled_axis_.at(X))
        {
          if(hovering_approximate_)
            {
              target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
            }
          else
            {
              target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
            }
          navigator_->setTargetPitch(target_pitch_);
        }
      if(!controlled_axis_.at(Y))
        {
          if(hovering_approximate_)
            {
              target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
            }
          else
            {
              target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
            }
          navigator_->setTargetRoll(target_roll_);
        }

}

void RollingController::calcWrenchAllocationMatrix()
{
  /* calculate normal allocation */
  Eigen::MatrixXd wrench_matrix = Eigen::MatrixXd::Zero(6, 3 * motor_num_);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) =  Eigen::MatrixXd::Identity(3, 3);

  int last_col = 0;
  std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();
  for(int i = 0; i < motor_num_; i++)
    {
      wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));
      wrench_matrix.middleCols(last_col, 3) = wrench_map;
      last_col += 3;
    }

  Eigen::Matrix3d inertia_inv = robot_model_for_control_->getInertia<Eigen::Matrix3d>().inverse();
  double mass_inv = 1 / robot_model_->getMass();
  wrench_matrix.topRows(3) = mass_inv * wrench_matrix.topRows(3);
  wrench_matrix.bottomRows(3) = inertia_inv * wrench_matrix.bottomRows(3);

  /* calculate masked and integrated rotaion matrix */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, 2 * motor_num_);
  const auto links_rotation_from_cog = rolling_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
  Eigen::MatrixXd mask(3, 2);
  mask << 0, 0, 1, 0, 0, 1;
  for(int i = 0; i < motor_num_; i++)
    {
      integrated_rot.block(3 * i, 2 * i, 3, 2) = links_rotation_from_cog.at(i) * mask;
    }

  /* calculate integarated allocation */
  full_q_mat_ = wrench_matrix * integrated_rot;
  full_q_trans_ = full_q_mat_.topRows(3);
  full_q_rot_ = full_q_mat_.bottomRows(3);

  /* extract controlled axis */
  Eigen::MatrixXd controlled_axis_mask = Eigen::MatrixXd::Zero(control_dof_, 6);
  int last_row = 0;
  for(int i = 0; i < controlled_axis_.size(); i++)
    {
      if(controlled_axis_.at(i))
        {
          controlled_axis_mask(last_row, i) = 1;
          last_row++;
        }
    }

  controlled_q_mat_ = controlled_axis_mask * full_q_mat_;
  controlled_q_mat_inv_ = aerial_robot_model::pseudoinverse(controlled_q_mat_);
  controlled_wrench_acc_cog_ = controlled_axis_mask * target_wrench_acc_cog_;

  if(use_sr_inv_)
    {
      // http://www.thothchildren.com/chapter/5bd8d78751d930518903af34
      Eigen::MatrixXd sr_inv = controlled_q_mat_.transpose() * (controlled_q_mat_ * controlled_q_mat_.transpose() + sr_inv_weight_ * Eigen::MatrixXd::Identity(controlled_q_mat_.rows(), controlled_q_mat_.rows())).inverse();
      controlled_q_mat_inv_ = sr_inv;
      ROS_WARN_STREAM_ONCE("[control] use SR-Inverse. weight is " << sr_inv_weight_);
    }
  else
    {
      ROS_WARN_ONCE("[control] use MP-Inverse");
    }
}

void RollingController::calcFullLambda()
{
  if(ground_navigation_mode_ == aerial_robot_navigation::FLYING_STATE || ground_navigation_mode_ == aerial_robot_navigation::STANDING_STATE || ground_navigation_mode_ == aerial_robot_navigation::STEERING_STATE || aerial_robot_navigation::RECOVERING_STATE)
    {
      /* actuator mapping */
      int rot_dof = 0;
      for(int i = 3; i < 6; i++)
        {
          if(controlled_axis_.at(i))
            {
              rot_dof++;
            }
        }
      full_lambda_trans_ = controlled_q_mat_inv_.leftCols(control_dof_ - rot_dof) * controlled_wrench_acc_cog_.head(control_dof_ - rot_dof);
      full_lambda_rot_ = controlled_q_mat_inv_.rightCols(rot_dof) * controlled_wrench_acc_cog_.tail(rot_dof);
      full_lambda_all_ = full_lambda_trans_ + full_lambda_rot_;
    }
}

void RollingController::wrenchAllocation()
{
  int last_col = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      Eigen::VectorXd full_lambda_trans_i = full_lambda_trans_.segment(last_col, 2);
      Eigen::VectorXd full_lambda_all_i = full_lambda_all_.segment(last_col, 2);

      /* calculate base thrusts */
      target_base_thrust_.at(i) = full_lambda_trans_i.norm() / fabs(cos(rotor_tilt_.at(i)));

      /* calculate gimbal angles */
      double gimbal_angle_i = atan2(-full_lambda_all_i(0), full_lambda_all_i(1));
      target_gimbal_angles_.at(i) = (gimbal_lpf_factor_ - 1.0) / gimbal_lpf_factor_ * prev_target_gimbal_angles_.at(i) + 1.0 / gimbal_lpf_factor_ * gimbal_angle_i;
      ROS_WARN_STREAM_ONCE("[control] gimbal lpf factor: " << gimbal_lpf_factor_);

      last_col += 2;
    }

  /* update robot model by calculated gimbal angle */
  const auto& joint_index_map = robot_model_->getJointIndexMap();
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray gimbal_processed_joint = robot_model_->getJointPositions();
  for(int i = 0; i < motor_num_; i++)
    {
      std::string s = std::to_string(i + 1);
      gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s)->second) = target_gimbal_angles_.at(i);
    }
  robot_model_for_control_->updateRobotModel(gimbal_processed_joint);

  /* calculate allocation matrix for realtime control */
  q_mat_ = robot_model_for_control_->calcWrenchMatrixOnCoG();
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);
}

void RollingController::calcYawTerm()
{
  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
  for (unsigned int i = 0; i < motor_num_; i++)
    {
      if(q_mat_inv_(i, control_dof_ - 1) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, control_dof_ - 1);
    }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;

  if(controlled_axis_.at(YAW) == 0)
    {
      candidate_yaw_term_ = 0;
    }
}

void RollingController::sendCmd()
{
  PoseLinearController::sendCmd();

  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < full_lambda_all_.size(); i++)
    {
      target_vectoring_force_msg.data.push_back(full_lambda_all_(i));
    }
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);

  std_msgs::Float32MultiArray target_wrench_acc_cog_msg;
  for(int i = 0; i < target_wrench_acc_cog_.size(); i++)
    {
      target_wrench_acc_cog_msg.data.push_back(target_wrench_acc_cog_(i));
    }
  target_wrench_acc_cog_pub_.publish(target_wrench_acc_cog_msg);

  aerial_robot_msgs::WrenchAllocationMatrix wrench_allocation_matrix_msg;
  for(int i = 0; i < q_mat_.cols(); i++)
    {
      wrench_allocation_matrix_msg.f_x.push_back(q_mat_(0, i));
      wrench_allocation_matrix_msg.f_y.push_back(q_mat_(1, i));
      wrench_allocation_matrix_msg.f_z.push_back(q_mat_(2, i));
      wrench_allocation_matrix_msg.t_x.push_back(q_mat_(3, i));
      wrench_allocation_matrix_msg.t_y.push_back(q_mat_(4, i));
      wrench_allocation_matrix_msg.t_z.push_back(q_mat_(5, i));
    }
  wrench_allocation_matrix_pub_.publish(wrench_allocation_matrix_msg);

  std_msgs::Float32MultiArray target_acc_cog_msg;
  for(int i = 0; i < target_acc_cog_.size(); i++)
    {
      target_acc_cog_msg.data.push_back(target_acc_cog_.at(i));
    }
  target_acc_cog_pub_.publish(target_acc_cog_msg);

  std_msgs::Float32MultiArray target_acc_dash_msg;
  for(int i = 0; i < target_acc_dash_.size(); i++)
    {
      target_acc_dash_msg.data.push_back(target_acc_dash_.at(i));
    }
  target_acc_dash_pub_.publish(target_acc_dash_msg);

  aerial_robot_msgs::WrenchAllocationMatrix full_q_mat_msg;
  for(int i = 0; i < 2 * motor_num_; i++)
    {
      full_q_mat_msg.f_x.push_back(full_q_mat_(0, i));
      full_q_mat_msg.f_y.push_back(full_q_mat_(1, i));
      full_q_mat_msg.f_z.push_back(full_q_mat_(2, i));
      full_q_mat_msg.t_x.push_back(full_q_mat_(3, i));
      full_q_mat_msg.t_y.push_back(full_q_mat_(4, i));
      full_q_mat_msg.t_z.push_back(full_q_mat_(5, i));
    }
  full_q_mat_pub_.publish(full_q_mat_msg);

  std_msgs::Float32MultiArray exerted_wrench_msg;
  Eigen::VectorXd exerted_wrench = full_q_mat_ * full_lambda_all_;
  for(int i = 0; i < exerted_wrench.size(); i++)
    {
      exerted_wrench_msg.data.push_back(exerted_wrench(i));
    }
  exerted_wrench_pub_.publish(exerted_wrench_msg);

  std_msgs::Float32 operability_msg;
  Eigen::MatrixXd q_qt;
  q_qt = controlled_q_mat_ * controlled_q_mat_.transpose();
  float det = q_qt.determinant();
  operability_msg.data =sqrt(det);
  operability_pub_.publish(operability_msg);

  sendGimbalAngles();

  sendFourAxisCommand();

  sendTorqueAllocationMatrixInv();
}

void RollingController::sendGimbalAngles()
{
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  for(int i = 0; i < motor_num_; i++){
    gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
    gimbal_control_msg.name.push_back(std::string("gimbal") + std::to_string(i + 1));
  }
  gimbal_control_pub_.publish(gimbal_control_msg);
}

void RollingController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  if(!controlled_axis_[0])
    {
      flight_command_data.angles[1] = target_pitch_;
    }
  if(!controlled_axis_[1])
    {
      flight_command_data.angles[0] = target_roll_;
    }
  flight_cmd_pub_.publish(flight_command_data);
}

void RollingController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
      Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
      if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
        ROS_ERROR("Torque Allocation Matrix overflow");
      for (unsigned int i = 0; i < motor_num_; i++)
        {
          torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
        }
      torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
    }
}

void RollingController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  rpy_gain_pub_.publish(rpy_gain_msg);
}

void RollingController::jointStateCallback(const sensor_msgs::JointStateConstPtr & state)
{
  sensor_msgs::JointState joint_state = *state;

  /* get current gimbal angles */
  for(int i = 0; i < joint_state.name.size(); i++)
    {
      if(joint_state.name.at(i).find("gimbal") != string::npos)
        {
          for(int j = 0; j < motor_num_; j++)
            {
              if(joint_state.name.at(i) == std::string("gimbal") + std::to_string(j + 1))
                {
                  std::lock_guard<std::mutex> lock(current_gimbal_angles_mutex_);
                  current_gimbal_angles_.at(j) = joint_state.position.at(i);
                }
            }
        }
    }

  /* tf of contact point */
  geometry_msgs::TransformStamped contact_point_tf = rolling_robot_model_->getContactPoint<geometry_msgs::TransformStamped>();
  contact_point_tf.header = state->header;
  contact_point_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  contact_point_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("contact_point"));
  br_.sendTransform(contact_point_tf);

  /* tf of center point */
  geometry_msgs::TransformStamped center_point_tf = rolling_robot_model_->getCenterPoint<geometry_msgs::TransformStamped>();
  center_point_tf.header = state->header;
  center_point_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  center_point_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("center_point"));
  br_.sendTransform(center_point_tf);

  /* tf of contact point alined to ground plane */
  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point = rolling_robot_model_->getContactPoint<KDL::Frame>();
  double baselink_roll = estimator_->getEuler(Frame::COG, estimate_mode_).x();
  double baselink_pitch = estimator_->getEuler(Frame::COG, estimate_mode_).y();

  if(true)
    {
      std::lock_guard<std::mutex> lock(contact_point_alined_mutex_);
      contact_point_alined_.p = contact_point.p;
      contact_point_alined_.M = cog.M;
      contact_point_alined_.M.DoRotX(-baselink_roll);
      contact_point_alined_.M.DoRotY(-baselink_pitch);
    }
  geometry_msgs::TransformStamped contact_point_alined_tf = kdlToMsg(contact_point_alined_);
  contact_point_alined_tf.header = state->header;
  contact_point_alined_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  contact_point_alined_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("contact_point_alined"));
  br_.sendTransform(contact_point_alined_tf);

}



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::RollingController, aerial_robot_control::ControlBase);

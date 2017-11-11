// -*- mode: c++ -*-

#include <hydrus_xi/full_over_actuated_controller.h>

namespace control_plugin
{
  FullOverActuatedController::FullOverActuatedController():
    FlatnessPid(), msg_pub_cnt_(0)
  {
    need_yaw_d_control_ = false;
  }

  void FullOverActuatedController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, Navigator* navigator, double ctrl_loop_rate) //override
  {
    FlatnessPid::initialize(nh, nhp, estimator, navigator, ctrl_loop_rate);

    transform_controller_ = std::shared_ptr<TransformController>(new TransformController(nh_, nhp_, true));
    transform_controller_->setRealtimeControlFlag(false); //do not solve LQI
    motor_num_ = transform_controller_->getRotorNum();

    nhp_.param("msg_pub_prescaler", msg_pub_prescaler_, 2);
    Q_pseudo_inv_ = Eigen::MatrixXd(motor_num_, 8);

    nhp_.param("q_matrix_pseudo_inverse_inertia_pub_topic_name", q_matrix_pseudo_inverse_inertia_pub_topic_name_, std::string("/q_matrix_pseudo_inverse_inertia"));
    q_matrix_pseudo_inverse_inertia_pub_ = nh_.advertise<hydrus_xi::QMatrixPseudoInverseInertia>(q_matrix_pseudo_inverse_inertia_pub_topic_name_, 1);
  }

  void FullOverActuatedController::pidUpdate() //override
  {
    aerial_robot_base::FlatnessPid pid_msg;
    pid_msg.header.stamp = ros::Time::now();

    /* roll/pitch integration flag */
    if(!start_rp_integration_)
      {
        if(state_pos_.z() > 0.01)
          {
            start_rp_integration_ = true;
            std_msgs::UInt8 integration_cmd;
            integration_cmd.data = navigator_->ROS_INTEGRATE_CMD;
            navigator_->flight_config_pub_.publish(integration_cmd);
            ROS_WARN("start rp integration");
          }
      }

    /* time diff */
    double du = ros::Time::now().toSec() - control_timestamp_;

    /* xy */
    tf::Vector3 xy_p_term, xy_d_term;
    tf::Matrix3x3 uav_rot;
    uav_rot.setRPY(estimator_->getState(State::ROLL_COG, estimate_mode_)[0],
                   estimator_->getState(State::PITCH_COG, estimate_mode_)[0],
                   estimator_->getState(State::YAW_COG, estimate_mode_)[0]);
    tf::Matrix3x3 uav_rot_inv = uav_rot.inverse();

    /* convert from world frame to CoG frame */
    pos_err_ = uav_rot_inv * (target_pos_ - state_pos_);

    switch(navigator_->getXyControlMode())
      {
      case flight_nav::POS_CONTROL_MODE:
        {
          /* P */
          xy_p_term = clampV(xy_gains_[0] * pos_err_,  -xy_terms_limits_[0], xy_terms_limits_[0]);

          /* I */
          if(navigator_->getNaviState() == Navigator::TAKEOFF_STATE || navigator_->getNaviState() == Navigator::LAND_STATE) //takeoff or land
            xy_i_term_ += (pos_err_ * du * xy_gains_[1]);
          else
            xy_i_term_ += (pos_err_ * du * xy_hovering_i_gain_);
          xy_i_term_ = clampV(xy_i_term_, -xy_terms_limits_[1], xy_terms_limits_[1]);

          /* D */
          vel_err_ = uav_rot_inv * (-state_vel_);
          xy_d_term = clampV(xy_gains_[2] * vel_err_,  -xy_terms_limits_[2], xy_terms_limits_[2]);
          break;
        }
      case flight_nav::VEL_CONTROL_MODE:
        {
          /* convert from world frame to CoG frame */
          vel_err_ = uav_rot_inv * (target_vel_ - state_vel_);

          /* P */
          xy_p_term = clampV(xy_gains_[2] * vel_err_,  -xy_terms_limits_[0], xy_terms_limits_[0]);
          xy_d_term.setValue(0, 0, 0);
          break;
        }
      case flight_nav::ACC_CONTROL_MODE:
        {
          /* convert from world frame to CoG frame */

          xy_p_term = uav_rot_inv * (target_acc_ / BasicEstimator::G);
          xy_i_term_.setValue(0, 0, 0);
          xy_d_term.setValue(0, 0, 0);
          break;
        }
      default:
        {
          break;
        }
      }

    tf::Vector3 xy_total_term = xy_p_term + xy_i_term_ + xy_d_term + xy_offset_;
    target_linear_acc_[0] = clamp(xy_total_term[0], -xy_limit_, xy_limit_);
    target_linear_acc_[1] = clamp(xy_total_term[1], -xy_limit_, xy_limit_);

    /* ros pub */
    pid_msg.pitch.total.push_back(target_linear_acc_[0]);
    pid_msg.roll.total.push_back(target_linear_acc_[1]);
    pid_msg.pitch.p_term.push_back(xy_p_term[0]);
    pid_msg.pitch.i_term.push_back(xy_i_term_[0]);
    pid_msg.pitch.d_term.push_back(xy_d_term[0]);
    pid_msg.roll.p_term.push_back(xy_p_term[1]);
    pid_msg.roll.i_term.push_back(xy_i_term_[1]);
    pid_msg.roll.d_term.push_back(xy_d_term[1]);
    pid_msg.pitch.pos_err = pos_err_[0];
    pid_msg.pitch.target_pos = target_pos_[0];
    pid_msg.roll.pos_err = pos_err_[1];
    pid_msg.roll.target_pos = target_pos_[1];
    pid_msg.pitch.target_vel = target_vel_[0];
    pid_msg.roll.target_vel = target_vel_[1];

    /* yaw */
    for(int j = 0; j < motor_num_; j++)
      {
        //**** P term
        double yaw_p_term = clamp(-yaw_gains_[j][0] * psi_err_, -yaw_terms_limits_[0], yaw_terms_limits_[0]);

        //**** I term:
        yaw_i_term_[j] += (psi_err_ * du * yaw_gains_[j][1]);
        yaw_i_term_[j] = clamp(yaw_i_term_[j], -yaw_terms_limits_[1], yaw_terms_limits_[1]);

        //***** D term: usaully it is in the flight board
        /* but for the gimbal control, we need the d term, set 0 if it is not gimbal type */
        double yaw_d_term = 0;
        if(need_yaw_d_control_)
          yaw_d_term = clamp(yaw_gains_[j][2] * (-state_psi_vel_), -yaw_terms_limits_[2], yaw_terms_limits_[2]);

        //*** each motor command value for log
        target_yaw_[j] = clamp(yaw_p_term + yaw_i_term_[j] + yaw_d_term, -yaw_limit_, yaw_limit_);

        pid_msg.yaw.total.push_back(target_yaw_[j]);
        pid_msg.yaw.p_term.push_back(yaw_p_term);
        pid_msg.yaw.i_term.push_back(yaw_i_term_[j]);
        pid_msg.yaw.d_term.push_back(yaw_d_term);

        if(yaw_gains_.size() == 1) break;
      }

    //**** ros pub
    pid_msg.yaw.target_pos = target_psi_;
    pid_msg.yaw.pos_err = psi_err_;

    /* throttle */
    /* convert from world frame to CoG frame */
    double alt_err = clamp(pos_err_.z(), -alt_err_thresh_, alt_err_thresh_);

    if(navigator_->getNaviState() == Navigator::LAND_STATE) alt_err += alt_landing_const_i_ctrl_thresh_;

    for(int j = 0; j < motor_num_; j++)
      {
        //**** P Term
        double alt_p_term = clamp(-alt_gains_[j][0] * alt_err, -alt_terms_limit_[0], alt_terms_limit_[0]);
        if(navigator_->getNaviState() == Navigator::LAND_STATE) alt_p_term = 0;

        /* two way to calculate the alt i term */
        //**** I Term
        alt_i_term_[j] +=  alt_err * du;
        double alt_i_term = clamp(alt_gains_[j][1] * alt_i_term_[j], -alt_terms_limit_[1], alt_terms_limit_[1]);
        //***** D Term
        double alt_d_term = clamp(alt_gains_[j][2] * state_vel_.z(), -alt_terms_limit_[2], alt_terms_limit_[2]);

        //*** each motor command value for log
        target_throttle_[j] = clamp(alt_p_term + alt_i_term + alt_d_term + alt_offset_, -alt_limit_, alt_limit_);
        pid_msg.throttle.total.push_back(target_throttle_[j]);
        pid_msg.throttle.p_term.push_back(alt_p_term);
        pid_msg.throttle.i_term.push_back(alt_i_term);
        pid_msg.throttle.d_term.push_back(alt_d_term);

        if(alt_gains_.size() == 1) break;
      }

    pid_msg.throttle.target_pos = target_pos_.z();
    pid_msg.throttle.pos_err = alt_err;
    pid_msg.throttle.vel_err = state_vel_.z();

    /* ros publish */
    pid_pub_.publish(pid_msg);

    /* update */
    control_timestamp_ = ros::Time::now().toSec();
  }

  void FullOverActuatedController::sendCmd() //override
  {
    /* send flight command */
    aerial_robot_msgs::FourAxisCommand flight_command_data;
    //TODO : input by planner
    flight_command_data.angles[0] = -target_roll_;
    flight_command_data.angles[1] = target_pitch_;
    flight_command_data.angles[2] = target_yaw_[0];

    flight_command_data.base_throttle.resize(motor_num_);
    /* Simple PID based position/attitude/altitude control */
    calcQPseudoInv();
    calcForceVector(flight_command_data.base_throttle);

    flight_cmd_pub_.publish(flight_command_data);

    /* send Inertia and Q_pseudo_inv_torque */
    msg_pub_prescaler_++;
    if (msg_pub_cnt_ != msg_pub_prescaler_)
      {
        hydrus_xi::QMatrixPseudoInverseInertia q_matrix_pseudo_inverse_inertia_msg;
        Eigen::Matrix3d uav_inertia = transform_controller_->getInertia();
        q_matrix_pseudo_inverse_inertia_msg.inertia[0] = uav_inertia(0, 0) * 1000;
        q_matrix_pseudo_inverse_inertia_msg.inertia[1] = uav_inertia(1, 1) * 1000;
        q_matrix_pseudo_inverse_inertia_msg.inertia[2] = uav_inertia(2, 2) * 1000;
        q_matrix_pseudo_inverse_inertia_msg.inertia[3] = uav_inertia(0, 1) * 1000;
        q_matrix_pseudo_inverse_inertia_msg.inertia[4] = uav_inertia(1, 2) * 1000;
        q_matrix_pseudo_inverse_inertia_msg.inertia[5] = uav_inertia(0, 2) * 1000;

        q_matrix_pseudo_inverse_inertia_msg.Q_matrix_pseudo_inverse.resize(3 * motor_num_);
        Eigen::MatrixXd Q_pseudo_inv_torque = Q_pseudo_inv_.block(3, 0, motor_num_, 3);
        for (unsigned int i = 0; i < motor_num_; i++)
          {
            for (unsigned int j = 0; j < 3; j++)
              {
                q_matrix_pseudo_inverse_inertia_msg.Q_matrix_pseudo_inverse.at(i * 3 + j) = Q_pseudo_inv_torque(i, j) * 1000;
              }
          }
        q_matrix_pseudo_inverse_inertia_pub_.publish(q_matrix_pseudo_inverse_inertia_msg);
      }
    else
      {
        msg_pub_prescaler_ = 0;
      }
  }

  void FullOverActuatedController::calcQPseudoInv()
  {
    std::vector<Eigen::Vector3d> rotors_origin;
    std::vector<Eigen::Vector3d> rotors_normal;
    transform_controller_->getRotorsOriginFromCog(rotors_origin);
    transform_controller_->getRotorsNormalFromCog(rotors_normal);
    if (rotors_origin.size() != motor_num_) {
      ROS_ERROR("full over actuated controller: motor num is incorrect");
      return;
    }

    Eigen::MatrixXd Q(6, motor_num_);
    for (unsigned int i = 0; i < motor_num_; i++) {
      Q.block(0, i, 3, 1) = rotors_normal.at(i);
      Q.block(3, i, 3, 1) = rotors_origin.at(i).cross(rotors_normal.at(i));
    }
    Q_pseudo_inv_ = pseudoinverse(Q);
  }

  void FullOverActuatedController::calcForceVector(std::vector<float>& force)
  {
    double uav_mass = transform_controller_->getMass();
    Eigen::Vector3d target_force = Eigen::Vector3d(target_linear_acc_.getX(), target_linear_acc_.getY(), target_throttle_[0]) * uav_mass;
    Eigen::MatrixXd Q_pseudo_inv_force = Q_pseudo_inv_.block(0, 0, motor_num_, 3);
    Eigen::VectorXd propeller_force = Q_pseudo_inv_force * target_force;
    for (unsigned int i = 0; i < force.size(); i++)
      force.at(i) = propeller_force[i];
  }

} //namespace control_plugin

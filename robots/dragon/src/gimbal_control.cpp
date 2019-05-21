#include <dragon/gimbal_control.h>

using namespace aerial_robot_model;

namespace control_plugin
{
  DragonGimbal::DragonGimbal():
    FlatnessPid(),
    servo_torque_(false), level_flag_(false), landing_flag_(false),
    curr_target_cog_rot_(0, 0, 0),
    final_target_cog_rot_(0, 0, 0),
    roll_i_term_(0), pitch_i_term_(0), gimbal_control_stamp_(0)
  {
    need_yaw_d_control_ = true;
  }

  void DragonGimbal::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                StateEstimator* estimator, Navigator* navigator,
                                double ctrl_loop_rate)
  {
    /* initialize the flight control */
    FlatnessPid::initialize(nh, nhp, estimator, navigator, ctrl_loop_rate);

    /* initialize the multilink kinematics */
    kinematics_ = std::make_unique<DragonRobotModel>(true);

    /* initialize the matrix */
    P_xy_ = Eigen::MatrixXd::Zero(2, kinematics_->getRotorNum() * 2);
    for(int i = 0; i < kinematics_->getRotorNum(); i++)
      {
        P_xy_(0, i * 2) = 1;
        P_xy_(1, 1 + i * 2) = 1;
      }

    /* initialize the gimbal target angles */
    target_gimbal_angles_.resize(kinematics_->getRotorNum() * 2, 0);

    gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("/gimbals_ctrl", 1);
    joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>("/joints_ctrl", 1);
    curr_target_cog_rot_pub_ = nh_.advertise<spinal::DesireCoord>("/desire_coordinate", 1);
    gimbal_target_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("gimbals_target_force", 1);
    roll_pitch_pid_pub_ = nh_.advertise<aerial_robot_msgs::FlatnessPid>("roll_pitch_gimbal_control", 1);

    joint_state_sub_ = nh_.subscribe("/joint_states", 1, &DragonGimbal::jointStateCallback, this);
    final_target_cog_rot_sub_ = nh_.subscribe("/final_desire_tilt", 1, &DragonGimbal::baselinkTiltCallback, this);
    target_coord_sub_ = nh_.subscribe("/desire_coordinate", 1, &DragonGimbal::targetCogRotCallback, this);

    //dynamic reconfigure server
    roll_pitch_pid_server_ = new dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>(ros::NodeHandle(nhp_, "pitch_roll"));
    dynamic_reconf_func_roll_pitch_pid_ = boost::bind(&DragonGimbal::cfgPitchRollPidCallback, this, _1, _2);
    roll_pitch_pid_server_->setCallback(dynamic_reconf_func_roll_pitch_pid_);

    yaw_pid_server_ = new dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>(ros::NodeHandle(nhp_, "yaw"));
    dynamic_reconf_func_yaw_pid_ = boost::bind(&DragonGimbal::cfgYawPidCallback, this, _1, _2);
    yaw_pid_server_->setCallback(dynamic_reconf_func_yaw_pid_);
  }

  void DragonGimbal::fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg)
  {
    /* update the motor number */
    if(motor_num_ == 0)
      {
        motor_num_ = msg->motor_num;

        alt_i_term_.resize(motor_num_);
        alt_gains_.resize(motor_num_);
        roll_gains_.resize(motor_num_);
        pitch_gains_.resize(motor_num_);
        att_lqi_term_.resize(motor_num_);
        target_throttle_.resize(motor_num_); // LQI(z)
        target_thrust_vector_.resize(motor_num_); // target thrust (throttle): ||f||

        ROS_WARN("[gimbal flight control] update the motor number: %d", motor_num_);
      }

    for(int i = 0; i < msg->motor_num; i++)
      {
        alt_gains_[i].setValue(msg->pos_p_gain_alt[i], msg->pos_i_gain_alt[i], msg->pos_d_gain_alt[i]);
        pitch_gains_[i].setValue(msg->pos_p_gain_pitch[i], msg->pos_i_gain_pitch[i], msg->pos_d_gain_pitch[i]);
        roll_gains_[i].setValue(msg->pos_p_gain_roll[i], msg->pos_i_gain_roll[i], msg->pos_d_gain_roll[i]);
      }
  }

  void DragonGimbal::baselinkTiltCallback(const spinal::DesireCoordConstPtr & msg)
  {
    final_target_cog_rot_.setValue(msg->roll, msg->pitch, msg->yaw);
  }

  void DragonGimbal::targetCogRotCallback(const spinal::DesireCoordConstPtr& msg)
  {
    kinematics_->setCogDesireOrientation(msg->roll, msg->pitch, msg->yaw);
  }

  void DragonGimbal::attControlFeedbackStateCallback(const spinal::RollPitchYawTermConstPtr& msg)
  {
    if(motor_num_ == 0) return;

    /* calcualte the control term about attitude in spinal based on LQI */
    /* -- only consider the P term and I term, since D term (angular velocity from gyro) in current Dragon platform is too noisy -- */
    for(int i = 0; i < motor_num_; i++)
      {
        att_lqi_term_.at(i) = -roll_gains_[i][0] * (msg->roll_p / 1000.0) + roll_gains_[i][1] * (msg->roll_i / 1000.0) + (-pitch_gains_[i][0]) * (msg->pitch_p / 1000.0) + pitch_gains_[i][1] * (msg->pitch_i / 1000.0);
      }
  }

  bool DragonGimbal::update()
  {
    if(!ControlBase::update() || gimbal_vectoring_check_flag_) return false;

    landingProcess();

    servoTorqueProcess();
    stateError();

    pidUpdate(); //LQI thrust control
    gimbalControl(); //gimbal vectoring control
    targetCogRotation();
    sendCmd();
  }

  void DragonGimbal::landingProcess()
  {

    sensor_msgs::JointState joint_control_msg;
    for(int i = 0; i < joint_state_.position.size(); i++)
      {
        if(joint_state_.name[i].find("joint") != std::string::npos)
          {
            double target_cmd;
            if(joint_state_.name[i].find("pitch") != std::string::npos)
              target_cmd = 0;
            else target_cmd = joint_state_.position[i];

            joint_control_msg.position.push_back(target_cmd);

          }
      }

    if(navigator_->getForceLandingFlag() || navigator_->getNaviState() == Navigator::LAND_STATE)
      {
        if(!level_flag_)
          {
            joint_control_pub_.publish(joint_control_msg);
            final_target_cog_rot_.setValue(0, 0, 0);

            /* force set the current deisre tilt to current estimated tilt */
            curr_target_cog_rot_.setValue(estimator_->getState(State::ROLL_BASE, estimate_mode_)[0], estimator_->getState(State::PITCH_BASE, estimate_mode_)[0], 0);
          }

        level_flag_ = true;

        if(navigator_->getNaviState() == Navigator::LAND_STATE && !landing_flag_)
          {
            landing_flag_ = true;
            navigator_->setTeleopFlag(false);
            navigator_->setTargetPosZ(estimator_->getState(State::Z_COG, estimate_mode_)[0]);
            navigator_->setNaviState(Navigator::HOVER_STATE);
          }
      }

    /* back to landing process */
    if(landing_flag_)
      {
        bool already_level = true;
        for(int i = 0; i < joint_state_.position.size(); i++)
          {
            if(joint_state_.name[i].find("joint") != std::string::npos
               && joint_state_.name[i].find("pitch") != std::string::npos)
              {
                if(fabs(joint_state_.position[i]) > 0.085) already_level = false;
              }
          }
        //ROS_INFO("curr_target_cog_rot: [%f, %f], nomr: %f", curr_target_cog_rot_.x(), curr_target_cog_rot_.y(), curr_target_cog_rot_.length());
        if(curr_target_cog_rot_.length()) already_level = false;

        if(already_level && navigator_->getNaviState() == Navigator::HOVER_STATE)
          {
            ROS_WARN("gimbal control: back to land state");
            navigator_->setNaviState(Navigator::LAND_STATE);
            navigator_->setTargetPosZ(estimator_->getLandingHeight());
            navigator_->setTeleopFlag(true);
          }
      }
  }

  void DragonGimbal::gimbalControl()
  {

    if (control_timestamp_ < 0) return;

    /* get roll/pitch angle */
    double roll_angle = estimator_->getState(State::ROLL_COG, estimate_mode_)[0];
    double pitch_angle = estimator_->getState(State::PITCH_COG, estimate_mode_)[0];

    /* do not do gimbal control in the early stage of takeoff phase */
    /*
    if(navigator_->getNaviState() == Navigator::TAKEOFF_STATE)
      {
        double total_thrust  = 0;
        for(int j = 0; j < motor_num_; j++)
          total_thrust += target_throttle_[j];
        if(kinematics_->getMass() * 0.75 > total_thrust / 10)
          {
            //ROS_INFO("force is to small, mass: %f, force: %f", kinematics_->getMass(), total_thrust / 10);
            return;
          }
      }
    */

    int rotor_num = kinematics_->getRotorNum();

    std::vector<Eigen::Vector3d> rotors_origin_from_cog = kinematics_->getRotorsOriginFromCog<Eigen::Vector3d>();
    Eigen::Matrix3d links_inertia = kinematics_->getInertia<Eigen::Matrix3d>();

    /* roll pitch condition */
    double max_x = 1e-6;
    double max_y = 1e-6;
    double max_z = 1e-6;

    Eigen::MatrixXd P_att = Eigen::MatrixXd::Zero(3, rotor_num  * 2);
    double f_z_sum = 0;
    for(int i = 0; i < rotor_num; i++)
      {
        /* obatin mapping matrix */
        P_att(0, 2 * i + 1) = -rotors_origin_from_cog[i](2); //x(roll)
        P_att(1, 2 * i) = rotors_origin_from_cog[i](2); //y(pitch)
        P_att(2, 2 * i) = -rotors_origin_from_cog[i](1);
        P_att(2, 2 * i + 1) = rotors_origin_from_cog[i](0);

        /* roll pitch condition */
        if(fabs(rotors_origin_from_cog[i](0)) > max_x) max_x = fabs(rotors_origin_from_cog[i](0));
        if(fabs(rotors_origin_from_cog[i](1)) > max_y) max_y = fabs(rotors_origin_from_cog[i](1));
        if(fabs(rotors_origin_from_cog[i](2)) > max_z) max_z = fabs(rotors_origin_from_cog[i](2));

        /* obtain the sum of f_z w.r.t {CoG} frame */
        f_z_sum += (att_lqi_term_.at(i) + target_throttle_.at(i));
        //f_z_sum += target_throttle_.at(i); // only LQI(f_z)
      }

    Eigen::MatrixXd P_att_orig = P_att;
    P_att = links_inertia.inverse() * P_att_orig;


    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(5, rotor_num  * 2);
    P.block(0, 0, 3, rotor_num * 2) = P_att;
    P.block(3, 0, 2, rotor_num * 2) = P_xy_ / kinematics_->getMass();

    double P_det = (P * P.transpose()).determinant();

    if(control_verbose_)
    {
      std::cout << "gimbal P original :"  << std::endl << P_att_orig << std::endl;
      std::cout << "gimbal P:"  << std::endl << P << std::endl;
      std::cout << "P det:"  << std::endl << P_det << std::endl;
      std::cout << "f_z_sum / mass: " << f_z_sum / kinematics_->getMass() << std::endl;
    }

    Eigen::VectorXd f_xy_vec;
    if(P_det < pitch_roll_control_p_det_thresh_) //no pitch and roll control
      {
        if(control_verbose_) ROS_ERROR("bad P_det: %f", P_det);
        P = Eigen::MatrixXd::Zero(3, rotor_num  * 2);
        P.block(0, 0, 1, rotor_num * 2) = P_att.block(2, 0, 1, rotor_num * 2);
        P.block(1, 0, 2, rotor_num * 2) = P_xy_ / kinematics_->getMass();

        f_xy_vec = pseudoinverse(P) * Eigen::Vector3d(target_yaw_[0], target_pitch_ - (pitch_angle * f_z_sum / kinematics_->getMass()), target_roll_ - (-roll_angle * f_z_sum / kinematics_->getMass()));
        /* -- approximation: around hovering state --*/
        //f_xy_vec = pseudoinverse(P) * Eigen::Vector3d(target_yaw_[0], target_pitch_ - (pitch_angle * 9.8), target_roll_ - (-roll_angle * 9.8));
      }
    else
      {
        /* roll & pitch gimbal additional control */
        double target_gimbal_roll = 0, target_gimbal_pitch = 0;
        if(gimbal_control_stamp_ == 0) gimbal_control_stamp_ = ros::Time::now().toSec();
        double du = ros::Time::now().toSec() - gimbal_control_stamp_;

        /* ros pub */
        aerial_robot_msgs::FlatnessPid pid_msg;
        pid_msg.header.stamp = ros::Time::now();

        //ROS_WARN("max_x: %f, max_y: %f, max_z: %f", max_x, max_y, max_z);

        /* roll addition control */
        if(max_z > pitch_roll_control_rate_thresh_ * max_y)
          {
            if(control_verbose_)
              ROS_WARN("roll gimbal control, max_z: %f, max_y: %f", max_z, max_y);

            double p_term, d_term;
            double state_phy = estimator_->getState(State::ROLL_COG, estimate_mode_)[0];
            double state_phy_vel = estimator_->getState(State::ROLL_COG, estimate_mode_)[1];

            /* P */
            p_term = clamp(pitch_roll_gains_[0] * (-state_phy),  -pitch_roll_terms_limits_[0], pitch_roll_terms_limits_[0]);

            /* I */
            roll_i_term_ += (-state_phy * du * pitch_roll_gains_[1]);
            roll_i_term_ = clamp(roll_i_term_, -pitch_roll_terms_limits_[1], pitch_roll_terms_limits_[1]);

            /* D */
            d_term = clamp(pitch_roll_gains_[2] * (-state_phy_vel),  -pitch_roll_terms_limits_[2], pitch_roll_terms_limits_[2]);

            target_gimbal_roll = clamp(p_term + roll_i_term_ + d_term, -pitch_roll_limit_, pitch_roll_limit_);

            pid_msg.roll.total.push_back(target_gimbal_roll);
            pid_msg.roll.p_term.push_back(p_term);
            pid_msg.roll.i_term.push_back(roll_i_term_);
            pid_msg.roll.d_term.push_back(d_term);

            pid_msg.roll.pos_err = state_phy;
            pid_msg.roll.target_pos = 0;
            pid_msg.roll.vel_err = state_phy_vel;
            pid_msg.roll.target_vel = 0;

          }

        /* pitch addition control */
        if(max_z > pitch_roll_control_rate_thresh_ * max_x)
          {
            if(control_verbose_)
              ROS_WARN("pitch gimbal control, max_z: %f, max_x: %f", max_z, max_y);

            double p_term, d_term;
            double state_theta = estimator_->getState(State::PITCH_COG, estimate_mode_)[0];
            double state_theta_vel = estimator_->getState(State::PITCH_COG, estimate_mode_)[1];

            /* P */
            p_term = clamp(pitch_roll_gains_[0] * (-state_theta),  -pitch_roll_terms_limits_[0], pitch_roll_terms_limits_[0]);

            /* I */
            pitch_i_term_ += (-state_theta * du * pitch_roll_gains_[1]);
            pitch_i_term_ = clamp(pitch_i_term_, -pitch_roll_terms_limits_[1], pitch_roll_terms_limits_[1]);

            /* D */
            d_term = clamp(pitch_roll_gains_[2] * (-state_theta_vel),  -pitch_roll_terms_limits_[2], pitch_roll_terms_limits_[2]);

            target_gimbal_pitch = clamp(p_term + pitch_i_term_ + d_term, -pitch_roll_limit_, pitch_roll_limit_);

            pid_msg.pitch.total.push_back(target_gimbal_pitch);
            pid_msg.pitch.p_term.push_back(p_term);
            pid_msg.pitch.i_term.push_back(pitch_i_term_);
            pid_msg.pitch.d_term.push_back(d_term);

            pid_msg.pitch.pos_err = state_theta;
            pid_msg.pitch.target_pos = 0;
            pid_msg.pitch.vel_err = state_theta_vel;
            pid_msg.pitch.target_vel = 0;
          }

        roll_pitch_pid_pub_.publish(pid_msg);
        gimbal_control_stamp_ = ros::Time::now().toSec();

        Eigen::VectorXd pid_values(5);
        /* F = P# * [roll_pid, pitch_pid, yaw_pid, x_pid, y_pid] */

        pid_values << target_gimbal_roll, target_gimbal_pitch, target_yaw_[0], target_pitch_ - (pitch_angle * f_z_sum / kinematics_->getMass()), target_roll_ - (-roll_angle * f_z_sum / kinematics_->getMass());
        /* -- approximation: around hovering state --*/
        //pid_values << target_gimbal_roll, target_gimbal_pitch, target_yaw_[0], target_pitch_ - (pitch_angle * 9.8), target_roll_ - (-roll_angle * 9.8);
        f_xy_vec = pseudoinverse(P) * pid_values;
      }

    if(control_verbose_)
      {
        std::cout << "gimbal P_pseudo_inverse:"  << std::endl << pseudoinverse(P) << std::endl;
        //std::cout << "gimbal P * P_pseudo_inverse:"  << std::endl << P * pseudoinverse(P) << std::endl;
        std::cout << "gimbal f:"  << std::endl << f_xy_vec << std::endl;
      }

    std_msgs::Float32MultiArray target_force_msg;

    for(int i = 0; i < rotor_num; i++)
      {
        /* vectoring force */
        /* --  LQI(z) + LQI(roll) + LQI(pitch) -- */
        tf::Vector3 f_i(f_xy_vec(2 * i), f_xy_vec(2 * i + 1), target_throttle_.at(i) + att_lqi_term_.at(i));
        /* --  only LQI(z) -- */
        //tf::Vector3 f_i(f_xy_vec(2 * i), f_xy_vec(2 * i + 1), target_throttle_.at(i));

        /* calculate ||f|| */
        target_thrust_vector_.at(i) = (f_i - tf::Vector3(0, 0, att_lqi_term_.at(i))).length(); //omit pitch and roll term, which will be added in spinal
        if(control_verbose_)
          ROS_INFO("[gimbal control]: rotor%d, target_thrust vs target_z: [%f vs %f]", i+1, target_thrust_vector_.at(i), target_throttle_.at(i));

        /* -------------------------------------------------------------
           in the early stage of takeoff, avoiding the large tilt angle,
           because of the small throttle
           --------------------------------------------------------------- */
        if(!start_rp_integration_)
          f_i.setValue(f_xy_vec(2 * i), f_xy_vec(2 * i + 1), kinematics_->getOptimalHoveringThrust()[i]);

        /* normalized vector */
        auto f_i_normalized = f_i.normalize();
        if(control_verbose_) ROS_INFO("gimbal%d f normaizled: [%f, %f, %f]", i + 1, f_i_normalized.x(), f_i_normalized.y(), f_i_normalized.z());

        /*f -> gimbal angle */
        std::vector<KDL::Rotation> links_frame_from_cog = kinematics_->getLinksRotationFromCog<KDL::Rotation>();

        /* [S_theta, -S_phy * C_theta, C_phy * C_phy]^T = R.transpose * f_i_normalized */
        tf::Quaternion q;  tf::quaternionKDLToTF(links_frame_from_cog[i], q);
        tf::Vector3 r_f_i_normalized = tf::Matrix3x3(q).inverse() * f_i_normalized;
        if(control_verbose_) ROS_INFO("gimbal%d r f normaizled: [%f, %f, %f]", i + 1, r_f_i_normalized.x(), r_f_i_normalized.y(), r_f_i_normalized.z());

        double gimbal_i_roll = atan2(-r_f_i_normalized[1], r_f_i_normalized[2]);
        double gimbal_i_pitch = atan2(r_f_i_normalized[0], -r_f_i_normalized[1] * sin(gimbal_i_roll) + r_f_i_normalized[2] * cos(gimbal_i_roll));
        /* TODO: check the singularity: gimbal pitch = pi/2 */

        target_gimbal_angles_[2 * i] = gimbal_i_roll;
        target_gimbal_angles_[2 * i + 1] = gimbal_i_pitch;

        if(control_verbose_) std::cout << "gimbal" << i + 1 <<"r & p: " << gimbal_i_roll << ", "<< gimbal_i_pitch  << std::endl;

        /* ros publish */
        target_force_msg.data.push_back(f_xy_vec(2 * i));
        target_force_msg.data.push_back(f_xy_vec(2 * i + 1));
      }
    gimbal_target_force_pub_.publish(target_force_msg);
  }

  void DragonGimbal::targetCogRotation()
  {
    static ros::Time prev_stamp = ros::Time::now();
    if(curr_target_cog_rot_ == final_target_cog_rot_) return;

    //ROS_INFO("current target_cog_rot_: [%f, %f, %f], final_target_cog_rot_: [%f, %f, %f]", curr_target_cog_rot_.x(), curr_target_cog_rot_.y(), curr_target_cog_rot_.z(), final_target_cog_rot_.x(), final_target_cog_rot_.y(), final_target_cog_rot_.z());

    if(ros::Time::now().toSec() - prev_stamp.toSec() > tilt_pub_interval_)
      {
        if((final_target_cog_rot_- curr_target_cog_rot_).length() > tilt_thresh_)
          curr_target_cog_rot_ += ((final_target_cog_rot_ - curr_target_cog_rot_).normalize() * tilt_thresh_);
        else
          curr_target_cog_rot_ = final_target_cog_rot_;

        spinal::DesireCoord target_cog_rot_msg;
        target_cog_rot_msg.roll = curr_target_cog_rot_.x();
        target_cog_rot_msg.pitch = curr_target_cog_rot_.y();
        curr_target_cog_rot_pub_.publish(target_cog_rot_msg);

        prev_stamp = ros::Time::now();
      }
  }

  void DragonGimbal::sendCmd()
  {
    /* send base throttle command */
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] =  0;
    flight_command_data.angles[1] =  0;

    flight_command_data.base_throttle.resize(motor_num_);
    for(int i = 0; i < motor_num_; i++)
      {
        /* ----------------------------------------
           1. need to send ||f||, instead of LQI(z).
           2. exclude the attitude part (LQI(roll), LQI(pitch), wich is calculated in spinal.
           ----------------------------------------- */
        flight_command_data.base_throttle[i] = target_thrust_vector_.at(i);

        /* -- general no-vectoring case: only LQI(z) --*/
        //flight_command_data.base_throttle[i] = target_throttle_.at(i);
      }
    flight_cmd_pub_.publish(flight_command_data);

    /* send gimbal control command */
    sensor_msgs::JointState gimbal_control_msg;
    gimbal_control_msg.header.stamp = ros::Time::now();
    for(int i = 0; i < kinematics_->getRotorNum() * 2; i++)
      gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));

    gimbal_control_pub_.publish(gimbal_control_msg);
  }

  void DragonGimbal::servoTorqueProcess()
  {
    if(!real_machine_) return;

    ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(joints_torque_control_srv_name_);
    std_srvs::SetBool srv;

    if(servo_torque_)
      {
        if(navigator_->getNaviState() == Navigator::LAND_STATE)
          {
            if(state_pos_.z() < height_thresh_ && real_machine_)
              {

                srv.request.data = false;

                if (client.call(srv))
                  {
                    ROS_INFO("dragon control: disable the joint torque");
                    servo_torque_ = false;
                  }
                else
                  {
                    ROS_ERROR("Failed to call service %s", joints_torque_control_srv_name_.c_str());
                  }
              }
          }

        /*
          if(navigator_->getForceLandingFlag() == true)
          {
          srv.request.torque_enable = false;
          if (client.call(srv))
          {
          ROS_INFO("dragon control: disable the joint torque");
          servo_torque_ = false;
          }
          else
          {
          ROS_ERROR("Failed to call service %s", joints_torque_control_srv_name_.c_str());
          }
          }
        */
      }
    else
      {
        if(navigator_->getNaviState() == Navigator::ARM_ON_STATE)
          {
            srv.request.data = true;

            if (client.call(srv))
              {
                ROS_INFO("dragon control: enable the joint torque");
                servo_torque_ = true;
              }
            else
              {
                ROS_ERROR("Failed to call service %s", joints_torque_control_srv_name_.c_str());
              }
          }
      }
  }

  void DragonGimbal::jointStateCallback(const sensor_msgs::JointStateConstPtr& state)
  {
    joint_state_ = *state;
    kinematics_->updateRobotModel(joint_state_);
    kinematics_->modelling();

    /* check the gimbal vectoring function */
    if (gimbal_vectoring_check_flag_)
      {
        sensor_msgs::JointState gimbal_control_msg;
        gimbal_control_msg.header = state->header;
        gimbal_control_msg.position = kinematics_->getGimbalNominalAngles();
        gimbal_control_pub_.publish(gimbal_control_msg);
      }
  }

  void DragonGimbal::cfgPitchRollPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level)
  {
    if(config.xy_pid_control_flag)
      {
        printf("Pitch Roll Pid Param:");
        switch(level)
          {
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_GAIN:
            pitch_roll_gains_[0] = config.p_gain;
            printf("change the p gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_GAIN:
            pitch_roll_gains_[1] = config.i_gain;
            printf("change the i gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_GAIN:
            pitch_roll_gains_[2] = config.d_gain;
            printf("change the d gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_LIMIT:
            pitch_roll_limit_ = config.limit;
            printf("change the limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_LIMIT:
            pitch_roll_terms_limits_[0] = config.p_limit;
            printf("change the p limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_LIMIT:
            pitch_roll_terms_limits_[1] = config.i_limit;
            printf("change the i limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_LIMIT:
            pitch_roll_terms_limits_[2] = config.d_limit;
            printf("change the d limit\n");
            break;
          default :
            printf("\n");
            break;
          }
      }
  }

  void DragonGimbal::cfgYawPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level)
  {
    if(config.xy_pid_control_flag)
      {
        printf("Yaw Pid Param:");
        switch(level)
          {
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_GAIN:
            yaw_gains_[0][0] = -config.p_gain;
            printf("change the p gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_GAIN:
            yaw_gains_[0][1] = config.i_gain;
            printf("change the i gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_GAIN:
            yaw_gains_[0][2] = config.d_gain;
            printf("change the d gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_LIMIT:
            yaw_limit_ = config.limit;
            printf("change the limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_LIMIT:
            yaw_terms_limits_[0] = config.p_limit;
            printf("change the p limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_LIMIT:
            yaw_terms_limits_[1] = config.i_limit;
            printf("change the i limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_LIMIT:
            yaw_terms_limits_[2] = config.d_limit;
            printf("change the d limit\n");
            break;
          default :
            printf("\n");
            break;
          }
      }
  }

  void DragonGimbal::rosParamInit()
  {
    FlatnessPid::rosParamInit();

    string ns = nhp_.getNamespace();
    nhp_.param("control_verbose", control_verbose_, false);
    if(param_verbose_) cout << ns << ": control_verbose is " << control_verbose_ << endl;
    nhp_.param("real_machine", real_machine_, true);
    if(param_verbose_) cout << ns << ": real_machine is " << real_machine_ << endl;
    nhp_.param("joints_torque_control_srv_name", joints_torque_control_srv_name_, std::string("/joints_controller/torque_enable"));

    /* height threshold to disable the joint servo when landing */
    nhp_.param("height_thresh", height_thresh_, 0.1);
    if(param_verbose_) cout << ns << ": height_thresh is " << height_thresh_ << endl;

    /* the threshold to tilt smoothly */
    nhp_.param("tilt_thresh", tilt_thresh_, 0.02);
    if(param_verbose_) cout << ns << ": tilt_thresh is " << tilt_thresh_ << endl;

    /* the rate to pub target tilt  command */
    nhp_.param("tilt_pub_interval", tilt_pub_interval_, 0.1);
    if(param_verbose_) cout << ns << ": tilt_pub_interval is " << tilt_pub_interval_ << endl;

    /* check the gimbal vectoring function without position and yaw control */
    nhp_.param("gimbal_vectoring_check_flag", gimbal_vectoring_check_flag_, false);
    if(param_verbose_) cout << ns << ": gimbal_vectoring_check_flag is " << gimbal_vectoring_check_flag_ << endl;

    /* pitch roll control */
    ros::NodeHandle pitch_roll_node(nhp_, "pitch_roll");
    string pitch_roll_ns = pitch_roll_node.getNamespace();
    pitch_roll_node.param("control_rate_thresh", pitch_roll_control_rate_thresh_, 1.0);
    if(param_verbose_) cout << pitch_roll_ns << ": pitch_roll_control_rate_thresh_ is " <<  pitch_roll_control_rate_thresh_ << endl;
    pitch_roll_node.param("control_p_det_thresh", pitch_roll_control_p_det_thresh_, 1e-3);
    if(param_verbose_) cout << pitch_roll_ns << ": pitch_roll_control_p_det_thresh_ is " <<  pitch_roll_control_p_det_thresh_ << endl;

    pitch_roll_node.param("limit", pitch_roll_limit_, 1.0e6);
    if(param_verbose_) cout << pitch_roll_ns << ": pos_limit_ is " <<  pitch_roll_limit_ << endl;
    pitch_roll_node.param("p_term_limit", pitch_roll_terms_limits_[0], 1.0e6);
    if(param_verbose_) cout << pitch_roll_ns << ": pos_p_limit_ is " <<  pitch_roll_terms_limits_[0] << endl;
    pitch_roll_node.param("i_term_limit", pitch_roll_terms_limits_[1], 1.0e6);
    if(param_verbose_) cout << pitch_roll_ns << ": pos_i_limit_ is " <<  pitch_roll_terms_limits_[1] << endl;
    pitch_roll_node.param("d_term_limit", pitch_roll_terms_limits_[2], 1.0e6);
    if(param_verbose_) cout << pitch_roll_ns << ": pos_d_limit_ is " <<  pitch_roll_terms_limits_[2] << endl;
    pitch_roll_node.param("p_gain", pitch_roll_gains_[0], 0.0);
    if(param_verbose_) cout << pitch_roll_ns << ": p_gain_ is " << pitch_roll_gains_[0] << endl;
    pitch_roll_node.param("i_gain", pitch_roll_gains_[1], 0.0);
    if(param_verbose_) cout << pitch_roll_ns << ": i_gain_ is " << pitch_roll_gains_[1] << endl;
    pitch_roll_node.param("d_gain", pitch_roll_gains_[2], 0.0);
    if(param_verbose_) cout << pitch_roll_ns << ": d_gain_ is " << pitch_roll_gains_[2] << endl;
  }
};

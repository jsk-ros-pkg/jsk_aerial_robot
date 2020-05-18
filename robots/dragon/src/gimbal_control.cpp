#include <dragon/gimbal_control.h>

using namespace aerial_robot_model;

namespace control_plugin
{
  DragonGimbal::DragonGimbal():
    FlatnessPid(),
    servo_torque_(false), level_flag_(false), landing_flag_(false),
    curr_target_baselink_rot_(0, 0, 0),
    final_target_baselink_rot_(0, 0, 0),
    roll_i_term_(0), pitch_i_term_(0),
    gimbal_roll_control_stamp_(0), gimbal_pitch_control_stamp_(0)
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
    /* additional vectoring force for grasping */
    extra_vectoring_force_ = Eigen::VectorXd::Zero(3 * kinematics_->getRotorNum());

    gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
    joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
    curr_target_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
    gimbal_target_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/gimbals_target_force", 1);
    roll_pitch_pid_pub_ = nh_.advertise<aerial_robot_msgs::FlatnessPid>("debug/roll_pitch_gimbal_control", 1);

    att_control_feedback_state_sub_ = nh_.subscribe("rpy/feedback_state", 1, &DragonGimbal::attControlFeedbackStateCallback, this);
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &DragonGimbal::jointStateCallback, this);
    final_target_baselink_rot_sub_ = nh_.subscribe("final_target_baselink_rot", 1, &DragonGimbal::setFinalTargetBaselinkRotCallback, this);
    target_baselink_rot_sub_ = nh_.subscribe("desire_coordinate", 1, &DragonGimbal::targetBaselinkRotCallback, this);
    extra_vectoring_force_sub_ = nh_.subscribe("extra_vectoring_force", 1, &DragonGimbal::extraVectoringForceCallback, this);

    std::string service_name;
    nh_.param("apply_external_wrench", service_name, std::string("apply_external_wrench"));
    add_external_wrench_service_ = nh_.advertiseService(service_name, &DragonGimbal::addExternalWrenchCallback, this);
    nh_.param("clear_external_wrench", service_name, std::string("clear_external_wrench"));
    clear_external_wrench_service_ = nh_.advertiseService(service_name, &DragonGimbal::clearExternalWrenchCallback, this);

    //dynamic reconfigure server
    roll_pitch_pid_server_ = new dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>(ros::NodeHandle(nhp_, "gain_generator/pitch_roll"));
    dynamic_reconf_func_roll_pitch_pid_ = boost::bind(&DragonGimbal::cfgPitchRollPidCallback, this, _1, _2);
    roll_pitch_pid_server_->setCallback(dynamic_reconf_func_roll_pitch_pid_);

    yaw_pid_server_ = new dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>(ros::NodeHandle(nhp_, "gain_generator/yaw"));
    dynamic_reconf_func_yaw_pid_ = boost::bind(&DragonGimbal::cfgYawPidCallback, this, _1, _2);
    yaw_pid_server_->setCallback(dynamic_reconf_func_yaw_pid_);
  }

  void DragonGimbal::fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg)
  {
    /* update the motor number */
    if(motor_num_ == 0)
      {
        motor_num_ = msg->motor_num;

        target_thrust_terms_.resize(motor_num_);

        z_gains_.resize(motor_num_);
        z_control_terms_.resize(motor_num_);

        lqi_roll_gains_.resize(motor_num_);
        lqi_pitch_gains_.resize(motor_num_);
        lqi_att_terms_.resize(motor_num_, 0);

        ROS_WARN("gimbal flight control: update the motor number: %d", motor_num_);
      }

    for(int i = 0; i < msg->motor_num; i++)
      {
        z_gains_[i].setValue(msg->pos_p_gain_z[i], msg->pos_i_gain_z[i], msg->pos_d_gain_z[i]);

        lqi_pitch_gains_[i].setValue(msg->pos_p_gain_pitch[i], msg->pos_i_gain_pitch[i], msg->pos_d_gain_pitch[i]);
        lqi_roll_gains_[i].setValue(msg->pos_p_gain_roll[i], msg->pos_i_gain_roll[i], msg->pos_d_gain_roll[i]);
      }
  }

  void DragonGimbal::setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg)
  {
    final_target_baselink_rot_.setValue(msg->roll, msg->pitch, msg->yaw);
  }

  void DragonGimbal::targetBaselinkRotCallback(const spinal::DesireCoordConstPtr& msg)
  {
    kinematics_->setCogDesireOrientation(msg->roll, msg->pitch, msg->yaw);
  }

  void DragonGimbal::attControlFeedbackStateCallback(const spinal::RollPitchYawTermConstPtr& msg)
  {
    if(motor_num_ == 0) return;

    if(!add_lqi_result_) return;

    /* reproduce the control term about attitude in spinal based on LQI */
    /* -- only consider the P term and I term, since D term (angular velocity from gyro) in current Dragon platform is too noisy -- */

    for(int i = 0; i < motor_num_; i++)
      {
        lqi_att_terms_.at(i) = -lqi_roll_gains_[i][0] * (msg->roll_p / 1000.0) + lqi_roll_gains_[i][1] * (msg->roll_i / 1000.0) + (-lqi_pitch_gains_[i][0]) * (msg->pitch_p / 1000.0) + lqi_pitch_gains_[i][1] * (msg->pitch_i / 1000.0);
      }

    // std::cout << "reproduce lqi att control term: ";
    //for(int i = 0; i < motor_num_; i++)
    // std::cout << lqi_att_terms_.at(i) << ", ";
    // std::cout << std::endl;
  }

  bool DragonGimbal::update()
  {
    if(!ControlBase::update() || gimbal_vectoring_check_flag_) return false;

    landingProcess();

    servoTorqueProcess();
    stateError();

    pidUpdate(); //LQI thrust control
    gimbalControl(); //gimbal vectoring control
    baselinkRotationProcess();
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
            final_target_baselink_rot_.setValue(0, 0, 0);

            /* force set the current deisre tilt to current estimated tilt */
            curr_target_baselink_rot_.setValue(estimator_->getState(State::ROLL_BASE, estimate_mode_)[0], estimator_->getState(State::PITCH_BASE, estimate_mode_)[0], 0);

            /* clear the external wrench */
            kinematics_->resetExternalStaticWrench();

            /* clear the extra vectoring force */
            extra_vectoring_force_.setZero();
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

        if(curr_target_baselink_rot_.length()) already_level = false;

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
    double yaw_angle = estimator_->getState(State::YAW_COG, estimate_mode_)[0];

    int rotor_num = kinematics_->getRotorNum();

    std::vector<Eigen::Vector3d> rotors_origin_from_cog = kinematics_->getRotorsOriginFromCog<Eigen::Vector3d>();
    Eigen::Matrix3d links_inertia = kinematics_->getInertia<Eigen::Matrix3d>();

    /* roll pitch condition */
    double max_x = 1e-6;
    double max_y = 1e-6;
    double max_z = 1e-6;

    Eigen::MatrixXd P_att = Eigen::MatrixXd::Zero(3, rotor_num  * 2);
    double acc_z = 0;
    for(int i = 0; i < rotor_num; i++)
      {
        P_att(0, 2 * i + 1) = -rotors_origin_from_cog[i](2); //x(roll)
        P_att(1, 2 * i) = rotors_origin_from_cog[i](2); //y(pitch)
        P_att(2, 2 * i) = -rotors_origin_from_cog[i](1);
        P_att(2, 2 * i + 1) = rotors_origin_from_cog[i](0);

        /* roll pitch condition */
        if(fabs(rotors_origin_from_cog[i](0)) > max_x) max_x = fabs(rotors_origin_from_cog[i](0));
        if(fabs(rotors_origin_from_cog[i](1)) > max_y) max_y = fabs(rotors_origin_from_cog[i](1));
        if(fabs(rotors_origin_from_cog[i](2)) > max_z) max_z = fabs(rotors_origin_from_cog[i](2));

        acc_z += (lqi_att_terms_.at(i) + z_control_terms_.at(i));
      }
    acc_z /= kinematics_->getMass();

    Eigen::MatrixXd P_att_orig = P_att;
    P_att = links_inertia.inverse() * P_att_orig;


    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(5, rotor_num  * 2);
    P.block(0, 0, 3, rotor_num * 2) = P_att;
    P.block(3, 0, 2, rotor_num * 2) = P_xy_ / kinematics_->getMass();

    double P_det = (P * P.transpose()).determinant();

    if(control_verbose_)
      {
        std::cout << "gimbal P original: \n"  << std::endl << P_att_orig << std::endl;
        std::cout << "gimbal P: \n"  << std::endl << P << std::endl;
        std::cout << "P det: "  << std::endl << P_det << std::endl;
        std::cout << "acc_z: " << acc_z  << std::endl;
      }

    Eigen::VectorXd f_xy;

    if(P_det < pitch_roll_control_p_det_thresh_)
      {
        // no pitch roll
        if(control_verbose_) ROS_ERROR("bad P_det: %f", P_det);
        P = Eigen::MatrixXd::Zero(3, rotor_num  * 2);
        P.block(0, 0, 1, rotor_num * 2) = P_att.block(2, 0, 1, rotor_num * 2);
        P.block(1, 0, 2, rotor_num * 2) = P_xy_ / kinematics_->getMass();

        f_xy = pseudoinverse(P) * Eigen::Vector3d(yaw_control_terms_[0], target_pitch_ - (pitch_angle * acc_z), (-target_roll_) - (-roll_angle * acc_z));

        // reset gimbal roll pitch control
        roll_i_term_ = 0;
        pitch_i_term_ = 0;
        gimbal_roll_control_stamp_ = 0;
        gimbal_pitch_control_stamp_ = 0;
      }
    else
      {
        /* roll & pitch gimbal additional control */
        double target_gimbal_roll = 0, target_gimbal_pitch = 0;

        /* ros pub */
        aerial_robot_msgs::FlatnessPid pid_msg;
        pid_msg.header.stamp = ros::Time::now();

        /* roll addition control */
        if(max_z > pitch_roll_control_rate_thresh_ * max_y)
          {
            if(gimbal_roll_control_stamp_ == 0)
              gimbal_roll_control_stamp_ = ros::Time::now().toSec();
            double du = ros::Time::now().toSec() - gimbal_roll_control_stamp_;

            if(control_verbose_)
              ROS_WARN("roll gimbal control, max_z: %f, max_y: %f", max_z, max_y);

            double p_term, d_term;
            double state_roll = estimator_->getState(State::ROLL_COG, estimate_mode_)[0];
            double state_roll_vel = estimator_->getState(State::ROLL_COG, estimate_mode_)[1];

            /* P */
            p_term = clamp(pitch_roll_gains_[0] * (-state_roll),  -pitch_roll_terms_limits_[0], pitch_roll_terms_limits_[0]);

            /* I */
            roll_i_term_ += (-state_roll * du * pitch_roll_gains_[1]);
            roll_i_term_ = clamp(roll_i_term_, -pitch_roll_terms_limits_[1], pitch_roll_terms_limits_[1]);

            /* D */
            d_term = clamp(pitch_roll_gains_[2] * (-state_roll_vel),  -pitch_roll_terms_limits_[2], pitch_roll_terms_limits_[2]);

            target_gimbal_roll = clamp(p_term + roll_i_term_ + d_term, -pitch_roll_limit_, pitch_roll_limit_);

            pid_msg.roll.total.push_back(target_gimbal_roll);
            pid_msg.roll.p_term.push_back(p_term);
            pid_msg.roll.i_term.push_back(roll_i_term_);
            pid_msg.roll.d_term.push_back(d_term);

            pid_msg.roll.pos_err = state_roll;
            pid_msg.roll.target_pos = 0;
            pid_msg.roll.vel_err = state_roll_vel;
            pid_msg.roll.target_vel = 0;

            gimbal_roll_control_stamp_ = ros::Time::now().toSec();
          }
        else
          {
            //reset
            roll_i_term_ = 0;
            gimbal_roll_control_stamp_ = 0;
          }

        /* pitch addition control */
        if(max_z > pitch_roll_control_rate_thresh_ * max_x)
          {
            if(gimbal_pitch_control_stamp_ == 0)
              gimbal_pitch_control_stamp_ = ros::Time::now().toSec();
            double du = ros::Time::now().toSec() - gimbal_pitch_control_stamp_;


            if(control_verbose_)
              ROS_WARN("pitch gimbal control, max_z: %f, max_x: %f", max_z, max_y);

            double p_term, d_term;
            double state_pitch = estimator_->getState(State::PITCH_COG, estimate_mode_)[0];
            double state_pitch_vel = estimator_->getState(State::PITCH_COG, estimate_mode_)[1];

            /* P */
            p_term = clamp(pitch_roll_gains_[0] * (-state_pitch),  -pitch_roll_terms_limits_[0], pitch_roll_terms_limits_[0]);

            /* I */
            pitch_i_term_ += (-state_pitch * du * pitch_roll_gains_[1]);
            pitch_i_term_ = clamp(pitch_i_term_, -pitch_roll_terms_limits_[1], pitch_roll_terms_limits_[1]);

            /* D */
            d_term = clamp(pitch_roll_gains_[2] * (-state_pitch_vel),  -pitch_roll_terms_limits_[2], pitch_roll_terms_limits_[2]);

            target_gimbal_pitch = clamp(p_term + pitch_i_term_ + d_term, -pitch_roll_limit_, pitch_roll_limit_);

            pid_msg.pitch.total.push_back(target_gimbal_pitch);
            pid_msg.pitch.p_term.push_back(p_term);
            pid_msg.pitch.i_term.push_back(pitch_i_term_);
            pid_msg.pitch.d_term.push_back(d_term);

            pid_msg.pitch.pos_err = state_pitch;
            pid_msg.pitch.target_pos = 0;
            pid_msg.pitch.vel_err = state_pitch_vel;
            pid_msg.pitch.target_vel = 0;

            gimbal_pitch_control_stamp_ = ros::Time::now().toSec();
          }
        else
          {
            //reset
            pitch_i_term_ = 0;
            gimbal_pitch_control_stamp_ = 0;
          }


        roll_pitch_pid_pub_.publish(pid_msg);

        Eigen::VectorXd pid_values(5);
        /* F = P# * [roll_pid, pitch_pid, yaw_pid, x_pid, y_pid] */
        pid_values << target_gimbal_roll, target_gimbal_pitch, yaw_control_terms_[0], target_pitch_ - (pitch_angle * acc_z), (-target_roll_) - (-roll_angle * acc_z);
        f_xy = pseudoinverse(P) * pid_values;
      }

    if(control_verbose_)
      {
        std::cout << "gimbal P_pseudo_inverse:"  << std::endl << pseudoinverse(P) << std::endl;
        std::cout << "gimbal force for horizontal control:"  << std::endl << f_xy << std::endl;
      }

    /* external wrench compensation */
    Eigen::MatrixXd cog_rot_inv = aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(roll_angle, pitch_angle, yaw_angle).Inverse());
    Eigen::MatrixXd extended_cog_rot_inv = Eigen::MatrixXd::Zero(6, 6);
    extended_cog_rot_inv.topLeftCorner(3,3) = cog_rot_inv;
    extended_cog_rot_inv.bottomRightCorner(3,3) = cog_rot_inv;
    std::map<std::string, DragonRobotModel::ExternalWrench> external_wrench_map = kinematics_->getExternalWrenchMap();
    for(auto& wrench: external_wrench_map) wrench.second.wrench = extended_cog_rot_inv * wrench.second.wrench;

    kinematics_->calcExternalWrenchCompThrust(external_wrench_map);
    const Eigen::VectorXd& wrench_comp_thrust = kinematics_->getExWrenchCompensateVectoringThrust();
    if(control_verbose_)
      {
        std::cout << "external wrench  compensate vectoring thrust: " << wrench_comp_thrust.transpose() << std::endl;
      }


    std_msgs::Float32MultiArray target_force_msg;

    for(int i = 0; i < rotor_num; i++)
      {
        /* vectoring force */
        tf::Vector3 f_i(f_xy(2 * i) + wrench_comp_thrust(3 * i) + extra_vectoring_force_(3 * i),
                        f_xy(2 * i + 1) + wrench_comp_thrust(3 * i + 1) + extra_vectoring_force_(3 * i + 1),
                        z_control_terms_.at(i) + lqi_att_terms_.at(i) + wrench_comp_thrust(3 * i + 2) + extra_vectoring_force_(3 * i + 2));


        /* calculate ||f||, but omit pitch and roll term, which will be added in spinal */
        target_thrust_terms_.at(i) = (f_i - tf::Vector3(0, 0, lqi_att_terms_.at(i))).length();
        if(control_verbose_)
          ROS_INFO("[gimbal control]: rotor%d, target_thrust vs target_z: [%f vs %f]", i+1, target_thrust_terms_.at(i), z_control_terms_.at(i));

        if(!start_rp_integration_) // in the early stage of takeoff avoiding the large tilt angle,
          f_i.setValue(f_xy(2 * i), f_xy(2 * i + 1), kinematics_->getStaticThrust()[i]);

        tf::Vector3 f_i_normalized = f_i.normalize();
        if(control_verbose_) ROS_INFO("gimbal%d f normaizled: [%f, %f, %f]", i + 1, f_i_normalized.x(), f_i_normalized.y(), f_i_normalized.z());


        /* f -> gimbal angle */
        std::vector<KDL::Rotation> links_frame_from_cog = kinematics_->getLinksRotationFromCog<KDL::Rotation>();

        /* [S_pitch, -S_roll * C_pitch, C_roll * C_roll]^T = R.transpose * f_i_normalized */
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
        target_force_msg.data.push_back(f_xy(2 * i));
        target_force_msg.data.push_back(f_xy(2 * i + 1));
      }
    gimbal_target_force_pub_.publish(target_force_msg);
  }

  void DragonGimbal::baselinkRotationProcess()
  {
    static ros::Time prev_stamp = ros::Time::now();
    if(curr_target_baselink_rot_ == final_target_baselink_rot_) return;

    if(ros::Time::now().toSec() - prev_stamp.toSec() > baselink_rot_pub_interval_)
      {
        if((final_target_baselink_rot_- curr_target_baselink_rot_).length() > baselink_rot_change_thresh_)
          curr_target_baselink_rot_ += ((final_target_baselink_rot_ - curr_target_baselink_rot_).normalize() * baselink_rot_change_thresh_);
        else
          curr_target_baselink_rot_ = final_target_baselink_rot_;

        spinal::DesireCoord target_baselink_rot_msg;
        target_baselink_rot_msg.roll = curr_target_baselink_rot_.x();
        target_baselink_rot_msg.pitch = curr_target_baselink_rot_.y();
        curr_target_baselink_rot_pub_.publish(target_baselink_rot_msg);

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
           2. exclude the attitude part (LQI(roll), LQI(pitch), which will be calculated in spinal.
           ----------------------------------------- */
        flight_command_data.base_throttle[i] = target_thrust_terms_.at(i);
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
    ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(joints_torque_control_srv_name_);
    std_srvs::SetBool srv;

    if(servo_torque_)
      {
        if(navigator_->getNaviState() == Navigator::LAND_STATE)
          {
            if(state_pos_.z() < height_thresh_)
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

  void DragonGimbal::halt()
  {
    ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(joints_torque_control_srv_name_);
    std_srvs::SetBool srv;
    srv.request.data = false;
    if (client.call(srv))
      ROS_INFO("dragon control halt process: disable the joint torque");
    else
      ROS_ERROR("Failed to call service %s", joints_torque_control_srv_name_.c_str());

    client = nh_.serviceClient<std_srvs::SetBool>(gimbals_torque_control_srv_name_);

    srv.request.data = false;
    if (client.call(srv))
      ROS_INFO("dragon control halt process: disable the gimbal torque");
    else
      ROS_ERROR("Failed to call service %s", gimbals_torque_control_srv_name_.c_str());
  }

  void DragonGimbal::jointStateCallback(const sensor_msgs::JointStateConstPtr& state)
  {
    joint_state_ = *state;
    kinematics_->updateRobotModel(joint_state_);

    /* check the gimbal vectoring function */
    if (gimbal_vectoring_check_flag_)
      {
        sensor_msgs::JointState gimbal_control_msg;
        gimbal_control_msg.header = state->header;
        gimbal_control_msg.position = kinematics_->getGimbalNominalAngles();
        gimbal_control_pub_.publish(gimbal_control_msg);
      }
  }

  /* external wrench */
  bool DragonGimbal::addExternalWrenchCallback(gazebo_msgs::ApplyBodyWrench::Request& req, gazebo_msgs::ApplyBodyWrench::Response& res)
  {
    if(kinematics_->addExternalStaticWrench(req.body_name, req.reference_frame, req.reference_point, req.wrench))
      res.success  = true;
    else
      res.success  = false;

    return true;
  }

  bool DragonGimbal::clearExternalWrenchCallback(gazebo_msgs::BodyRequest::Request& req, gazebo_msgs::BodyRequest::Response& res)
  {
    kinematics_->removeExternalStaticWrench(req.body_name);
    return true;
  }

  /* extra vectoring force  */
  void DragonGimbal::extraVectoringForceCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
  {
    if(navigator_->getNaviState() != Navigator::HOVER_STATE || navigator_->getForceLandingFlag() || landing_flag_) return;

    if(extra_vectoring_force_.size() != msg->data.size())
      {
        ROS_ERROR_STREAM("gimbal control: can not assign the extra vectroing force, the size is wrong: " << msg->data.size() << "; reset");
        extra_vectoring_force_.setZero();
        return;
      }

    extra_vectoring_force_ = (Eigen::Map<const Eigen::VectorXf>(msg->data.data(), msg->data.size())).cast<double>();

    ROS_INFO_STREAM("add extra vectoring force is: \n" << extra_vectoring_force_.transpose());
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

    ros::NodeHandle control_nh(nh_, "controller");
    ros::NodeHandle pitch_roll_nh(control_nh, "pitch_roll");

    getParam<bool>(control_nh, "control_verbose", control_verbose_, false);
    getParam<std::string>(control_nh, "joints_torque_control_srv_name", joints_torque_control_srv_name_, std::string("joints/torque_enable"));
    getParam<std::string>(control_nh, "gimbals_torque_control_srv_name", gimbals_torque_control_srv_name_, std::string("gimbals/torque_enable"));
    getParam<double>(control_nh, "height_thresh", height_thresh_, 0.1); // height threshold to disable the joint servo when landing
    getParam<double>(control_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_, 0.02);  // the threshold to change the baselink rotation
    getParam<double>(control_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_, 0.1); // the rate to pub baselink rotation command
    getParam<bool>(control_nh, "gimbal_vectoring_check_flag", gimbal_vectoring_check_flag_, false); // check the gimbal vectoring function without position and yaw control

    /* pitch roll control */
    getParam<double>(pitch_roll_nh, "control_rate_thresh", pitch_roll_control_rate_thresh_, 1.0);
    getParam<double>(pitch_roll_nh, "control_p_det_thresh", pitch_roll_control_p_det_thresh_, 1e-3);
    getParam<double>(pitch_roll_nh, "limit", pitch_roll_limit_, 1.0e6);
    getParam<double>(pitch_roll_nh, "p_term_limit", pitch_roll_terms_limits_[0], 1.0e6);
    getParam<double>(pitch_roll_nh, "i_term_limit", pitch_roll_terms_limits_[1], 1.0e6);
    getParam<double>(pitch_roll_nh, "d_term_limit", pitch_roll_terms_limits_[2], 1.0e6);
    getParam<double>(pitch_roll_nh, "p_gain", pitch_roll_gains_[0], 0.0);
    getParam<double>(pitch_roll_nh, "i_gain", pitch_roll_gains_[1], 0.0);
    getParam<double>(pitch_roll_nh, "d_gain", pitch_roll_gains_[2], 0.0);
    getParam<bool>(control_nh, "add_lqi_result", add_lqi_result_, false);
  }
};

#include <dragon/gimbal_control.h>

using namespace std;

namespace control_plugin
{
  DragonGimbal::DragonGimbal():
    servo_torque_(false), level_flag_(false), landing_flag_(false),
    curr_desire_tilt_(0, 0, 0),
    final_desire_tilt_(0, 0, 0)
  {
  }

  void DragonGimbal::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                           BasicEstimator* estimator, Navigator* navigator,
                           double ctrl_loop_rate)
  {
    /* initialize the flight control */
    FlatnessPid::initialize(nh, nhp, estimator, navigator, ctrl_loop_rate);

    /* initialize the multilink kinematics */
    kinematics_ = boost::shared_ptr<DragonTransformController>(new DragonTransformController(nh_, nhp_, false));

    /* initialize the matrix */
    P_xy_ = Eigen::MatrixXd::Zero(2, kinematics_->getRotorNum() * 2);
    for(int i = 0; i < kinematics_->getRotorNum(); i++)
      {
        P_xy_(0, i * 2) = 1;
        P_xy_(1, 1 + i * 2) = 1;
      }
    //std::cout << "P_xy :"  << std::endl << P_xy_ << std::endl;


    /* initialize the gimbal target angles */
    target_gimbal_angles_.resize(kinematics_->getRotorNum() * 2, 0);

    string pub_name;
    nhp_.param("gimbal_control_topic_name", pub_name, string("gimbals_ctrl"));
    gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>(pub_name, 1);

    nhp_.param("joint_control_topic_name", pub_name, string("joints_ctrl"));
    joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>(pub_name, 1);

    nhp_.param("current_desire_tilt_topic_name", pub_name, string("/desire_coordinate"));
    curr_desire_tilt_pub_ = nh_.advertise<aerial_robot_base::DesireCoord>(pub_name, 1);

    nhp_.param("gimbal_target_force_topic_name", pub_name, string("gimbals_target_force"));
    gimbal_target_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(pub_name, 1);

    string sub_name;
    nhp_.param("joint_state_sub_name", sub_name, std::string("joint_state"));
    joint_state_sub_ = nh_.subscribe(sub_name, 1, &DragonGimbal::jointStateCallback, this);
    nhp_.param("final_desire_tilt_sub_name", sub_name, std::string("/final_desire_tilt"));
    final_desire_tilt_sub_ = nh_.subscribe(sub_name, 1, &DragonGimbal::baselinkTiltCallback, this);

  }

  void DragonGimbal::fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg)
  {
    /* update the motor number */
    if(motor_num_ == 1)
      {
        motor_num_ = msg->motor_num;

        alt_i_term_.resize(motor_num_);
        alt_gains_.resize(motor_num_);

        target_throttle_.resize(motor_num_);

        ROS_WARN("gimbal flight control: update the motor number: %d", motor_num_);
      }

    for(int i = 0; i < msg->motor_num; i++)
      alt_gains_[i].setValue(msg->pos_p_gain_alt[i], msg->pos_i_gain_alt[i], msg->pos_d_gain_alt[i]);
  }

  void DragonGimbal::baselinkTiltCallback(const aerial_robot_base::DesireCoordConstPtr & msg)
  {
    final_desire_tilt_.setValue(msg->roll, msg->pitch, msg->yaw);
  }

  void DragonGimbal::update()
  {
    servoTorqueProcess();
    stateError();

    landingProcess();

    if(!pidUpdate()) return;
    gimbalControl();
    desireTilt();
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
            final_desire_tilt_.setValue(0, 0, 0);
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
        //ROS_INFO("curr_desire_tilt: [%f, %f], nomr: %f", curr_desire_tilt_.x(), curr_desire_tilt_.y(), curr_desire_tilt_.length());
        if(curr_desire_tilt_.length()) already_level = false;

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

    int rotor_num = kinematics_->getRotorNum();

    std::vector<Eigen::Vector3d> rotors_origin_from_cog(rotor_num);
    kinematics_->getRotorsFromCog(rotors_origin_from_cog);
    Eigen::Matrix3d links_inertia = kinematics_->getInertia();

    Eigen::MatrixXd P_att = Eigen::MatrixXd::Zero(3, rotor_num  * 2);
    for(int i = 0; i < rotor_num; i++)
      {
        P_att(0, 2 * i + 1) = -rotors_origin_from_cog[i](2); //x(roll)
        P_att(1, 2 * i) = rotors_origin_from_cog[i](2); //y(pitch)
        P_att(2, 2 * i) = -rotors_origin_from_cog[i](1);
        P_att(2, 2 * i + 1) = rotors_origin_from_cog[i](0);
      }

    Eigen::MatrixXd P_att_tmp = P_att;
    P_att = links_inertia.inverse() * P_att_tmp;


    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(5, rotor_num  * 2);
    P.block(0, 0, 3, rotor_num * 2) = P_att;
    P.block(3, 0, 2, rotor_num * 2) = P_xy_;
    double P_det = (P * P.transpose()).determinant();

    if(control_verbose_)
      {
        std::cout << "gimbal P original :"  << std::endl << P_att_tmp << std::endl;
        std::cout << "gimbal P:"  << std::endl << P << std::endl;
        std::cout << "P det:"  << std::endl << P_det << std::endl;
      }

    Eigen::VectorXd f;

    if(P_det < 1e-3)
      { // bad pitch roll
        if(control_verbose_) ROS_ERROR("bad P_det: %f", P_det);
        P = Eigen::MatrixXd::Zero(3, rotor_num  * 2);
        P.block(0, 0, 1, rotor_num * 2) = P_att.block(2, 0, 1, rotor_num * 2);
        P.block(1, 0, 2, rotor_num * 2) = P_xy_;

        Eigen::VectorXd pid_values(3);
        pid_values << target_yaw_[0], target_pitch_, target_roll_;
        f = pseudoinverse(P) * pid_values;
      }
    else
      {
        Eigen::VectorXd pid_values(5);
        /* F = P# * [0, 0, yaw_pid, x_pid, y_pid] */
        pid_values << 0, 0, target_yaw_[0], target_pitch_, target_roll_;
        f = pseudoinverse(P) * pid_values;
      }

    if(control_verbose_)
      {
        std::cout << "gimbal P_pseudo_inverse:"  << std::endl << pseudoinverse(P) << std::endl;
        std::cout << "gimbal P * P_pseudo_inverse:"  << std::endl << P * pseudoinverse(P) << std::endl;
        //std::cout << "pid values:"  << std::endl << pid_values << std::endl;
        //std::cout << "gimbal f:"  << std::endl << f << std::endl;
      }

    std_msgs::Float32MultiArray target_force_msg;

    for(int i = 0; i < rotor_num; i++)
      {
        /* normalized vector */
        /* 1: use state_x */
        double base_throttle = (target_throttle_[i] > 0)?target_throttle_[i]:1;
        tf::Vector3 f_i = tf::Vector3(f(2 * i), f(2 * i + 1), base_throttle);
        /* 2: use target_throttle */
        //tf::Vector3 f_i = tf::Vector3(f(2 * i), f(2 * i + 1), kinematics_->(getStableState())(i));

        tf::Vector3 f_i_normalized = f_i.normalize();
        if(control_verbose_) ROS_INFO("gimbal%d f normaizled: [%f, %f, %f]", i + 1, f_i_normalized.x(), f_i_normalized.y(), f_i_normalized.z());

        /*f -> gimbal angle */
        std::vector<KDL::Rotation> links_frame_from_cog;
        kinematics_->getLinksOrientation(links_frame_from_cog);

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
        target_force_msg.data.push_back(f(2 * i));
        target_force_msg.data.push_back(f(2 * i + 1));
        gimbal_target_force_pub_.publish(target_force_msg);
      }
  }

  void DragonGimbal::desireTilt()
  {
    static ros::Time prev_stamp = ros::Time::now();
    if(curr_desire_tilt_ == final_desire_tilt_) return;

    //ROS_INFO("current desire_tilt_: [%f, %f, %f], final_desire_tilt_: [%f, %f, %f]", curr_desire_tilt_.x(), curr_desire_tilt_.y(), curr_desire_tilt_.z(), final_desire_tilt_.x(), final_desire_tilt_.y(), final_desire_tilt_.z());

    if(ros::Time::now().toSec() - prev_stamp.toSec() > tilt_pub_interval_)
      {
        if((final_desire_tilt_- curr_desire_tilt_).length() > tilt_thresh_)
          curr_desire_tilt_ += ((final_desire_tilt_ - curr_desire_tilt_).normalize() * tilt_thresh_);
        else
          curr_desire_tilt_ = final_desire_tilt_;

        aerial_robot_base::DesireCoord desire_tilt_msg;
        desire_tilt_msg.roll = curr_desire_tilt_.x();
        desire_tilt_msg.pitch = curr_desire_tilt_.y();
        curr_desire_tilt_pub_.publish(desire_tilt_msg);

        prev_stamp = ros::Time::now();
      }
  }

  void DragonGimbal::sendCmd()
  {
    /* send base throttle command */
    aerial_robot_msgs::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] =  0;
    flight_command_data.angles[1] =  0;

    flight_command_data.base_throttle.resize(motor_num_);
    for(int i = 0; i < motor_num_; i++)
      flight_command_data.base_throttle[i] = target_throttle_[i];
    flight_cmd_pub_.publish(flight_command_data);

    /* send gimbal control command */
    sensor_msgs::JointState gimbal_control_msg;
    gimbal_control_msg.header.stamp = ros::Time::now();
    for(int i = 0; i < kinematics_->getRotorNum() * 2; i++)
      gimbal_control_msg.position.push_back(target_gimbal_angles_[i]);

    gimbal_control_pub_.publish(gimbal_control_msg);
  }

  void DragonGimbal::servoTorqueProcess()
  {
    if(!real_machine_) return;

    ros::ServiceClient client = nh_.serviceClient<dynamixel_controllers::TorqueEnable>(joints_torque_control_srv_name_);
    dynamixel_controllers::TorqueEnable srv;

    if(servo_torque_)
      {
        if(navigator_->getNaviState() == Navigator::LAND_STATE)
          {
            if(state_pos_.z() < height_thresh_ && real_machine_)
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
          }

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
      }
    else
      {
        if(navigator_->getNaviState() == Navigator::ARM_ON_STATE)
          {
            srv.request.torque_enable = true;

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
    kinematics_->gimbalProcess(joint_state_);
    kinematics_->kinematics(joint_state_);
  }

  void DragonGimbal::rosParamInit()
  {
    FlatnessPid::rosParamInit();

    string ns = nhp_.getNamespace();
    nhp_.param("control_verbose", control_verbose_, false);
    if(verbose_) cout << ns << ": control_verbose is " << control_verbose_ << endl;
    nhp_.param("real_machine", real_machine_, true);
    if(verbose_) cout << ns << ": real_machine is " << real_machine_ << endl;
    nhp_.param("joints_torque_control_srv_name", joints_torque_control_srv_name_, std::string("/joints_controller/torque_enable"));

    /* height threshold to disable the joint servo when landing */
    nhp_.param("height_thresh", height_thresh_, 0.1);
    if(verbose_) cout << ns << ": height_thresh is " << height_thresh_ << endl;

    /* the threshold to tilt smoothly */
    nhp_.param("tilt_thresh", tilt_thresh_, 0.02);
    if(verbose_) cout << ns << ": tilt_thresh is " << tilt_thresh_ << endl;

    /* the rate to pub target tilt  command */
    nhp_.param("tilt_pub_interval", tilt_pub_interval_, 0.1);
    if(verbose_) cout << ns << ": tilt_pub_interval is " << tilt_pub_interval_ << endl;
  }
};

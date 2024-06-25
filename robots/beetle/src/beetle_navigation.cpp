// -*- mode: c++ -*-

#include <beetle/beetle_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

BeetleNavigator::BeetleNavigator():
  GimbalrotorNavigator(),
  roll_pitch_control_flag_(false),
  pre_assembled_(false)
{
}

void BeetleNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                 double loop_du)
{
  GimbalrotorNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);
  nh_ = nh;
  nhp_ = nhp;
  beetle_robot_model_ = boost::dynamic_pointer_cast<BeetleRobotModel>(robot_model);
  int max_modules_num = beetle_robot_model_->getMaxModuleNum();
  assembly_nav_sub_ = nh_.subscribe("/assembly/uav/nav", 1, &BeetleNavigator::assemblyNavCallback, this);
  assembly_target_rot_sub_ = nh_.subscribe("/assembly/final_target_baselink_rot", 1, &BeetleNavigator::setAssemblyFinalTargetBaselinkRotCallback, this);
  for(int i = 0; i < max_modules_num; i++){
    std::string module_name  = string("/beetle") + std::to_string(i+1);
    assembly_flag_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/assembly_flag"), 1, &BeetleNavigator::assemblyFlagCallback, this)));
  }
  
}

void BeetleNavigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  sensor_msgs::Joy joy_cmd;
  if(joy_msg->axes.size() == PS3_AXES && joy_msg->buttons.size() == PS3_BUTTONS)
    {
      joy_cmd = (*joy_msg);
    }
  else if(joy_msg->axes.size() == PS4_AXES && joy_msg->buttons.size() == PS4_BUTTONS)
    {
      joy_cmd = ps4joyToPs3joyConvert(*joy_msg);
    }
  else
    {
      ROS_WARN("the joystick type is not supported (buttons: %d, axes: %d)", (int)joy_msg->buttons.size(), (int)joy_msg->axes.size());
      return;
    }

  /* Motion: Roll and Pitch*/
  /* this is the roll and pitch_angle control */
   if(joy_cmd.buttons[PS3_BUTTON_REAR_LEFT_1] == 1)
    {
      tf::Vector3 target_roll_pitch;
      // create another command to control roll direction
      // target_roll_pitch.setX(getFinalTargetBaselinkRot().x()
      //                        + joy_cmd.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] * max_target_roll_pitch_rate_                     
      //                        );
        
      target_roll_pitch.setY(getFinalTargetBaselinkRot().y()
                             + joy_cmd.axes[PS3_AXIS_STICK_LEFT_UPWARDS] * max_target_roll_pitch_rate_
                             );

      setFinalTargetBaselinkRot(target_roll_pitch);
      roll_pitch_control_flag_ = true;
      joy_rotation_flag_ = true;
    }
  else
    {
      if(roll_pitch_control_flag_)
        {
          roll_pitch_control_flag_= false;
          ROS_INFO("Joy Control: fixed roll pich state, target roll angle is %f, pitch angle is %f", getFinalTargetBaselinkRot().x(),getFinalTargetBaselinkRot().y());
        }
    }
  if(joy_cmd.buttons[PS3_BUTTON_REAR_LEFT_1] == 1 && joy_cmd.buttons[PS3_BUTTON_REAR_RIGHT_1] == 1)
    {
      tf::Vector3 target_roll_pitch;
      target_roll_pitch.setX(0.0);
      target_roll_pitch.setY(0.0);
      setFinalTargetBaselinkRot(target_roll_pitch);
    }

  BaseNavigator::joyStickControl(joy_msg);
  joy_rotation_flag_ = false;
}

void BeetleNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)
{  if(getNaviState() == TAKEOFF_STATE || BaseNavigator::getNaviState() == LAND_STATE) return;

  gps_waypoint_ = false;

  if(force_att_control_flag_) return;

  /* yaw */
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      setTargetYaw(angles::normalize_angle(msg->target_yaw));
      setTargetOmegaZ(0);
    }
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::VEL_MODE)
    {
      setTargetOmegaZ(msg->target_omega_z);

      teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();
    }
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      setTargetYaw(angles::normalize_angle(msg->target_yaw));
      setTargetOmegaZ(msg->target_omega_z);
      trajectory_mode_ = true;
      trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();
    }

  /* xy control */
  switch(msg->pos_xy_nav_mode)
    {
    case aerial_robot_msgs::FlightNav::POS_MODE:
      {
        tf::Vector3 target_cog_pos(msg->target_pos_x, msg->target_pos_y, 0);
        if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
          {
            /* check the transformation */
            tf::Transform cog2baselink_tf;
            tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
            target_cog_pos -= tf::Matrix3x3(tf::createQuaternionFromYaw(getTargetRPY().z()))
              * cog2baselink_tf.getOrigin();
          }
        else if(msg->target == CONTACT_POINT)
          {
            /* check the transformation */
            tf::Transform cog2cp_tf;
            tf::transformKDLToTF(beetle_robot_model_->getCog2Cp<KDL::Frame>(), cog2cp_tf);
            target_cog_pos -= cog2cp_tf.getOrigin();
          }

        tf::Vector3 target_delta = getTargetPos() - target_cog_pos;
        target_delta.setZ(0);

        if(target_delta.length() > vel_nav_threshold_)
          {
            ROS_WARN("start vel nav control for waypoint");
            vel_based_waypoint_ = true;
            xy_control_mode_ = VEL_CONTROL_MODE;
          }

        if(!vel_based_waypoint_)
          xy_control_mode_ = POS_CONTROL_MODE;

        setTargetPosCandX(target_cog_pos.x());
        setTargetPosCandY(target_cog_pos.y());
        setTargetVelX(0);
        setTargetVelY(0);

        break;
      }
    case aerial_robot_msgs::FlightNav::VEL_MODE:
      {
        /* do not switch to pure vel mode */
        xy_control_mode_ = POS_CONTROL_MODE;

        teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();

        switch(msg->control_frame)
          {
          case WORLD_FRAME:
            {
              setTargetVelX(msg->target_vel_x);
              setTargetVelY(msg->target_vel_y);
              break;
            }
          case LOCAL_FRAME:
            {
              double yaw_angle = estimator_->getState(State::YAW_COG, estimate_mode_)[0];
              tf::Vector3 target_vel = frameConversion(tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0), yaw_angle);
              setTargetVelX(target_vel.x());
              setTargetVelY(target_vel.y());
              break;
            }
          default:
            {
              break;
            }
          }
        break;
      }
    case aerial_robot_msgs::FlightNav::POS_VEL_MODE:
      {
        xy_control_mode_ = POS_CONTROL_MODE;
        setTargetPosCandX(msg->target_pos_x);
        setTargetPosCandY(msg->target_pos_y);
        setTargetVelX(msg->target_vel_x);
        setTargetVelY(msg->target_vel_y);

        trajectory_mode_ = true;
        trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();

        break;
      }
    case aerial_robot_msgs::FlightNav::ACC_MODE:
      {
        /* should be in COG frame */
        xy_control_mode_ = ACC_CONTROL_MODE;
        prev_xy_control_mode_ = ACC_CONTROL_MODE;

        switch(msg->control_frame)
          {
          case WORLD_FRAME:
            {
              target_acc_.setValue(msg->target_acc_x, msg->target_acc_y, 0);
              break;
            }
          case LOCAL_FRAME:
            {
              tf::Vector3 target_acc = frameConversion(tf::Vector3(msg->target_acc_x, msg->target_acc_y, 0), estimator_->getState(State::YAW_COG, estimate_mode_)[0]);
              setTargetAccX(target_acc.x());
              setTargetAccY(target_acc.y());
              break;
            }
          default:
            {
              break;
            }
          }
        break;
      }
    case aerial_robot_msgs::FlightNav::STAY_HERE_MODE:
      {
        xy_control_mode_ = POS_CONTROL_MODE;
        setTargetVelX(0);
        setTargetVelY(0);
        setTargetXyFromCurrentState();
        break;
      }      
    case aerial_robot_msgs::FlightNav::GPS_WAYPOINT_MODE:
      {
        target_wp_ = geodesy::toMsg(msg->target_pos_x, msg->target_pos_y);
        gps_waypoint_ = true;

        break;
      }
    }
  if(msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::ACC_MODE) target_acc_.setValue(0,0,0);

  /* z */
  if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::VEL_MODE)
    {
      /* special */
      addTargetPosZ(msg->target_pos_diff_z);
      setTargetVelZ(0);
    }
  else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      tf::Vector3 target_cog_pos(0, 0, msg->target_pos_z);
      if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
        {
          /* check the transformation */
          tf::Transform cog2baselink_tf;
          tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
          target_cog_pos -= cog2baselink_tf.getOrigin();
        }
      else if(msg->target == CONTACT_POINT)
        {
          /* check the transformation */
          tf::Transform cog2cp_tf;
          tf::transformKDLToTF(beetle_robot_model_->getCog2Cp<KDL::Frame>(), cog2cp_tf);
          target_cog_pos -= cog2cp_tf.getOrigin();
        }

      setTargetPosCandZ(target_cog_pos.z());
      setTargetVelZ(0);      

    }
  else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      setTargetPosCandZ(msg->target_pos_z);
      setTargetVelZ(msg->target_vel_z);
    }


}

void BeetleNavigator::assemblyNavCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)
{
  if(beetle_robot_model_->getModuleState() != SEPARATED) BeetleNavigator::naviCallback(msg);
}

void BeetleNavigator::setAssemblyFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg)
{
  if(beetle_robot_model_->getModuleState() != SEPARATED) GimbalrotorNavigator::setFinalTargetBaselinkRotCallback(msg);
}


void BeetleNavigator::assemblyFlagCallback(const diagnostic_msgs::KeyValue & msg)
{
  int module_id = std::stoi(msg.key);
  int assembly_flag = std::stoi(msg.value);
  beetle_robot_model_->setAssemblyFlag(module_id,assembly_flag);
}

void BeetleNavigator::update()
{
  rotateContactPointFrame();
  beetle_robot_model_->calcCenterOfMoving();
  convertTargetPosFromCoG2CoM();
  GimbalrotorNavigator::update();
  beetle_robot_model_->setControlFlag((getNaviState() == HOVER_STATE || getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE) ? true : false);
}

void BeetleNavigator::rotateContactPointFrame()
{
  geometry_msgs::TransformStamped tf = beetle_robot_model_-> getContactFrame<geometry_msgs::TransformStamped>();
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = tf::resolve(std::string(nh_.getNamespace()), beetle_robot_model_->getRootFrameName());
  tf.child_frame_id = tf::resolve(std::string(nh_.getNamespace()), std::string("contact_point"));
  br_.sendTransform(tf); 
}

void BeetleNavigator::convertTargetPosFromCoG2CoM()
{
  //TODO: considering correct rotaion axis

  tf::Transform cog2com_tf;
  tf::transformKDLToTF(beetle_robot_model_->getCog2CoM<KDL::Frame>(), cog2com_tf);
  tf::Matrix3x3 cog_orientation_tf;
  tf::matrixEigenToTF(beetle_robot_model_->getCogDesireOrientation<Eigen::Matrix3d>(),cog_orientation_tf);
  tf::Vector3 com_conversion = cog_orientation_tf *  tf::Matrix3x3(tf::createQuaternionFromYaw(getTargetRPY().z())) * cog2com_tf.getOrigin();
  bool current_assembled = beetle_robot_model_->getCurrentAssembled();
  bool reconfig_flag = beetle_robot_model_->getReconfigFlag();

  KDL::Frame empty_frame;

  if(pre_assembled_  && !current_assembled){ //disassembly process
    setTargetPosCandX(getTargetPos().x());
    setTargetPosCandY(getTargetPos().y());
    setTargetPosCandZ(getTargetPos().z());
    ROS_INFO("switched");
    pre_assembled_ = current_assembled;
  } else if((!pre_assembled_  && current_assembled) || (current_assembled && reconfig_flag)){ //assembly or reconfig process
    int my_id = beetle_robot_model_->getMyID();
    tf::Vector3 pos_cog = estimator_->getPos(Frame::COG, estimate_mode_);
    tf::Vector3 orientation_err = getTargetRPY() - estimator_ ->getEuler(Frame::COG, estimate_mode_);
    ROS_INFO_STREAM("ID: " << my_id << "'s orientation_err is "<< "(" << orientation_err.x() << ", " << orientation_err.y() << ", " << orientation_err.z() << ")");
    tf::Matrix3x3 att_err_mat = tf::Matrix3x3(tf::createQuaternionFromRPY(orientation_err.x(), orientation_err.y(),orientation_err.z()));
    tf::Vector3 corrected_target_pos =  tf::Matrix3x3(tf::createQuaternionFromRPY(orientation_err.x(), orientation_err.y(),orientation_err.z())) * pos_cog;
    if(getNaviState() == HOVER_STATE){
      setTargetPosCandX(pos_cog.x() + (att_err_mat.inverse() * com_conversion).x());
      setTargetPosCandY(pos_cog.y() + (att_err_mat.inverse() * com_conversion).y());
      setTargetPosCandZ(pos_cog.z() + (att_err_mat.inverse() * com_conversion).z());
    }
    ROS_INFO("switched");
    pre_assembled_ = current_assembled;
  }else if(beetle_robot_model_->getCog2CoM<KDL::Frame>() == empty_frame && getNaviState() != HOVER_STATE){
    return;
  }


  /* Check whether the target value was changed by someway other than uav nav */
  /* Target pos candidate represents a target pos in a assembly frame */
  if( int(pre_target_pos_.x() * 1000) != int(getTargetPos().x() * 1000)){
    float target_x_com = getTargetPos().x() + com_conversion.x();
    setTargetPosCandX(target_x_com);
  }

  if( int(pre_target_pos_.y() * 1000) != int(getTargetPos().y() * 1000)){
    float target_y_com = getTargetPos().y() + com_conversion.y();
    setTargetPosCandY(target_y_com);
  }

  if( int(pre_target_pos_.z() * 1000) != int(getTargetPos().z() * 1000)){
    float target_z_com = getTargetPos().z() + com_conversion.z();
    setTargetPosCandZ(target_z_com);
  }
  
  tf::Vector3 target_cog_pos = getTargetPosCand();
  target_cog_pos -=  com_conversion;

  if( getNaviState() == HOVER_STATE ||
      getNaviState() == TAKEOFF_STATE){
    setTargetPosX(target_cog_pos.x());
    setTargetPosY(target_cog_pos.y());
    setTargetPosZ(target_cog_pos.z());
  }

  pre_target_pos_.setX(target_cog_pos.x());
  pre_target_pos_.setY(target_cog_pos.y());
  pre_target_pos_.setZ(target_cog_pos.z());
}

void BeetleNavigator::rosParamInit()
{
  ros::NodeHandle nh(nh_, "navigation");
  getParam<double>(nh, "max_target_roll_pitch_rate", max_target_roll_pitch_rate_, 0.0);
  GimbalrotorNavigator::rosParamInit();
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::BeetleNavigator, aerial_robot_navigation::BaseNavigator);

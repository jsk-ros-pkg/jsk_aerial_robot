// -*- mode: c++ -*-

#include <beetle/beetle_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

BeetleNavigator::BeetleNavigator():
  GimbalrotorNavigator(),
  roll_pitch_control_flag_(false),
  pre_assembled_(false),
  current_assembled_(false),
  module_state_(SEPARATED),
  leader_fix_flag_(false),
  tfBuffer_(),
  tfListener_(tfBuffer_),
  pre_assembled_modules_(0),
  my_index_(0)
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
  cog_com_dist_pub_ = nh_.advertise<geometry_msgs::Point>("cog_com_dist", 1);
  assembly_nav_sub_ = nh_.subscribe("/assembly/uav/nav", 1, &BeetleNavigator::assemblyNavCallback, this);
  assembly_target_rot_sub_ = nh_.subscribe("/assembly/final_target_baselink_rot", 1, &BeetleNavigator::setAssemblyFinalTargetBaselinkRotCallback, this);

  beetle_robot_model_ = boost::dynamic_pointer_cast<BeetleRobotModel>(robot_model);

  for(int i = 0; i < max_modules_num_; i++){
    std::string module_name  = string("/") + getMyName() + std::to_string(i+1);
    assembly_flag_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/assembly_flag"), 1, &BeetleNavigator::assemblyFlagCallback, this)));
    assembly_flags_.insert(std::make_pair(i+1,false));
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
{
  if(getNaviState() == TAKEOFF_STATE || BaseNavigator::getNaviState() == LAND_STATE) return;

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

        setTargetPosX(target_cog_pos.x());
        setTargetPosY(target_cog_pos.y());

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

        setTargetPosX(target_cog_pos.x());
        setTargetPosY(target_cog_pos.y());            

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
              setTargetAccX(msg->target_acc_x);
              setTargetAccY(msg->target_acc_y);
              break;
            }
          case LOCAL_FRAME:
            {
              double yaw_angle = estimator_->getState(State::YAW_COG, estimate_mode_)[0];
              tf::Vector3 target_acc = frameConversion(tf::Vector3(msg->target_acc_x, msg->target_acc_y, 0), yaw_angle);
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
  if(msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::ACC_MODE) setTargetZeroAcc();

  /* z */
  if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::VEL_MODE)
    {
      setTargetVelZ(msg->target_vel_z);
      teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();
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

      setTargetPosZ(target_cog_pos.z());

      setTargetVelZ(0);
    }
  else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
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

      trajectory_mode_ = true;
      trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();

      setTargetPosZ(target_cog_pos.z());
      setTargetVelZ(msg->target_vel_z);
    }
}


void BeetleNavigator::assemblyNavCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)
{
  if(getNaviState() == TAKEOFF_STATE || BaseNavigator::getNaviState() == LAND_STATE || getModuleState() == SEPARATED) return;

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
            ROS_ERROR("[Nav] Only cog frame is available during assembled state!");
            return;

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
          ROS_ERROR("[Nav] Only CoG can be set as a target frame during assembled state!");
          return;

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


void BeetleNavigator::setAssemblyFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg)
{
  if(getModuleState() != SEPARATED) GimbalrotorNavigator::setFinalTargetBaselinkRotCallback(msg);
}


void BeetleNavigator::assemblyFlagCallback(const diagnostic_msgs::KeyValue & msg)
{
  int module_id = std::stoi(msg.key);
  int assembly_flag = std::stoi(msg.value);
  setAssemblyFlag(module_id,assembly_flag);
}

void BeetleNavigator::update()
{
  rotateContactPointFrame();
  calcCenterOfMoving();
  GimbalrotorNavigator::update();
  setControlFlag((getNaviState() == HOVER_STATE || getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE) ? true : false);
  convertTargetPosFromCoG2CoM();
}

void BeetleNavigator::rotateContactPointFrame()
{
  geometry_msgs::TransformStamped tf = beetle_robot_model_-> getContactFrame<geometry_msgs::TransformStamped>();
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = tf::resolve(std::string(nh_.getNamespace()), beetle_robot_model_->getRootFrameName());
  tf.child_frame_id = tf::resolve(std::string(nh_.getNamespace()), std::string("contact_point"));
  br_.sendTransform(tf); 
}

void BeetleNavigator::calcCenterOfMoving()
{
  std::string cog_name = my_name_ + std::to_string(my_id_) + "/cog";
  Eigen::Vector3f center_of_moving = Eigen::Vector3f::Zero();
  int assembled_module = 0;
  geometry_msgs::Point cog_com_dist_msg;
  assembled_modules_ids_.clear();
  for(const auto & item : assembly_flags_){
    geometry_msgs::TransformStamped transformStamped;
    int id = item.first;
    bool value = item.second;
    if(!value) continue;
    try
      {
        transformStamped = tfBuffer_.lookupTransform(cog_name, my_name_ + std::to_string(id) + std::string("/cog") , ros::Time(0));
        auto& trans = transformStamped.transform.translation;
        Eigen::Vector3f module_root(trans.x,trans.y,trans.z); 
        center_of_moving += module_root;
        assembled_module ++;
        assembled_modules_ids_.push_back(id);
      }
    catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM("not exist module is mentioned. ID is "<<id );
        return;
      }
  }
  setModuleNum(assembled_module);
  if(!assembled_module || assembled_module == 1 || !assembly_flags_[my_id_]){
    pre_assembled_modules_ = assembled_module;
    KDL::Frame com_frame;
    setCog2CoM(com_frame);
    current_assembled_ = false;
    module_state_ = SEPARATED;
    cog_com_dist_msg.x = Cog2CoM_.p.x();
    cog_com_dist_msg.y = Cog2CoM_.p.y();
    cog_com_dist_msg.z = Cog2CoM_.p.z();
    cog_com_dist_pub_.publish(cog_com_dist_msg);
    return;
  }

  //define a module closest to the center as leader
  std::sort(assembled_modules_ids_.begin(), assembled_modules_ids_.end());
  int leader_index = std::round((assembled_modules_ids_.size())/2.0) -1;
  if(!leader_fix_flag_) leader_id_ = assembled_modules_ids_[leader_index];
  if(my_id_ == leader_id_ && control_flag_){
    module_state_ = LEADER;
  }else if(control_flag_){
    module_state_ = FOLLOWER;
  }


  center_of_moving = center_of_moving / assembled_module;

  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = cog_name;
  tf.child_frame_id = my_name_ + std::to_string(my_id_)+"/center_of_moving";
  tf.transform.translation.x = center_of_moving.x();
  tf.transform.translation.y = center_of_moving.y();
  tf.transform.translation.z = center_of_moving.z();
  tf.transform.rotation.x = 0;
  tf.transform.rotation.y = 0;
  tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;
  br_.sendTransform(tf);

  //update com-cog distance only during hovering
  if(control_flag_){
    Eigen::Vector3f cog_com_dist(center_of_moving.norm() * center_of_moving.x()/fabs(center_of_moving.x()),0,0);
    // ROS_INFO_STREAM("cog_com_dist is " << cog_com_dist.transpose());
    tf.transform.translation.x = cog_com_dist.x();
    tf.transform.translation.y = cog_com_dist.y();
    tf.transform.translation.z = cog_com_dist.z();
    setCog2CoM(tf2::transformToKDL(tf));
    reconfig_flag_ =  (pre_assembled_modules_ != assembled_module) ? true : false;
    if(reconfig_flag_){
      pre_assembled_modules_ = assembled_module;
      Eigen::VectorXi id_vector = Eigen::Map<Eigen::VectorXi>(assembled_modules_ids_.data(), assembled_modules_ids_.size());
      for(const auto & item : assembly_flags_){
        if(item.second)
          {
            std::cout << "id: " << item.first << " -> assembled"<< std::endl;
          } else {
          std::cout << "id: " << item.first << " -> separated"<< std::endl;
        }
      }
      ROS_INFO_STREAM(id_vector);
      ROS_INFO_STREAM("Leader's ID is " <<leader_id_);
    }
  }
  cog_com_dist_msg.x = Cog2CoM_.p.x();
  cog_com_dist_msg.y = Cog2CoM_.p.y();
  cog_com_dist_msg.z = Cog2CoM_.p.z();
  cog_com_dist_pub_.publish(cog_com_dist_msg);
  if(control_flag_) current_assembled_ = true;
}


void BeetleNavigator::convertTargetPosFromCoG2CoM()
{
  //TODO: considering correct rotaion axis

  tf::Transform cog2com_tf;
  tf::transformKDLToTF(getCog2CoM<KDL::Frame>(), cog2com_tf);
  tf::Matrix3x3 cog_orientation_tf;
  tf::matrixEigenToTF(beetle_robot_model_->getCogDesireOrientation<Eigen::Matrix3d>(),cog_orientation_tf);
  tf::Vector3 com_conversion = cog_orientation_tf *  tf::Matrix3x3(tf::createQuaternionFromYaw(getTargetRPY().z())) * cog2com_tf.getOrigin();
  bool current_assembled = getCurrentAssembled();
  bool reconfig_flag = getReconfigFlag();

  KDL::Frame empty_frame;

  if(pre_assembled_  && !current_assembled){ //disassembly process
    setTargetPosCandX(getTargetPos().x());
    setTargetPosCandY(getTargetPos().y());
    setTargetPosCandZ(getTargetPos().z());
    ROS_INFO("switched");
    pre_assembled_ = current_assembled;
  } else if((!pre_assembled_  && current_assembled) || (current_assembled && reconfig_flag)){ //assembly or reconfig process
    int my_id = getMyID();
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
  }else if(getCog2CoM<KDL::Frame>() == empty_frame && getNaviState() != HOVER_STATE){
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

  nh_.getParam("robot_id", my_id_);
  nh_.getParam("aerial_robot_base_node/tf_prefix", my_name_);
  my_name_.pop_back(); // extract common robot name
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::BeetleNavigator, aerial_robot_navigation::BaseNavigator);

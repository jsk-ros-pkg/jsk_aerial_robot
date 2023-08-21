// -*- mode: c++ -*-

#include <beetle/beetle_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

BeetleNavigator::BeetleNavigator():
  GimbalrotorNavigator()
{
}

void BeetleNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  GimbalrotorNavigator::initialize(nh, nhp, robot_model, estimator);
  nh_ = nh;
  nhp_ = nhp;
  beetle_robot_model_ = boost::dynamic_pointer_cast<BeetleRobotModel>(robot_model);
  int max_modules_num = beetle_robot_model_->getMaxModuleNum();
  for(int i = 0; i < max_modules_num; i++){
    std::string module_name  = string("/beetle") + std::to_string(i+1);
    assembly_flag_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/assembly_flag"), 1, &BeetleNavigator::assemblyFlagCallback, this)));
  }
}
void BeetleNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)
{  if(getNaviState() == TAKEOFF_STATE || BaseNavigator::getNaviState() == LAND_STATE) return;

  gps_waypoint_ = false;

  if(force_att_control_flag_) return;

  /* yaw */
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      setTargetYaw(angles::normalize_angle(msg->target_yaw));
      setTargetOmageZ(0);
    }
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      setTargetYaw(angles::normalize_angle(msg->target_yaw));
      setTargetOmageZ(msg->target_omega_z);
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
        if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
          {
            ROS_ERROR("[Flight nav] can not do vel nav for baselink");
            return;
          }
        /* should be in COG frame */
        xy_control_mode_ = VEL_CONTROL_MODE;
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
              tf::Vector3 target_vel = frameConversion(tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0),  estimator_->getState(State::YAW_COG, estimate_mode_)[0]);
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
        if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
          {
            ROS_ERROR("[Flight nav] can not do pos_vel nav for baselink");
            return;
          }

        xy_control_mode_ = POS_CONTROL_MODE;
        setTargetPosX(msg->target_pos_x);
        setTargetPosY(msg->target_pos_y);
        setTargetVelX(msg->target_vel_x);
        setTargetVelY(msg->target_vel_y);

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

      setTargetPosZ(target_cog_pos.z());
      setTargetVelZ(0);      

    }
  else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      setTargetPosZ(msg->target_pos_z);
      setTargetVelZ(msg->target_vel_z);
    }

}

 void BeetleNavigator::assemblyFlagCallback(const diagnostic_msgs::KeyValue & msg)
 {
   int module_id = std::stoi(msg.key);
   int assembly_flag = std::stoi(msg.value);
   beetle_robot_model_->setAssemblyFlag(module_id,assembly_flag);
   map<int, bool> flags = beetle_robot_model_->getAssemblyFlags();
   for(const auto & item : flags){
     if(item.second)
       {
         std::cout << "id: " << item.first << " -> assembled"<< std::endl;
       } else {
         std::cout << "id: " << item.first << " -> separated"<< std::endl;
       }
   }
   
 }

void BeetleNavigator::update()
{
  rotateContactPointFrame();
  GimbalrotorNavigator::update();
}

void BeetleNavigator::rotateContactPointFrame()
{
  geometry_msgs::TransformStamped tf = beetle_robot_model_-> getContactFrame<geometry_msgs::TransformStamped>();
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = tf::resolve(std::string(nh_.getNamespace()), beetle_robot_model_->getRootFrameName());
  tf.child_frame_id = tf::resolve(std::string(nh_.getNamespace()), std::string("contact_point"));
  br_.sendTransform(tf); 
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::BeetleNavigator, aerial_robot_navigation::BaseNavigator);

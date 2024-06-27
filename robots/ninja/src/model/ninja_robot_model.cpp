#include <ninja/model/ninja_robot_model.h>

NinjaRobotModel::NinjaRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  BeetleRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  
}

void NinjaRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  GimbalrotorRobotModel::updateRobotModelImpl(joint_positions);
}

void NinjaRobotModel::calcCenterOfMoving()
{
  /*Update assembled modules*/
  int assembled_module = 0;
  assembled_modules_ids_.clear();
  for(const auto & item : assembly_flags_){
    int id = item.first;
    bool value = item.second;
    if(!value) continue;
    assembled_module ++;
    assembled_modules_ids_.push_back(id);
  }
  setModuleNum(assembled_module);

  /*Check state*/
  geometry_msgs::Point cog_com_dist_msg;
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

  /*Define a module closest to the center as leader*/
  std::sort(assembled_modules_ids_.begin(), assembled_modules_ids_.end());
  int leader_index = std::round((assembled_modules_ids_.size())/2.0) -1;
  if(!leader_fix_flag_) leader_id_ = assembled_modules_ids_[leader_index];
  if(my_id_ == leader_id_ && control_flag_){
    module_state_ = LEADER;
  }else if(control_flag_){
    module_state_ = FOLLOWER;
  }

  /* Use leader's CoG as CoM*/
  Eigen::Vector3f center_of_moving_trans = Eigen::Vector3f::Zero();
  Eigen::VectorXf center_of_moving_rot(4);
  std::string cog_name = my_name_ + std::to_string(my_id_) + "/cog";
  try
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped = tfBuffer_.lookupTransform(cog_name, my_name_ + std::to_string(leader_id_) + std::string("/cog") , ros::Time(0));
      /*Translation*/
      auto& trans = transformStamped.transform.translation;
      Eigen::Vector3f module_root_trans(trans.x,trans.y,trans.z);
      center_of_moving_trans = module_root_trans;
      /*Rotation*/
      auto& rot = transformStamped.transform.rotation;
      center_of_moving_rot << rot.x, rot.y, rot.z, rot.w;
    }
  catch (tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM("not exist module is mentioned. ID is "<<leader_id_ );
      return;
    }

  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = cog_name;
  tf.child_frame_id = my_name_ + std::to_string(my_id_)+"/center_of_moving";
  tf.transform.translation.x = center_of_moving_trans.x();
  tf.transform.translation.y = center_of_moving_trans.y();
  tf.transform.translation.z = center_of_moving_trans.z();
  tf.transform.rotation.x = center_of_moving_rot(0);
  tf.transform.rotation.y = center_of_moving_rot(1);
  tf.transform.rotation.z = center_of_moving_rot(2);
  tf.transform.rotation.w = center_of_moving_rot(3);
  br_.sendTransform(tf);

  /*Update com-cog distance only during hovering */
  if(control_flag_){
    Eigen::Vector3f cog_com_dist(center_of_moving_trans.norm() * center_of_moving_trans.x()/fabs(center_of_moving_trans.x()),0,0);
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

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NinjaRobotModel, aerial_robot_model::RobotModel);

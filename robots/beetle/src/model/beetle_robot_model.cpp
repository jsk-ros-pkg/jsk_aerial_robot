#include <beetle/model/beetle_robot_model.h>

BeetleRobotModel::BeetleRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  GimbalrotorRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon),
  tfBuffer_(),
  tfListener_(tfBuffer_),
  current_assembled_(false),
  module_state_(SEPARATED)
{
  for(int i = 0; i < max_modules_num_; i++){
    assembly_flags_.insert(std::make_pair(i+1,false));
  }
  nh_.getParam("robot_id", my_id_);
  cog_com_dist_pub_ = nh_.advertise<geometry_msgs::Point>("cog_com_dist", 1);
}

void BeetleRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  GimbalrotorRobotModel::updateRobotModelImpl(joint_positions);

  const auto seg_tf_map = getSegmentsTf();
  KDL::Frame fix_cp_frame = seg_tf_map.at("contact_point");
  KDL::Frame variable_cp_frame;
  variable_cp_frame.M = fix_cp_frame.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  variable_cp_frame.p = fix_cp_frame.p;
  setContactFrame(variable_cp_frame);
  KDL::Frame Cog2Cp;
  setCog2Cp(getCog<KDL::Frame>().Inverse() * variable_cp_frame);

  // calcCenterOfMoving();
}

void BeetleRobotModel::calcCenterOfMoving()
{
  std::string cog_name = "beetle" + std::to_string(my_id_) + "/cog";
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
        transformStamped = tfBuffer_.lookupTransform(cog_name, "beetle" + std::to_string(id) + std::string("/cog") , ros::Time(0));
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
  leader_id_ = assembled_modules_ids_[leader_index];
  if(my_id_ == leader_id_ && control_flag_){
    module_state_ = LEADER;
  }else if(control_flag_){
    module_state_ = FOLLOWER;
  }


  center_of_moving = center_of_moving / assembled_module;

  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = cog_name;
  tf.child_frame_id = "beelte" + std::to_string(my_id_)+"/center_of_moving";
  tf.transform.translation.x = center_of_moving.x();
  tf.transform.translation.y = center_of_moving.y();
  tf.transform.translation.z = center_of_moving.z();
  tf.transform.rotation.x = 0;
  tf.transform.rotation.y = 0;
  tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;
  br_.sendTransform(tf);

  //update com-cog distance only during hovering
  reconfig_flag_ =  (pre_assembled_modules_ != assembled_module) ? true : false;
  if(reconfig_flag_ && control_flag_){
    Eigen::Vector3f cog_com_dist(center_of_moving.norm() * center_of_moving.x()/fabs(center_of_moving.x()),0,0);
    ROS_INFO_STREAM("cog_com_dist is " << cog_com_dist.transpose());
    tf.transform.translation.x = cog_com_dist.x();
    tf.transform.translation.y = cog_com_dist.y();
    tf.transform.translation.z = cog_com_dist.z();
    setCog2CoM(tf2::transformToKDL(tf));
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
    cog_com_dist_msg.x = Cog2CoM_.p.x();
    cog_com_dist_msg.y = Cog2CoM_.p.y();
    cog_com_dist_msg.z = Cog2CoM_.p.z();
    cog_com_dist_pub_.publish(cog_com_dist_msg);
    if(control_flag_) current_assembled_ = true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(BeetleRobotModel, aerial_robot_model::RobotModel);

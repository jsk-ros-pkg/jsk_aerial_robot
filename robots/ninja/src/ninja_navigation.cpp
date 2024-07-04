// -*- mode: c++ -*-

#include <ninja/ninja_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

NinjaNavigator::NinjaNavigator():
  BeetleNavigator()
{
}

void NinjaNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                double loop_du)
{
  BeetleNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);
  ninja_robot_model_ = boost::dynamic_pointer_cast<NinjaRobotModel>(robot_model);
  target_com_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("target_com_pose", 1); //for debug
  target_com_rot_sub_ = nh_.subscribe("/target_com_rot", 1, &NinjaNavigator::setTargetCoMRotCallback, this);
}

void NinjaNavigator::update()
{
  updateEntSysState();
  // updateAssemblyTree();
  BeetleNavigator::update();
  bool current_assembled = getCurrentAssembled();
  if(current_assembled){
    try
      {
        KDL::Frame current_com;
        KDL::Frame target_cog_pose;
        KDL::Frame target_com_pose;
        geometry_msgs::TransformStamped transformStamped;
        
        target_com_pose.M = target_com_rot_;
        tf::vectorTFToKDL(getTargetPosCand(),target_com_pose.p);
        target_cog_pose = target_com_pose * getCog2CoM<KDL::Frame>().Inverse();

        geometry_msgs::TransformStamped tf;
        tf::transformKDLToMsg(target_cog_pose, tf.transform);
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id ="world";
        tf.child_frame_id = my_name_ + std::to_string(my_id_)+"/target_cog_pose";
        br_.sendTransform(tf);
      }
    catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM("CoM is not defined");
        return;
      }
    if(control_flag_)
      {
        geometry_msgs::Pose com_pose_msg;
        KDL::Frame target_com_pose;
        target_com_pose.p = KDL::Vector(target_pos_candidate_.x(), target_pos_candidate_.y(), target_pos_candidate_.z());
        target_com_pose.M = target_com_rot_;
        tf::PoseKDLToMsg(target_com_pose, com_pose_msg);
        target_com_pose_pub_.publish(com_pose_msg);
      }
  }
  
}

void NinjaNavigator::updateEntSysState()
{
  for(const auto & item : assembly_flags_){
    int id = item.first;
    bool value = item.second;
    auto it = std::find(assembled_modules_ids_.begin(), assembled_modules_ids_.end(), id);
    bool id_pre_exist = (it != assembled_modules_ids_.end()) ? true : false;
    if(!id_pre_exist && value)
      {
        assembled_modules_ids_.push_back(id);
        ModuleData module_data(id);
        ninja_robot_model_->copyTreeStructure(ninja_robot_model_->getInitModuleTree(), module_data.module_tree_);
        module_data.joint_pos_ = KDL::JntArray(4); //docking yaw and pitch
        assembled_modules_data_.insert(std::make_pair(id,module_data));
      }
    else if(id_pre_exist && ! value)
      {
        assembled_modules_ids_.erase(it);
        assembled_modules_data_.erase(id);
      }
    else
      {
        continue;
      }
  }
  setModuleNum(assembled_modules_ids_.size());
  std::sort(assembled_modules_ids_.begin(), assembled_modules_ids_.end());
  if(control_flag_){
    reconfig_flag_ =  (pre_assembled_modules_ != module_num_) ? true : false;
    pre_assembled_modules_ = module_num_;
  }
}

void NinjaNavigator::calcCenterOfMoving()
{
  /*Check state*/
  geometry_msgs::Point cog_com_dist_msg;
  if(!module_num_ || module_num_ == 1 || !assembly_flags_[my_id_]){
    // pre_assembled_modules_ = module_num_;
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

  /*Update com-cog transformation only during hovering */
  if(control_flag_){
    KDL::Frame com_frame;
    KDL::Chain chain;
    std::string left_dock = "pitch_connect_point";
    std::string right_dock = "yaw_connect_point";
    if(my_id_ == leader_id_)
      {
        setCog2CoM(com_frame);
      }
    else
      {
        if (my_id_ < leader_id_)
          {
            /*Calculate FK*/
            for(auto & it: assembled_modules_data_)
              {
                ModuleData data = it.second;
                if(it.first < my_id_)
                  {
                    continue;
                  }
                else if(it.first == my_id_)
                  {
                    if(!data.module_tree_.getChain("fc",right_dock,chain))
                      {
                        ROS_ERROR_STREAM("Failed to get a chain of module" << it.first);
                        return;
                      }
                    KDL::ChainFkSolverPos_recursive fk_solver(chain);
                    KDL::Frame frame;
                    KDL::JntArray joint_positions(1);
                    joint_positions(0) = data.joint_pos_(YAW);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0)
                      {
                        ROS_ERROR_STREAM("Failed to compute FK for module" << it.first);
                        return;
                      }
                    com_frame = com_frame * frame;
                  }
                else if( it.first == leader_id_)
                  {
                    if(!data.module_tree_.getChain(left_dock,"fc",chain))
                      {
                        ROS_ERROR_STREAM("Failed to get a chain of module" << it.first);
                        return;
                          }
                    KDL::ChainFkSolverPos_recursive fk_solver(chain);
                    KDL::Frame frame;
                    KDL::JntArray joint_positions(1);
                    joint_positions(0) = data.joint_pos_(PITCH);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0) {
                      ROS_ERROR_STREAM("Failed to compute FK for module" << it.first);
                      return;
                    }
                    com_frame = com_frame * frame;
                  }
                else if(my_id_ < it.first)
                  {
                    if(!data.module_tree_.getChain(left_dock,right_dock,chain))
                      {
                        ROS_ERROR_STREAM("Failed to get a chain of module" << it.first);
                        return;
                          }
                    KDL::ChainFkSolverPos_recursive fk_solver(chain);
                    KDL::Frame frame;
                    KDL::JntArray joint_positions(2);
                    joint_positions(0) = data.joint_pos_(PITCH);
                    joint_positions(1) = data.joint_pos_(YAW);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0) {
                      ROS_ERROR_STREAM("Failed to compute FK for module" << it.first);
                      return;
                    }
                    com_frame = com_frame * frame;
                  }
              }
            com_frame = com_frame.Inverse();
          }
        else
          {
            /*Calculate FK*/
            for(auto & it: assembled_modules_data_)
              {
                ModuleData data = it.second;
                if( it.first == leader_id_)
                  {
                    if(!data.module_tree_.getChain("fc",right_dock,chain))
                      {
                        ROS_ERROR_STREAM("Failed to get a chain of module" << it.first);
                        return;
                          }
                    KDL::ChainFkSolverPos_recursive fk_solver(chain);
                    KDL::Frame frame;
                    KDL::JntArray joint_positions(1);
                    joint_positions(0) = data.joint_pos_(YAW);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0) {
                      ROS_ERROR_STREAM("Failed to compute FK for module" << it.first);
                      return;
                    }
                    com_frame = com_frame * frame;
                  }
                else if(it.first < my_id_)
                  {
                    if(!data.module_tree_.getChain(left_dock,right_dock,chain))
                      {
                        ROS_ERROR_STREAM("Failed to get a chain of module" << it.first);
                        return;
                          }
                    KDL::ChainFkSolverPos_recursive fk_solver(chain);
                    KDL::Frame frame;
                    KDL::JntArray joint_positions(2);
                    joint_positions(0) = data.joint_pos_(YAW);
                    joint_positions(1) = data.joint_pos_(PITCH);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0) {
                      ROS_ERROR_STREAM("Failed to compute FK for module" << it.first);
                      return;
                    }
                    com_frame = com_frame * frame;
                  }
                else if(it.first == my_id_)
                  {
                    if(!data.module_tree_.getChain(left_dock,"fc",chain))
                      {
                        ROS_ERROR_STREAM("Failed to get a chain of module" << it.first);
                        return;
                          }
                    KDL::ChainFkSolverPos_recursive fk_solver(chain);
                    KDL::Frame frame;
                    KDL::JntArray joint_positions(1);
                    joint_positions(0) = data.joint_pos_(PITCH);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0) {
                      ROS_ERROR_STREAM("Failed to compute FK for module" << it.first);
                      return;
                    }
                    com_frame = com_frame * frame;
                  }
                else if(my_id_ < it.first)
                  {
                    continue;
                  }
              }
          }
        KDL::Frame cog2base;
        cog2base.p = ninja_robot_model_->getCogDesireOrientation<KDL::Rotation>().Inverse() * ninja_robot_model_->getCog2Baselink<KDL::Frame>().p;
        setCog2CoM((cog2base.Inverse() * com_frame * cog2base).Inverse());

      }

    if(reconfig_flag_){
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

  if(control_flag_) current_assembled_ = true;
}

void NinjaNavigator::convertTargetPosFromCoG2CoM()
{
  if(!control_flag_) return;

  bool current_assembled = getCurrentAssembled();
  bool reconfig_flag = getReconfigFlag();

  KDL::Frame init_frame;

  if(pre_assembled_  && !current_assembled){ //disassembly process
    ROS_INFO("switched");
    pre_assembled_ = current_assembled;
    return;
  } else if((!pre_assembled_  && current_assembled) || (current_assembled && reconfig_flag)){ //assembly or reconfig process
    setTargetCoMPoseFromCurrState();
    ROS_INFO("switched");
    pre_assembled_ = current_assembled;
    return;
  }else if(!current_assembled){
    return;
  }else if(my_id_ == leader_id_){
    KDL::Frame target_my_pose;
    if(getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE)
      {
        setTargetPosX(target_pos_candidate_.x());
        setTargetPosY(target_pos_candidate_.y());
        setTargetPosZ(getTargetPos().z());
      }
    else
      {
        setTargetPos(target_pos_candidate_);
      }
    tf::Vector3 target_rot;
    double target_roll, target_pitch, target_yaw;
    target_com_rot_.GetEulerZYX(target_yaw, target_pitch, target_roll);
    target_rot.setX(target_roll);
    target_rot.setY(target_pitch);
    target_rot.setZ(target_yaw);
    setFinalTargetBaselinkRot(target_rot);
    setTargetYaw(target_rot.z());
    return;
  }else if(getCog2CoM<KDL::Frame>() == init_frame){
    return;
  }

  if( int(pre_target_pos_.x() * 1000) != int(getTargetPos().x() * 1000) ||
      int(pre_target_pos_.y() * 1000) != int(getTargetPos().y() * 1000) ||
      int(pre_target_pos_.z() * 1000) != int(getTargetPos().z() * 1000) ||
      int(pre_target_rot_.z() * 1000) != int(getTargetRPY().z() * 1000)){
    setTargetCoMPoseFromCurrState();
    ROS_INFO("Target com pose is updated!");
  }

  /*Target pose conversion*/
  KDL::Frame target_com_pose;
  KDL::Frame target_my_pose;
  target_com_pose.M = target_com_rot_;
  tf::vectorTFToKDL(getTargetPosCand(),target_com_pose.p);
  target_my_pose = target_com_pose * getCog2CoM<KDL::Frame>().Inverse(); // com -> cog

  tf::Vector3 target_pos, target_rot;
  double target_roll, target_pitch, target_yaw;
  tf::vectorKDLToTF(target_my_pose.p, target_pos);
  target_my_pose.M.GetEulerZYX(target_yaw, target_pitch, target_roll);
  target_rot.setX(target_roll);
  target_rot.setY(target_pitch);
  target_rot.setZ(target_yaw);

  /*Set new target pose*/
  if( getNaviState() == HOVER_STATE ||
      getNaviState() == TAKEOFF_STATE){
    setTargetPos(target_pos);
    setTargetYaw(target_rot.z());
    setFinalTargetBaselinkRot(target_rot);  
  }

  pre_target_pos_.setX(target_pos.x());
  pre_target_pos_.setY(target_pos.y());
  pre_target_pos_.setZ(target_pos.z());
  pre_target_rot_.setX(target_rot.x());
  pre_target_rot_.setY(target_rot.y());
  pre_target_rot_.setZ(target_rot.z());
}

void NinjaNavigator::setTargetCoMPoseFromCurrState()
{
  bool current_assembled = getCurrentAssembled();
  if(current_assembled){
    try
      {
        KDL::Frame current_com;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer_.lookupTransform("world", my_name_ + std::to_string(my_id_) + std::string("/center_of_moving") , ros::Time(0));
        tf::transformMsgToKDL(transformStamped.transform, current_com);
        tf::Vector3 target_com_pos;
        tf::vectorKDLToTF(current_com.p, target_com_pos);
        if(getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE) target_com_pos.setZ(getTargetPos().z());
        setTargetPosCand(target_com_pos);
        setTargetComRot(current_com.M);
      }
    catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM("CoM is not defined");
        return;
      }
  }

}

void NinjaNavigator::setTargetCoMRotCallback(const spinal::DesireCoordConstPtr & msg)
{
  setTargetComRot(KDL::Rotation::RPY(msg->roll, msg->pitch, msg->yaw)); 
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::NinjaNavigator, aerial_robot_navigation::BaseNavigator);

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
}

void NinjaNavigator::update()
{
  updateEntSysState();
  // updateAssemblyTree();
  BeetleNavigator::update();
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
        module_data.joint_pos_ = KDL::JntArray(2); //docking yaw and pitch
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
    std::string left_dock = "pitch_dock_link";
    std::string right_dock = "yaw_dock_link";
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
        KDL::Frame leader_base2cog;
        leader_base2cog.p = ninja_robot_model_->getInitCog2BaseVec();
        setCog2CoM(leader_base2cog.Inverse() * com_frame);
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
    cog_com_dist_msg.x = Cog2CoM_.p.x();
    cog_com_dist_msg.y = Cog2CoM_.p.y();
    cog_com_dist_msg.z = Cog2CoM_.p.z();
    cog_com_dist_pub_.publish(cog_com_dist_msg);
    if(control_flag_) current_assembled_ = true;
}

void NinjaNavigator::convertTargetPosFromCoG2CoM(){

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

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::NinjaNavigator, aerial_robot_navigation::BaseNavigator);

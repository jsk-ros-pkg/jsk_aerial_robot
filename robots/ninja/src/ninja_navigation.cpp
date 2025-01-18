// -*- mode: c++ -*-

#include <ninja/ninja_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

NinjaNavigator::NinjaNavigator():
  BeetleNavigator(),
  module_joint_num_(2),
  morphing_flag_(false),
  asm_teleop_reset_time_(0),
  asm_vel_based_waypoint_(false)
{
}

void NinjaNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                double loop_du)
{
  BeetleNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);
  ninja_robot_model_ = boost::dynamic_pointer_cast<NinjaRobotModel>(robot_model);
  target_com_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("debug/target_com_pose", 1); //for debug
  joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  dock_joints_pos_pub_ = nh_.advertise<sensor_msgs::JointState>("dock_joints_pos", 1);
  target_com_rot_sub_ = nh_.subscribe("/target_com_rot", 1, &NinjaNavigator::setGoalCoMRotCallback, this);
  target_joints_pos_sub_ = nh_.subscribe("/assembly/target_joint_pos", 1, &NinjaNavigator::assemblyJointPosCallback, this);
  joint_state_sub_ = nh_.subscribe("joint_states", 1, &NinjaNavigator::jointStateCallback, this);
  tfBuffer_.setUsingDedicatedThread(true);
  joint_pos_errs_.resize(module_joint_num_);
  prev_morphing_stamp_ = ros::Time::now().toSec();
  for(int i = 0; i < getMaxModuleNum(); i++){
    std::string module_name  = string("/") + getMyName() + std::to_string(i+1);
    module_joints_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/dock_joints_pos"), 1, &NinjaNavigator::moduleJointsCallback, this)));
    KDL::JntArray joints_pos(2);
    all_modules_joints_pos_.insert(make_pair(i+1,joints_pos));
  }
}

void NinjaNavigator::update()
{
  updateEntSysState();
  updateMyState();
  calcCenterOfMoving();

  if(ros::Time::now().toSec() - prev_morphing_stamp_ > morphing_process_interval_)
    {
      comRotationProcess();
      morphingProcess();
      morphing_flag_ = true;
      prev_morphing_stamp_ = ros::Time::now().toSec();
    }
  else
    {
      morphing_flag_ = false;
    }
  comMovingProcess();

  GimbalrotorNavigator::update();
  setControlFlag((getNaviState() == HOVER_STATE || getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE) ? true : false);
  convertTargetPosFromCoG2CoM();
  land_height_ = getInitHeight();  

  //for debug
  bool current_assembled = getCurrentAssembled();
  if(current_assembled && !getReconfigFlag()){
    try
      {
        KDL::Frame current_com;
        KDL::Frame target_cog_pose;
        KDL::Frame target_com_pose;
        KDL::Frame raw_base2cog; // base2cog conversion without desire coord process
        geometry_msgs::TransformStamped transformStamped;
        target_com_pose.M = target_com_rot_;
        tf::vectorTFToKDL(getTargetPosCand(),target_com_pose.p);
        raw_base2cog.p = ninja_robot_model_->getCog2Baselink<KDL::Frame>().Inverse().p;
        // target_cog_pose = target_com_pose * getCom2Base<KDL::Frame>() * ninja_robot_model_->getCog2Baselink<KDL::Frame>().Inverse();
        target_cog_pose = target_com_pose * getCom2Base<KDL::Frame>() * raw_base2cog;

        geometry_msgs::TransformStamped tf;

        //send target cog pose
        tf::transformKDLToMsg(target_cog_pose, tf.transform);
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id ="world";
        tf.child_frame_id = my_name_ + std::to_string(my_id_)+"/target_cog_pose";
        br_.sendTransform(tf);

        //send target cog pose
        tf::transformKDLToMsg(target_com_pose, tf.transform);
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id ="world";
        tf.child_frame_id = my_name_ + std::to_string(my_id_)+"/target_com_pose";
        br_.sendTransform(tf);
      }
    catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM("CoM is not defined (update)");
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
        module_data.des_joint_pos_ = KDL::JntArray(module_joint_num_); //docking yaw and pitch
        module_data.est_joint_pos_ = KDL::JntArray(module_joint_num_);
        module_data.goal_joint_pos_ = KDL::JntArray(module_joint_num_);
        module_data.start_joint_pos_ = KDL::JntArray(module_joint_num_);
        module_data.first_joint_processed_time_ = std::vector<double>(2,-1);
        module_data.joint_process_coef_ = std::vector<double>(2);
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
  if(assembled_modules_ids_.size() >= 2)
    {
      left_id_ = assembled_modules_ids_.front();
      right_id_ = assembled_modules_ids_.back();
    }
  if(control_flag_){
    reconfig_flag_ =  (pre_assembled_modules_ != module_num_) ? true : false;
    disassembly_flag_ = (module_num_ < pre_assembled_modules_) ? true : false;
    pre_assembled_modules_ = module_num_;
  }
}

void NinjaNavigator::updateMyState()
{
  auto it = std::find(assembled_modules_ids_.begin(), assembled_modules_ids_.end(), my_id_);
  if(!module_num_ || module_num_ == 1 || it == assembled_modules_ids_.end())
    {
      current_assembled_ = false;
      module_state_ = SEPARATED;
    }
  else
    {
      my_index_ = std::distance(assembled_modules_ids_.begin(), it);
      if(control_flag_) current_assembled_ = true;
    }
}

void NinjaNavigator::calcCenterOfMoving()
{
  /*Check state*/
  geometry_msgs::Point cog_com_dist_msg;
  if(!current_assembled_){

    KDL::Frame raw_cog2base; // co2base conversion without desire coord process
    raw_cog2base.p = ninja_robot_model_->getCogDesireOrientation<KDL::Rotation>().Inverse() * ninja_robot_model_->getCog2Baselink<KDL::Frame>().p;
    KDL::Frame com_frame = raw_cog2base;
    setCoM2Base(com_frame);
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
        KDL::Frame raw_cog2base; // co2base conversion without desire coord process
        raw_cog2base.p = ninja_robot_model_->getCogDesireOrientation<KDL::Rotation>().Inverse() * ninja_robot_model_->getCog2Baselink<KDL::Frame>().p;
        KDL::Frame com_frame = raw_cog2base;
        setCoM2Base(com_frame);
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
                    joint_positions(0) = data.des_joint_pos_(YAW);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0)
                      {
                        ROS_ERROR_STREAM("Failed to compute FK for module" << it.first);
                        return;
                      }
                    com_frame = com_frame * frame;
                  }
                else if(my_id_ < it.first && it.first < leader_id_)
                  {
                    if(!data.module_tree_.getChain(left_dock,right_dock,chain))
                      {
                        ROS_ERROR_STREAM("Failed to get a chain of module" << it.first);
                        return;
                      }
                    KDL::ChainFkSolverPos_recursive fk_solver(chain);
                    KDL::Frame frame;
                    KDL::JntArray joint_positions(2);
                    joint_positions(0) = data.des_joint_pos_(PITCH);
                    joint_positions(1) = data.des_joint_pos_(YAW);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0) {
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
                    joint_positions(0) = data.des_joint_pos_(PITCH);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0) {
                      ROS_ERROR_STREAM("Failed to compute FK for module" << it.first);
                      return;
                    }
                    com_frame = com_frame * frame;
                  }
                else if(leader_id_ < it.first)
                  {
                    continue;
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
                if(it.first < leader_id_)
                  {
                    continue;
                  }
                else if( it.first == leader_id_)
                  {
                    if(!data.module_tree_.getChain("fc",right_dock,chain))
                      {
                        ROS_ERROR_STREAM("Failed to get a chain of module" << it.first);
                        return;
                      }
                    KDL::ChainFkSolverPos_recursive fk_solver(chain);
                    KDL::Frame frame;
                    KDL::JntArray joint_positions(1);
                    joint_positions(0) = data.des_joint_pos_(YAW);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0) {
                      ROS_ERROR_STREAM("Failed to compute FK for module" << it.first);
                      return;
                    }
                    com_frame = com_frame * frame;
                  }
                else if(leader_id_ < it.first && it.first < my_id_)
                  {
                    if(!data.module_tree_.getChain(left_dock,right_dock,chain))
                      {
                        ROS_ERROR_STREAM("Failed to get a chain of module" << it.first);
                        return;
                      }
                    KDL::ChainFkSolverPos_recursive fk_solver(chain);
                    KDL::Frame frame;
                    KDL::JntArray joint_positions(2);
                    joint_positions(0) = data.des_joint_pos_(PITCH);
                    joint_positions(1) = data.des_joint_pos_(YAW);
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
                    joint_positions(0) = data.des_joint_pos_(PITCH);
                    if (fk_solver.JntToCart(joint_positions, frame) < 0) {
                      ROS_ERROR_STREAM("Failed to compute FK for module" << it.first);
                      return;
                    }
                    com_frame = com_frame * frame;
                    test_frame_ = com_frame;
                  }
                else if(my_id_ < it.first)
                  {
                    continue;
                  }
              }
          }
        KDL::Frame raw_cog2base; // co2base conversion without desire coord process
        raw_cog2base.p = ninja_robot_model_->getCogDesireOrientation<KDL::Rotation>().Inverse() * ninja_robot_model_->getCog2Baselink<KDL::Frame>().p;
        setCoM2Base(raw_cog2base * com_frame);
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

  // if(control_flag_) current_assembled_ = true;
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
    if(!disassembly_flag_) setTargetJointPosFromCurrState(); //bad implementation
    ROS_INFO("switched");
    pre_assembled_ = current_assembled;
    return;
  }else if(!current_assembled){
    return;
  }
  if( std::round(pre_target_pos_.x() * 1000) != std::round(getTargetPos().x() * 1000) ||
      std::round(pre_target_pos_.y() * 1000) != std::round(getTargetPos().y() * 1000) ||
      std::round(pre_target_pos_.z() * 1000) != std::round(getTargetPos().z() * 1000) ||
      std::round(pre_target_rot_.z() * 1000) != std::round(getTargetRPY().z() * 1000)){
    setTargetCoMPoseFromCurrState();
    ROS_INFO("Target com pose is updated!");
  }

  /*Target pose conversion*/
  KDL::Frame target_com_pose;
  KDL::Frame target_my_pose;
  KDL::Frame raw_base2cog; // base2cog conversion without desire coord process
  target_com_pose.M = target_com_rot_;
  tf::vectorTFToKDL(getTargetPosCand(),target_com_pose.p);
  raw_base2cog.p = ninja_robot_model_->getCog2Baselink<KDL::Frame>().Inverse().p;
  target_my_pose = target_com_pose * getCom2Base<KDL::Frame>() * raw_base2cog; // com -> cog

  /*Target twist conversion*/
  KDL::Vector target_com_vel;
  KDL::Vector target_my_vel;
  tf::vectorTFToKDL(getTargetVelCand(),target_com_vel);
  target_my_vel = (getCom2Base<KDL::Frame>() * raw_base2cog).M * target_com_vel; // com -> cog

  tf::Vector3 target_pos, target_rot, target_vel;
  double target_roll, target_pitch, target_yaw;
  tf::vectorKDLToTF(target_my_pose.p, target_pos);
  tf::vectorKDLToTF(target_my_vel, target_vel);
  target_my_pose.M.GetEulerZYX(target_yaw, target_pitch, target_roll);
  target_rot.setX(target_roll);
  target_rot.setY(target_pitch);
  target_rot.setZ(target_yaw);

  /*Set new target pose*/
  if( getNaviState() == HOVER_STATE ||
      getNaviState() == TAKEOFF_STATE){
    setTargetPos(target_pos);
    // setTargetVel(target_vel);
    setTargetYaw(target_rot.z());
    forceSetTargetBaselinkRot(target_rot);  
  }

  pre_target_pos_.setX(target_pos.x());
  pre_target_pos_.setY(target_pos.y());
  pre_target_pos_.setZ(target_pos.z());
  pre_target_rot_.setX(target_rot.x());
  pre_target_rot_.setY(target_rot.y());
  pre_target_rot_.setZ(target_rot.z());
}

void NinjaNavigator::comRotationProcess()
{
  if(!control_flag_) return;
  double target_roll, target_pitch, target_yaw;
  double goal_roll, goal_pitch, goal_yaw;
  target_com_rot_.GetEulerZYX(target_yaw, target_pitch, target_roll);
  goal_com_rot_.GetEulerZYX(goal_yaw, goal_pitch, goal_roll);
  //roll
  if(target_roll != goal_roll){
    if(abs(goal_roll- target_roll) > com_roll_change_thresh_)
      target_roll += ((goal_roll - target_roll) * com_roll_change_thresh_);
    else
      target_roll = goal_roll;
  }
  //pitch
  if(target_pitch != goal_pitch)
    {
      if(abs(goal_pitch- target_pitch) > com_pitch_change_thresh_)
        target_pitch += ((goal_pitch - target_pitch) * com_pitch_change_thresh_);
      else
        target_pitch = goal_pitch;
    }

  //yaw
  if(ros::Time::now().toSec() <= asm_teleop_reset_time_)
    {
      target_yaw +=  getTargetOmegaCand().z() * loop_du_;
      goal_yaw = target_yaw;
      setGoalComYaw(goal_yaw);
    }
  if(target_yaw != goal_yaw)
    {
      if(abs(goal_yaw- target_yaw) > com_yaw_change_thresh_)
        target_yaw += ((goal_yaw - target_yaw) * com_yaw_change_thresh_);
      else
        target_yaw = goal_yaw;
    }
  setTargetComRot(KDL::Rotation::RPY(target_roll, target_pitch, target_yaw));
}

void NinjaNavigator::comMovingProcess()
{
  if(!getCurrentAssembled() || !control_flag_) return;
  if(getNaviState() != HOVER_STATE) return;
  /* get current com position*/
  tf::Vector3 current_com_pos;
  try
    {
      KDL::Frame current_com;
      geometry_msgs::TransformStamped transformStamped;
      transformStamped = tfBuffer_.lookupTransform("world", my_name_ + std::to_string(my_id_) + std::string("/center_of_moving") , ros::Time(0));
      tf::transformMsgToKDL(transformStamped.transform, current_com);
      tf::vectorKDLToTF(current_com.p, current_com_pos);
    }
  catch (tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM("CoM is not defined (comMovingProcess)");
      return;
    }
  
  tf::Vector3 delta = getTargetFinalPosCand() - current_com_pos;

  /* process for vel_vased_waypoint mode */
  if(asm_vel_based_waypoint_)
    {
      if(delta.length() > asm_vel_nav_threshold_)
        {
          tf::Vector3 nav_vel = delta * asm_nav_vel_limit_/delta.length();
          setTargetVelCand(nav_vel);
          asm_teleop_reset_time_ = asm_teleop_reset_duration_ + ros::Time::now().toSec();
        }
      else
        {
          asm_vel_based_waypoint_ = false;
          ROS_WARN("back to assembly pos nav control for way point");
          setTargetVelCand(tf::Vector3(0,0,0));
          asm_xy_control_mode_ = POS_CONTROL_MODE;
        }
    }

  switch(asm_xy_control_mode_)
    {
    case POS_CONTROL_MODE:
      {
        setTargetPosCand(target_final_pos_candidate_);
        setTargetVelCand(tf::Vector3(0,0,0));
        break;
      }
    case VEL_CONTROL_MODE:
      {
        if(ros::Time::now().toSec() > asm_teleop_reset_time_)
          {
            setTargetVelCand(tf::Vector3(0,0,0));
            asm_xy_control_mode_ = POS_CONTROL_MODE;
            setFinalTargetPosCand(getTargetPosCand());
          }
        else
          {
            tf::Vector3 new_target_com_pos = getTargetPosCand() + getTargetVelCand()*loop_du_;
            setTargetPosCand(new_target_com_pos);
          }
        break;  
      }
    }
}

void NinjaNavigator::setTargetCoMPoseFromCurrState()
{
  bool current_assembled = getCurrentAssembled();
  if(current_assembled){
    try
      {
        tfBuffer_.canTransform("world", my_name_ + std::to_string(my_id_) + std::string("/center_of_moving"), ros::Time::now(), ros::Duration(0.1));
        KDL::Frame current_com;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer_.lookupTransform("world", my_name_ + std::to_string(my_id_) + std::string("/center_of_moving") , ros::Time(0));
        tf::transformMsgToKDL(transformStamped.transform, current_com);
        tf::Vector3 target_com_pos;
        tf::vectorKDLToTF(current_com.p, target_com_pos);

        double current_roll, current_pitch, current_yaw;
        current_com.M.GetEulerZYX(current_yaw, current_pitch, current_roll);
        
        if(getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE)
          {
            target_com_pos.setZ(getTargetPos().z());
            setTargetPosCand(target_com_pos);
            setTargetVelCand(tf::Vector3(0,0,0));
            setFinalTargetPosCand(target_com_pos);
            setTargetComRot(KDL::Rotation::RPY(0,0,current_yaw));
            setGoalComRot(KDL::Rotation::RPY(0,0,current_yaw));
          }
        else
          {
            setTargetPosCand(target_com_pos);
            setTargetVelCand(tf::Vector3(0,0,0));
            setFinalTargetPosCand(target_com_pos);
            setTargetComRot(current_com.M);
            setGoalComRot(current_com.M);
          }
        asm_xy_control_mode_ = POS_CONTROL_MODE;
        asm_vel_based_waypoint_ = false;
        asm_teleop_reset_time_ = ros::Time::now().toSec();
      }
    catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM("CoM is not defined (setTargetCoMPoseFromCurrState)");
        return;
      }
  }

}

void NinjaNavigator::setTargetJointPosFromCurrState()
{
  for(auto & it:assembled_modules_data_)
    {
      ModuleData& data = it.second;
      data.goal_joint_pos_ = all_modules_joints_pos_[data.id_];
      data.des_joint_pos_ = all_modules_joints_pos_[data.id_];
      data.first_joint_processed_time_[YAW] = -1.0;
      data.first_joint_processed_time_[PITCH] = -1.0;
      ROS_INFO_STREAM("Module" << data.id_ << "-> " << "yaw: "<<data.des_joint_pos_(YAW)<< ", pitch: "<<data.des_joint_pos_(PITCH));
    }
  calcCenterOfMoving();
}

void NinjaNavigator::setGoalCoMRotCallback(const spinal::DesireCoordConstPtr & msg)
{
  setGoalComRot(KDL::Rotation::RPY(msg->roll, msg->pitch, msg->yaw));
}

void NinjaNavigator::assemblyJointPosCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  sensor_msgs::JointState joints_ctrl_msg;
  bool joint_send_flag = false;
  
  //TODO: get joint names from yaml
  std::map<std::string, std::string> joint_map;
  joint_map["pitch"] = "pitch_dock_joint";
  joint_map["yaw"] = "yaw_dock_joint";

  if(msg->name.size() > 0)
    {
      for(int i = 0; i < msg->name.size(); i++)
        {
          if(msg->position.size() !=  msg->name.size()) 
            {
              ROS_ERROR("The joint position num and name num are different in ros msgs [%d vs %d]",
                        (int)msg->position.size(), (int)msg->name.size());
              return;
            }
          // Extract id and joint name
          int id;
          std::string joint_name;
          size_t pos = msg->name.at(i).find('/');
          
          std::string module_name = msg->name.at(i).substr(0, pos);
          joint_name = msg->name.at(i).substr(pos + 1);
          std::regex numberRegex("\\d+");
          std::smatch match;
          if (std::regex_search(module_name, match, numberRegex)) {
            id = std::stoi(match.str(0));
          } else {
            std::cout << "Please distinguish module id." << std::endl;
          }

          //check if the module is assembled
          auto it_id = std::find(assembled_modules_ids_.begin(), assembled_modules_ids_.end(), id);
          bool id_exist = (it_id != assembled_modules_ids_.end()) ? true : false;
          if(!id_exist)
            {
              ROS_ERROR_STREAM("ID: " << id << "is not assembled.");
              return;
            }

          //check the joint name
          if(joint_map.find(joint_name) == joint_map.end())
            {
              ROS_ERROR_STREAM("name: " << joint_name << " does not exists");
              return;
            }

          //add to joint control msg
          std::string target_joint_name = joint_map[joint_name];
          double target_joint_angle = msg->position.at(i);

          //setup morphing velocity
          double morphing_vel;
          if(msg->velocity.empty())
            morphing_vel = default_morphing_vel_;
          else
            morphing_vel = msg->velocity.at(0);
          //update joint info for FK
          ModuleData& data = assembled_modules_data_[id];
          int axis;
          if(target_joint_name == "yaw_dock_joint") axis = YAW;
          if(target_joint_name == "pitch_dock_joint") axis = PITCH;
          data.goal_joint_pos_(axis) = target_joint_angle;
          data.start_joint_pos_(axis) = data.des_joint_pos_(axis);
          data.first_joint_processed_time_[axis] = ros::Time::now().toSec();
          double conv_time = abs(data.goal_joint_pos_(axis) - data.des_joint_pos_(axis)) / morphing_vel;
          
          switch(joint_process_func_)
            {
            case CONSTANT:
              if(id == my_id_ && !free_joint_flag_)
                {
                  joint_send_flag = true;
                  joints_ctrl_msg.name.push_back(target_joint_name);
                  joints_ctrl_msg.position.push_back(target_joint_angle);
                }
              data.first_joint_processed_time_[axis] = -1;
              data.des_joint_pos_(axis) = target_joint_angle;
              break;
            case FRAC:
              data.joint_process_coef_[axis] = 99.0/conv_time;
              if(id == my_id_) ROS_INFO_STREAM("Morphing of module"<<my_id_ << "'s " <<(target_joint_name).c_str() << " has started. (" << conv_time << " sec)");
              break;
            case EXP:
              data.joint_process_coef_[axis] = 2 * log(10)/conv_time;
              if(id == my_id_) ROS_INFO_STREAM("Morphing of module"<<my_id_ << "'s " <<(target_joint_name).c_str() << " has started. (" << conv_time << " sec)");
              break;
            default:
              break;
            }
        }
    }
  if(joint_send_flag) joint_control_pub_.publish(joints_ctrl_msg);
}

void NinjaNavigator::jointStateCallback(const sensor_msgs::JointStateConstPtr& states)
{
  // send docking joint positions to other modules
  sensor_msgs::JointState joint_states = *states;
  sensor_msgs::JointState dock_joint_pos_msg;
  for(int i =0; i< joint_states.name.size(); i++) {
    if(joint_states.name[i] =="yaw_dock_joint")
      {
        dock_joint_pos_msg.name.push_back(string("mod")+std::to_string(getMyID())+string("/yaw"));
        dock_joint_pos_msg.position.push_back(joint_states.position[i]);
      }
    else if(joint_states.name[i] =="pitch_dock_joint")
      {
        dock_joint_pos_msg.name.push_back(string("mod")+std::to_string(getMyID())+string("/pitch"));
        dock_joint_pos_msg.position.push_back(joint_states.position[i]);
      }
  }
  dock_joints_pos_pub_.publish(dock_joint_pos_msg);
}

void NinjaNavigator::moduleJointsCallback(const sensor_msgs::JointStateConstPtr& state)
{
  sensor_msgs::JointState joint_states = *state;
  for(int i=0; i<joint_states.name.size(); i++)
    {
      // Extract id and joint name
      int id;
      std::string joint_name;
      size_t pos = joint_states.name[i].find('/');
          
      std::string module_name = joint_states.name[i].substr(0, pos);
      joint_name = joint_states.name[i].substr(pos + 1);
      std::regex numberRegex("\\d+");
      std::smatch match;
      if (std::regex_search(module_name, match, numberRegex))
        id = std::stoi(match.str(0));
      else
        return;
      KDL::JntArray& module_joint_pos = all_modules_joints_pos_[id];
      int axis;
      if(joint_name == "yaw") axis = YAW;
      if(joint_name == "pitch") axis = PITCH;
      module_joint_pos(axis) = joint_states.position[i];
    }
}

void NinjaNavigator::assemblyNavCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)
{
  if(getNaviState() == TAKEOFF_STATE || BaseNavigator::getNaviState() == LAND_STATE || getModuleState() == SEPARATED) return;

  gps_waypoint_ = false;

  if(force_att_control_flag_) return;

  /* xy control */
  switch(msg->pos_xy_nav_mode)
    {
    case aerial_robot_msgs::FlightNav::POS_MODE:
      {
        ROS_ERROR("start pos mode");
        tf::Vector3 target_cog_pos(msg->target_pos_x, msg->target_pos_y, 0);
        tf::Vector3 target_delta = getTargetPosCand() - target_cog_pos;
        target_delta.setZ(0);

        if(target_delta.length() > asm_vel_nav_threshold_)
          {
            ROS_WARN("start assembly vel nav control for waypoint");
            asm_vel_based_waypoint_ = true;
            asm_xy_control_mode_ = VEL_CONTROL_MODE;
          }

        if(!vel_based_waypoint_)
          asm_xy_control_mode_ = POS_CONTROL_MODE;

        setFinalTargetPosCandX(target_cog_pos.x());
        setFinalTargetPosCandY(target_cog_pos.y());

        setTargetVelCandX(0);
        setTargetVelCandY(0);

        break;
      }
    case aerial_robot_msgs::FlightNav::VEL_MODE:
      {
        asm_xy_control_mode_ = VEL_CONTROL_MODE;

        asm_teleop_reset_time_ = asm_teleop_reset_duration_ + ros::Time::now().toSec();

        switch(msg->control_frame)
          {
          case WORLD_FRAME:
            {
              setTargetVelCandX(msg->target_vel_x);
              setTargetVelCandY(msg->target_vel_y);
              break;
            }
          case LOCAL_FRAME:
            {
              double current_com_roll, current_com_pitch, current_com_yaw;
              try
                {
                  tfBuffer_.canTransform("world", my_name_ + std::to_string(my_id_) + std::string("/center_of_moving"), ros::Time::now(), ros::Duration(0.1));
                  KDL::Frame current_com;
                  geometry_msgs::TransformStamped transformStamped;
                  transformStamped = tfBuffer_.lookupTransform("world", my_name_ + std::to_string(my_id_) + std::string("/center_of_moving") , ros::Time(0));
                  tf::transformMsgToKDL(transformStamped.transform, current_com);

                  current_com.M.GetEulerZYX(current_com_yaw, current_com_pitch, current_com_roll);
                }
              catch (tf2::TransformException& ex)
                {
                  ROS_ERROR_STREAM("CoM is not defined (assemblyNavCallback)");
                  return;
                }
              tf::Vector3 target_vel = frameConversion(tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0), current_com_yaw);
              setTargetVelCandX(target_vel.x());
              setTargetVelCandY(target_vel.y());
              break;
            }
          case LEFT_DOCK:
            {
              double current_com_roll, current_com_pitch, current_com_yaw;
              try
                {
                  tfBuffer_.canTransform("world", my_name_ + std::to_string(left_id_) + std::string("/pitch_connect_point"), ros::Time::now(), ros::Duration(0.1));
                  KDL::Frame current_left_dock;
                  geometry_msgs::TransformStamped transformStamped;
                  transformStamped = tfBuffer_.lookupTransform("world", my_name_ + std::to_string(left_id_) + std::string("/pitch_connect_point") , ros::Time(0));
                  tf::transformMsgToKDL(transformStamped.transform, current_left_dock);

                  current_left_dock.M.GetEulerZYX(current_com_yaw, current_com_pitch, current_com_roll);
                }
              catch (tf2::TransformException& ex)
                {
                  ROS_ERROR_STREAM("CoM is not defined (assemblyNavCallback)");
                  return;
                }
              tf::Vector3 target_vel = frameConversion(tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0), current_com_yaw);
              setTargetVelCandX(target_vel.x());
              setTargetVelCandY(target_vel.y());
              break;
            }
          case RIGHT_DOCK:
            {
              double current_com_roll, current_com_pitch, current_com_yaw;
              try
                {
                  tfBuffer_.canTransform("world", my_name_ + std::to_string(right_id_) + std::string("/yaw_connect_point"), ros::Time::now(), ros::Duration(0.1));
                  KDL::Frame current_right_dock;
                  geometry_msgs::TransformStamped transformStamped;
                  transformStamped = tfBuffer_.lookupTransform("world", my_name_ + std::to_string(right_id_) + std::string("/yaw_connect_point") , ros::Time(0));
                  tf::transformMsgToKDL(transformStamped.transform, current_right_dock);

                  current_right_dock.M.GetEulerZYX(current_com_yaw, current_com_pitch, current_com_roll);
                }
              catch (tf2::TransformException& ex)
                {
                  ROS_ERROR_STREAM("CoM is not defined (assemblyNavCallback)");
                  return;
                }
              tf::Vector3 target_vel = frameConversion(tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0), current_com_yaw);
              setTargetVelCandX(target_vel.x());
              setTargetVelCandY(target_vel.y());
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
        asm_xy_control_mode_ = POS_CONTROL_MODE;
        setTargetVelCandX(0);
        setTargetVelCandY(0);
        setTargetCoMPoseFromCurrState();
        break;
      }
    }

  /* z */
  if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      tf::Vector3 target_cog_pos(0, 0, msg->target_pos_z);
      setTargetPosCandZ(target_cog_pos.z());

      setTargetVelCandZ(0);
    }
}

void NinjaNavigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  if(getCurrentAssembled())
    joy_duplicated_flag_ = true;
  else
    joy_duplicated_flag_ = false;

  auto copied_joy_msg = boost::make_shared<sensor_msgs::Joy>(*joy_msg);  

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

  /* Translational Contol*/

  double raw_x_cmd = joy_cmd.axes[PS3_AXIS_STICK_LEFT_UPWARDS];
  double raw_y_cmd = joy_cmd.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
  double raw_z_cmd = joy_cmd.axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
  double raw_yaw_cmd = joy_cmd.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];

  if(fabs(raw_x_cmd) >= joy_stick_deadzone_ || fabs(raw_y_cmd) >= joy_stick_deadzone_)
    {
      asm_teleop_reset_time_ = asm_teleop_reset_duration_ + ros::Time::now().toSec();
      switch (asm_xy_control_mode_)
        {
        case POS_CONTROL_MODE:
          {
            if(fabs(raw_x_cmd) < joy_stick_deadzone_) raw_x_cmd = 0;
            if(fabs(raw_y_cmd) < joy_stick_deadzone_) raw_y_cmd = 0;

            /* vel command */
            setTargetVelCandX(raw_x_cmd * max_teleop_xy_vel_);
            setTargetVelCandY(raw_y_cmd * max_teleop_xy_vel_);
            asm_xy_control_mode_ = VEL_CONTROL_MODE;
            break;
          }
        case VEL_CONTROL_MODE:
          {
            if(fabs(raw_x_cmd) < joy_stick_deadzone_) raw_x_cmd = 0;
            if(fabs(raw_y_cmd) < joy_stick_deadzone_) raw_y_cmd = 0;

            /* vel command */
            setTargetVelCandX(raw_x_cmd * max_teleop_xy_vel_);
            setTargetVelCandY(raw_y_cmd * max_teleop_xy_vel_);
            break;
          }
        default:
          {
            break;
          }
        }
    }
  if(fabs(raw_yaw_cmd) > joy_stick_deadzone_)
    {
      asm_teleop_reset_time_ = asm_teleop_reset_duration_ + ros::Time::now().toSec();
      setTargetOmegaCandZ(raw_yaw_cmd * max_teleop_yaw_vel_);
    }
  else
    {
      setTargetOmegaCandZ(0);
    }
  BeetleNavigator::joyStickControl(copied_joy_msg);
}


void NinjaNavigator::morphingProcess()
{
  if(!getCurrentAssembled() || !control_flag_) return;
  sensor_msgs::JointState joints_ctrl_msg;
  bool joint_send_flag = false;
  std::map<int, std::string> joint_map;
  joint_map[PITCH] = "pitch_dock_joint";
  joint_map[YAW] = "yaw_dock_joint";
  for(auto & it : assembled_modules_data_)
    {
      int id = it.first;
      ModuleData& data = it.second;
      for(int i = 0; i < module_joint_num_; i ++)
        {
          double t = ros::Time::now().toSec() - data.first_joint_processed_time_[i];
          if(data.first_joint_processed_time_[i] < 0) continue;
          if(abs(data.goal_joint_pos_(i) - data.des_joint_pos_(i)) < joint_pos_chnage_thresh_)
            {
              data.first_joint_processed_time_[i] = -1.0;
              data.des_joint_pos_(i) = data.goal_joint_pos_(i);
              if(id == my_id_ && !free_joint_flag_)
                {
                  joint_send_flag = true;
                  joints_ctrl_msg.name.push_back(joint_map[i]);
                  joints_ctrl_msg.position.push_back(data.des_joint_pos_(i));
                  ROS_INFO_STREAM("Morphing of module"<<my_id_ << "'s " <<(joint_map[i]).c_str() << " has finished. (" << t << " sec)");
                }
              continue;
            }
          switch(joint_process_func_)
            {
            case FRAC:
              data.des_joint_pos_(i) = data.start_joint_pos_(i) + (data.goal_joint_pos_(i) - data.start_joint_pos_(i)) * (1 - 1/(data.joint_process_coef_[i] * t + 1) );
              break;
            case EXP:
              data.des_joint_pos_(i) = data.start_joint_pos_(i) + (data.goal_joint_pos_(i) - data.start_joint_pos_(i)) * (1 - exp(-data.joint_process_coef_[i] * t));
              break;
            default:
              break;
            }
          if(id == my_id_ && !free_joint_flag_)
            {
              joint_send_flag = true;
              joints_ctrl_msg.name.push_back(joint_map[i]);
              joints_ctrl_msg.position.push_back(data.des_joint_pos_(i));
            }
        }
    }
  if(joint_send_flag) joint_control_pub_.publish(joints_ctrl_msg);

  /* calculate joint pos err */
  for(auto it = assembled_modules_data_.begin(); it != assembled_modules_data_.end(); ++it)
    {
      int id = it->first;
      ModuleData& data = it->second;
      int left_id, right_id;
      if(it != assembled_modules_data_.begin())
        {
          auto left = std::prev(it);
          left_id = left->first;
          try
            {
              KDL::Frame myCog2LeftCog;
              geometry_msgs::TransformStamped transformStamped;
              transformStamped = tfBuffer_.lookupTransform(my_name_ + std::to_string(id) + std::string("/fc") ,
                                                           my_name_ + std::to_string(left_id) + std::string("/fc"),
                                                           ros::Time(0));
              tf::transformMsgToKDL(transformStamped.transform, myCog2LeftCog);
              data.est_joint_pos_(PITCH) = std::asin(myCog2LeftCog.M(0,2));
            }
          catch (tf2::TransformException& ex)
            {
              ROS_ERROR("cannot estimate joint position");
              return;
            }
        }
      if(it != assembled_modules_data_.end())
        {
          auto right = std::next(it);
          right_id = right->first;
          try
            {
              KDL::Frame myCog2RightCog;
              geometry_msgs::TransformStamped transformStamped;
              transformStamped = tfBuffer_.lookupTransform(my_name_ + std::to_string(id) + std::string("/fc") ,
                                                           my_name_ + std::to_string(right_id) + std::string("/fc"),
                                                           ros::Time(0));
              tf::transformMsgToKDL(transformStamped.transform, myCog2RightCog);  
              data.est_joint_pos_(YAW) = std::atan2(-myCog2RightCog.M(0,1),myCog2RightCog.M(0,0));
            }
          catch (tf2::TransformException& ex)
            {
              ROS_ERROR("cannot estimate joint position");
              return;
            }
        }
    }

  int neighbor_id_;
  if(my_id_ < leader_id_)
    {
      neighbor_id_ = assembled_modules_ids_[my_index_+1];
      joint_pos_errs_[PITCH] = assembled_modules_data_[neighbor_id_].goal_joint_pos_(PITCH) - assembled_modules_data_[neighbor_id_].est_joint_pos_(PITCH);
      joint_pos_errs_[YAW] = assembled_modules_data_[my_id_].goal_joint_pos_(YAW) - assembled_modules_data_[my_id_].est_joint_pos_(YAW);
    }
  else if(my_id_ > leader_id_)
    {
      neighbor_id_ = assembled_modules_ids_[my_index_-1];
      joint_pos_errs_[PITCH] = assembled_modules_data_[my_id_].goal_joint_pos_(PITCH) - assembled_modules_data_[my_id_].est_joint_pos_(PITCH); 
      joint_pos_errs_[YAW] = assembled_modules_data_[neighbor_id_].goal_joint_pos_(YAW) - assembled_modules_data_[neighbor_id_].est_joint_pos_(YAW);
      // ROS_ERROR_STREAM("pitch: " << joint_pos_errs_[PITCH]);
      // ROS_ERROR_STREAM("yaw: " << joint_pos_errs_[YAW]);
    }
  else
    {
      return;
    }
}


void NinjaNavigator::rosParamInit()
{
  ros::NodeHandle nh(nh_, "navigation");
  getParam<double>(nh, "default_morphing_vel", default_morphing_vel_, M_PI/4.0);
  getParam<double>(nh, "joint_pos_change_thresh", joint_pos_chnage_thresh_, 0.01);
  getParam<int>(nh, "joint_process_func", joint_process_func_, CONSTANT);
  getParam<bool>(nh, "free_joint_flag", free_joint_flag_, false);
  getParam<double>(nh, "com_roll_change_thresh", com_roll_change_thresh_, 0.02);
  getParam<double>(nh, "com_yaw_pitch_thresh", com_pitch_change_thresh_, 0.02);
  getParam<double>(nh, "com_yaw_change_thresh", com_yaw_change_thresh_, 0.02);
  
  getParam<double>(nh, "morphing_process_interval", morphing_process_interval_, 0.1);

  getParam<double>(nh, "asm_nav_vel_limit_", asm_nav_vel_limit_, 0.1);
  getParam<double>(nh, "asm_vel_nav_threshold_", asm_vel_nav_threshold_, 0.4);
  getParam<double>(nh, "asm_teleop_reset_duration_", asm_teleop_reset_duration_, 0.5);

  BeetleNavigator::rosParamInit();
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::NinjaNavigator, aerial_robot_navigation::BaseNavigator);

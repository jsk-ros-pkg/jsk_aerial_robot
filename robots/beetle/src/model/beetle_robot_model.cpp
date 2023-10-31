#include <beetle/model/beetle_robot_model.h>

BeetleRobotModel::BeetleRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  GimbalrotorRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon),
  tfBuffer_(),
  tfListener_(tfBuffer_),
  current_assembled_(false)
{
  for(int i = 0; i < max_modules_num_; i++){
    assembly_flags_.insert(std::make_pair(i+1,false));
  }
  nh_.getParam("robot_id", my_id_);
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

  calcCenterOfMoving();
}

void BeetleRobotModel::calcCenterOfMoving()
{
  if(!assembly_flags_[my_id_]){
      KDL::Frame com_frame;
      setCog2CoM(com_frame);
      current_assembled_ = false;
      return;
  }
  std::string cog_name = "beetle" + std::to_string(my_id_) + "/cog";
  Eigen::Vector3f center_of_moving = Eigen::Vector3f::Zero();
  int assembled_module = 0;
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
      }
    catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM("not exist module is mentioned. ID is "<<id );
        return;
      }
  }
  if(!assembled_module) return;
  center_of_moving = center_of_moving / assembled_module;
  if(std::isnan(center_of_moving.x())) return;

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

  //process to calcurate an accurate conversion from cog to com
  if(pre_assembled_modules_ != assembled_module){
    Eigen::Vector3f cog_com_dist(center_of_moving.norm() * center_of_moving.x()/fabs(center_of_moving.x()),0,0);
    tf.transform.translation.x = cog_com_dist.x();
    tf.transform.translation.y = cog_com_dist.y();
    tf.transform.translation.z = cog_com_dist.z();
    KDL::Frame com_frame = tf2::transformToKDL(tf);
    setCog2CoM(com_frame);
  }
  pre_assembled_modules_ = assembled_module;
  current_assembled_ = true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(BeetleRobotModel, aerial_robot_model::RobotModel);

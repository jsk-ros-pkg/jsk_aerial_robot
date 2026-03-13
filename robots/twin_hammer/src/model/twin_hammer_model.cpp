#include <twin_hammer/model/twin_hammer_model.h>

TwinHammerModel::TwinHammerModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  inertia_gimbal1_total_eigen_ = Eigen::Matrix3d::Zero();
  inertia_gimbal2_total_eigen_ = Eigen::Matrix3d::Zero();
}

void TwinHammerModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  RobotModel::updateRobotModelImpl(joint_positions);
  const auto seg_tf_map = getSegmentsTf();
  const auto inertia_map = getInertiaMap();
  KDL::Frame root_to_gimbal1_roll = seg_tf_map.at("gimbal1_roll_module");
  KDL::Frame root_to_gimbal1_pitch = seg_tf_map.at("gimbal1_pitch_module");
  KDL::Frame root_to_gimbal2_roll = seg_tf_map.at("gimbal2_roll_module");
  KDL::Frame root_to_gimbal2_pitch = seg_tf_map.at("gimbal2_pitch_module");
  KDL::Frame gimbal1_roll_to_pitch = root_to_gimbal1_roll.Inverse() * root_to_gimbal1_pitch;
  KDL::Frame gimbal2_roll_to_pitch = root_to_gimbal2_roll.Inverse() * root_to_gimbal2_pitch;

  KDL::RigidBodyInertia inertia_gimbal1_roll = KDL::RigidBodyInertia::Zero();
  KDL::RigidBodyInertia inertia_gimbal1_pitch = KDL::RigidBodyInertia::Zero();
  KDL::RigidBodyInertia inertia_gimbal2_roll = KDL::RigidBodyInertia::Zero();
  KDL::RigidBodyInertia inertia_gimbal2_pitch = KDL::RigidBodyInertia::Zero();
  for(const auto& inertia : inertia_map)
  {
    if(inertia.first=="gimbal1_roll_module"){
      inertia_gimbal1_roll = inertia.second;
    }
    if(inertia.first=="gimbal1_pitch_module"){
      inertia_gimbal1_pitch = inertia.second;
    }
    if(inertia.first=="gimbal2_roll_module"){
      inertia_gimbal2_roll = inertia.second;
    }
    if(inertia.first=="gimbal2_pitch_module"){
      inertia_gimbal2_pitch = inertia.second;
    }
  }
  KDL::RigidBodyInertia inertia_gimbal1_pitch_in_roll_frame = gimbal1_roll_to_pitch * inertia_gimbal1_pitch;
  KDL::RigidBodyInertia inertia_gimbal1_total = inertia_gimbal1_pitch_in_roll_frame + inertia_gimbal1_roll;
  KDL::RigidBodyInertia inertia_gimbal2_pitch_in_roll_frame = gimbal2_roll_to_pitch * inertia_gimbal2_pitch;
  KDL::RigidBodyInertia inertia_gimbal2_total = inertia_gimbal2_pitch_in_roll_frame + inertia_gimbal2_roll;
  inertia_gimbal1_total_eigen_ = aerial_robot_model::kdlToEigen(inertia_gimbal1_total.getRotationalInertia());
  inertia_gimbal2_total_eigen_ = aerial_robot_model::kdlToEigen(inertia_gimbal2_total.getRotationalInertia());

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TwinHammerModel, aerial_robot_model::RobotModel);
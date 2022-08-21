#include <tiger/navigation/walk_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation::Tiger;

WalkNavigator::WalkNavigator():
  BaseNavigator(),
  target_pos_(0,0,0),
  target_vel_(0,0,0),
  target_rpy_(0,0,0)
{
}

void WalkNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator);

  tiger_robot_model_ = boost::dynamic_pointer_cast<::Tiger::FullVectoringRobotModel>(robot_model);
}

void WalkNavigator::update()
{
  BaseNavigator::update();

  // skip before the model initialization
  if (tiger_robot_model_->getStaticVectoringF().size() == 0 ||
      tiger_robot_model_->getStaticJointT().size() == 0) {
    return;
  }


  // TODO: add new states for walk/stand.
  // e.g., idle stand state, joint move state, fee joint end state

  tf::Vector3 baselink_pos = estimator_->getPos(Frame::BASELINK, estimate_mode_);
  tf::Vector3 baselink_vel = estimator_->getVel(Frame::BASELINK, estimate_mode_);
  tf::Vector3 baselink_rpy = estimator_->getEuler(Frame::BASELINK, estimate_mode_);

  auto current_joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();

  if (target_joint_state_.name.size() == 0) {
    // initialize the target joint
    target_joint_state_.name = tiger_robot_model_->getLinkJointNames();
    target_joint_state_.position.resize(target_joint_state_.name.size(), 0);

    // set target joint angles
    const auto& names = target_joint_state_.name;
    auto& positions = target_joint_state_.position;
    for(int i = 0; i < names.size(); i++) {

      auto n = names.at(i);
      const auto& v = current_joint_state.name;
      auto res = std::find(v.begin(), v.end(), n);

      if (res == v.end()) {
        ROS_ERROR_STREAM("[Tiger][Navigator] joint target angles initialization, cannot find " << n);
        continue;
      }

      auto id = std::distance(v.begin(), res);
      positions.at(i) = current_joint_state.position.at(id);
    }
  }

  if (getNaviState() == aerial_robot_navigation::START_STATE) {

    // set the target position for baselink
    ROS_INFO("[Walk][Navigator] set initial position and joint angles as target ones");
    target_pos_ = baselink_pos;
    target_rpy_ = baselink_rpy;

    // set target joint angles
    const auto& names = target_joint_state_.name;
    auto& positions = target_joint_state_.position;
    for(int i = 0; i < names.size(); i++) {

      auto n = names.at(i);
      const auto& v = current_joint_state.name;
      auto res = std::find(v.begin(), v.end(), n);

      if (res == v.end()) {
        ROS_ERROR_STREAM("[Tiger][Navigator] joint target angles in start state, cannot find " << n);
        continue;
      }

      auto id = std::distance(v.begin(), res);
      positions.at(i) = current_joint_state.position.at(id);
    }
  }

}




void WalkNavigator::reset()
{
}


void WalkNavigator::halt()
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joints/torque_enable");
  std_srvs::SetBool srv;
  srv.request.data = false;
  if (client.call(srv))
    ROS_INFO("dragon control halt process: disable the joint torque");
  else
    ROS_ERROR("Failed to call service joints/torque_enable");

  client = nh_.serviceClient<std_srvs::SetBool>("gimbals/torque_enable");

  srv.request.data = false;
  if (client.call(srv))
    ROS_INFO("dragon control halt process: disable the gimbal torque");
  else
    ROS_ERROR("Failed to call service gimbals/torque_enable");
}

void WalkNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::Tiger::WalkNavigator, aerial_robot_navigation::BaseNavigator);

#include <tiger/navigation/walk_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation::Tiger;

WalkNavigator::WalkNavigator():
  BaseNavigator(),
  joint_index_map_(0),
  target_baselink_pos_(0,0,0),
  target_baselink_vel_(0,0,0),
  target_baselink_rpy_(0,0,0),
  target_leg_ends_(0),
  target_link_rots_(0)
{
}

void WalkNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator);

  tiger_robot_model_ = boost::dynamic_pointer_cast<::Tiger::FullVectoringRobotModel>(robot_model);
  robot_model_for_nav_ = boost::make_shared<aerial_robot_model::RobotModel>();

  target_baselink_pos_sub_ = nh_.subscribe("walk/baselink/traget/pos", 1, &WalkNavigator::targetBaselinkPosCallback, this);
  target_baselink_delta_pos_sub_ = nh_.subscribe("walk/baselink/traget/delta_pos", 1, &WalkNavigator::targetBaselinkDeltaPosCallback, this);

  target_joint_angles_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/nav/target_joint_angles", 1); // for debug
}

void WalkNavigator::update()
{
  BaseNavigator::update();

  // skip before the model initialization
  if (tiger_robot_model_->getStaticVectoringF().size() == 0 ||
      tiger_robot_model_->getStaticJointT().size() == 0) {
    return;
  }

  auto current_joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();

  // TODO: add new states for walk/stand.
  // e.g., idle stand state, joint move state, fee joint end state

  tf::Vector3 baselink_pos = estimator_->getPos(Frame::BASELINK, estimate_mode_);
  tf::Vector3 baselink_vel = estimator_->getVel(Frame::BASELINK, estimate_mode_);
  tf::Vector3 baselink_rpy = estimator_->getEuler(Frame::BASELINK, estimate_mode_);

  // target joint angles
  if (target_joint_state_.name.size() == 0) {

    // initialize the target joint
    target_joint_state_.name = tiger_robot_model_->getLinkJointNames();
    target_joint_state_.position.resize(target_joint_state_.name.size());

    // set (initialize) joint index map
    setJointIndexMap();

    // set target joint angles
    target_joint_state_.position = getCurrentJointAngles();
  }

  // target link rotation
  if (target_link_rots_.size() == 0) {
    target_link_rots_.resize(tiger_robot_model_->getRotorNum());
  }

  // get target leg position w.r.t. world frame
  const auto& seg_tf_map = tiger_robot_model_->getSegmentsTf();
  if(seg_tf_map.size() == 0) return;
  tf::Transform tf_w_baselink(estimator_->getOrientation(Frame::BASELINK, estimate_mode_), baselink_pos);
  KDL::Frame fw_baselink;
  tf::transformTFToKDL(tf_w_baselink, fw_baselink);

  std::vector<KDL::Frame> curr_leg_ends;
  const int leg_num = tiger_robot_model_->getRotorNum() / 2;
  KDL::Frame fr_baselink = seg_tf_map.at(tiger_robot_model_->getBaselinkName());
  for (int i = 0; i < leg_num; i++) {
    std::string frame_name = std::string("link") + std::to_string(2 * (i + 1)) + std::string("_foot");
    KDL::Frame fr_end = seg_tf_map.at(frame_name); // w.r.t. root link
    KDL::Frame fb_end = fr_baselink.Inverse() * fr_end; // w.r.t. baselink
    KDL::Frame fw_end = fw_baselink * fb_end; // w.r.t. world frame
    curr_leg_ends.push_back(fw_end);
  }

  if (target_leg_ends_.size() == 0) {
    target_leg_ends_ = curr_leg_ends;

    target_baselink_pos_ = baselink_pos;
    target_baselink_rpy_ = baselink_rpy;
  }

  if (getNaviState() == aerial_robot_navigation::START_STATE) {

    // set the target position for baselink
    ROS_INFO("[Walk][Navigator] set initial position and joint angles as target ones");
    target_baselink_pos_ = baselink_pos;
    target_baselink_rpy_ = baselink_rpy;

    // set target leg end frame (position)
    target_leg_ends_ = curr_leg_ends;
  }

  // calculate the target joint angles from baselink and end position
  KDL::Frame fw_target_baselink;
  fw_target_baselink.M = KDL::Rotation::EulerZYX(target_baselink_rpy_.z(),
                                                 target_baselink_rpy_.y(),
                                                 target_baselink_rpy_.x());
  tf::vectorTFToKDL(target_baselink_pos_, fw_target_baselink.p);

  double l0 = -1; // distance from "joint_junction_linkx" to "linkx"
  double l1 = -1; // distance from "linkx" to "linkx+1"
  double l2 = -1; // distance from "linkx+1" to "linkx+1_foot"

  const auto& names = target_joint_state_.name;
  auto& target_angles = target_joint_state_.position;
  std::stringstream ss;

  for (int i = 0; i < leg_num; i++) {
    std::string yaw_frame_name = std::string("joint_junction_link") + std::to_string(2 * i + 1);
    KDL::Frame fr_joint_yaw = seg_tf_map.at(yaw_frame_name); // w.r.t. root link
    fr_joint_yaw.M = KDL::Rotation::EulerZYX(M_PI/ 4 + M_PI/2 * i, 0, 0);  // workaround: get the orientation of the parent frame
    KDL::Frame fb_joint_yaw = fr_baselink.Inverse() * fr_joint_yaw; // w.r.t. baselink
    KDL::Frame fw_joint_yaw = fw_target_baselink * fb_joint_yaw; // w.r.t. world frame


    //KDL::Frame fw_end = curr_leg_ends.at(i); // TODO: change from curr_leg_ens to target_leg_ends
    KDL::Frame fw_end = target_leg_ends_.at(i);
    KDL::Frame fy_end = fw_joint_yaw.Inverse() * fw_end; // w.r.t. yaw (yaw1) frame.
    KDL::Vector p = fy_end.p;
    // if (i == 0)
    //   ROS_ERROR_STREAM_THROTTLE(1.0, "fw_end pos: " << aerial_robot_model::kdlToEigen(p).transpose());

    double angle = atan2(p.y(), p.x());

    std::string pitch1_frame_name = std::string("link") + std::to_string(2 * i + 1);
    std::string pitch2_frame_name = std::string("link") + std::to_string(2 * (i + 1));
    std::string end_frame_name = std::string("link") + std::to_string(2 * (i + 1)) + std::string("_foot");

    if(l0 < 0) {
      KDL::Frame fr_joint_pitch1 = seg_tf_map.at(pitch1_frame_name);
      KDL::Frame fr_joint_pitch2 = seg_tf_map.at(pitch2_frame_name);
      KDL::Frame fr_end = seg_tf_map.at(end_frame_name);

      l0 = (fr_joint_pitch1.p - fr_joint_yaw.p).Norm();
      l1 = (fr_joint_pitch2.p - fr_joint_pitch1.p).Norm();
      l2 = (fr_end.p - fr_joint_pitch2.p).Norm();
    }

    // TODO: hard-coding for the kinemtaics from "joint_junction_linkx" to "linkx"
    KDL::Frame frame_delta1(KDL::Rotation::EulerZYX(angle, 0, 0), KDL::Vector::Zero());
    KDL::Frame frame_delta2(KDL::Rotation::Identity(), KDL::Vector(l0, 0, 0));
    KDL::Frame fr_joint_pitch1 = fr_joint_yaw * frame_delta1 * frame_delta2;
    // if (i == 0)
    //   ROS_ERROR_STREAM_THROTTLE(1.0, "l0: " << l0 << "; fr_joint_pitch1: " << aerial_robot_model::kdlToEigen(fr_joint_pitch1.p).transpose() << "; fr_joint_yaw: " << aerial_robot_model::kdlToEigen(fr_joint_yaw.p).transpose());
    KDL::Frame fb_joint_pitch1 = fr_baselink.Inverse() * fr_joint_pitch1; // w.r.t. baselink
    KDL::Frame fw_joint_pitch1 = fw_target_baselink * fb_joint_pitch1; // w.r.t. world frame

    KDL::Frame fp1_end = fw_joint_pitch1.Inverse() * fw_end;  // w.r.t. pitch1 frame
    double x_e = fp1_end.p.x();
    double y_e = - fp1_end.p.z(); // w.r.t pitch1 frame
    double b = l1 * l1 - x_e * x_e - y_e * y_e - l2 * l2;
    double b_d = b / (- 2 * l2);
    double d = sqrt(x_e * x_e + y_e * y_e);
    double b_dd = b_d / d;
    if (fabs(b_dd) > 1) {
      ROS_ERROR_STREAM("[Tiger][Navigator] leg " << i + 1 << ": b_dd exceeds the valid scope: " << b_dd
                       << "; end pos: " << aerial_robot_model::kdlToEigen(fw_end.p).transpose()
                       << "; baselink pos: " << aerial_robot_model::kdlToEigen(fw_target_baselink.p).transpose());
      continue;
    }

    double phi = atan2(y_e, x_e);
    double theta2_d = acos(b_dd) + phi;
    if (theta2_d < 0) {
      theta2_d = - acos(b_dd) + phi;
    }
    double theta1 = atan2(y_e - l2 * sin(theta2_d), x_e - l2 * cos(theta2_d));
    double theta2 = theta2_d - theta1;

    // set the target joint angles
    if(names.at(4 * i) == std::string("joint") + std::to_string(2*i+1) + std::string("_yaw")) {
      target_angles.at(4 * i) = angle;
      target_angles.at(4 * i + 1) = theta1;
      target_angles.at(4 * i + 3) = theta2;

      ss << "(" << angle << ", " << theta1 << ", 0" << ", " << theta2 << ") ";
    }
    else {
      ROS_ERROR_STREAM("[Tiger][Navigator] name order is different. ID" << i << " name is " << names.at(4 * i));
    }
  }
  ROS_DEBUG_STREAM_THROTTLE(1.0, "[Tiger][Navigator] analytical joint angles from leg end and baselink: " << ss.str());

  std_msgs::Float32MultiArray msg;
  for(int i = 0; i < target_angles.size(); i++) {
    msg.data.push_back(target_angles.at(i));
  }
  target_joint_angles_pub_.publish(msg);

  // get the target link orientation
  for(int i = 0; i < joint_index_map_.size(); i++) {
    auto id = joint_index_map_.at(i);
    current_joint_state.position.at(id) = target_angles.at(i);
  }

  robot_model_for_nav_->updateRobotModel(current_joint_state);
  const auto& target_seg_tf_map = robot_model_for_nav_->getSegmentsTf();
  for(int i = 0; i < tiger_robot_model_->getRotorNum(); i++) {
    std::string link_name = std::string("link") + std::to_string(i+1);
    KDL::Frame fr_target_link = target_seg_tf_map.at(link_name);
    KDL::Frame fb_target_link = fr_baselink.Inverse() * fr_target_link;
    KDL::Frame fw_target_link = fw_target_baselink * fb_target_link;
    target_link_rots_.at(i) = fw_target_link.M;
  }

}

void WalkNavigator::setJointIndexMap()
{
  const auto current_joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  const auto& search_v = current_joint_state.name;

  joint_index_map_.resize(0); // resize
  const auto& joint_names = target_joint_state_.name;
  for(const auto& n: joint_names) {

    auto res = std::find(search_v.begin(), search_v.end(), n);

    if (res == search_v.end()) {
      ROS_ERROR_STREAM("[Tiger][Navigator] joint index mapping, cannot find " << n);
      continue;
    }

    auto id = std::distance(search_v.begin(), res);

    joint_index_map_.push_back(id);
  }
}

std::vector<double> WalkNavigator::getCurrentJointAngles()
{
  const auto current_joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  std::vector<double> angles(0);
  for(const auto id: joint_index_map_) {
    angles.push_back(current_joint_state.position.at(id));
  }

  return angles;
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

void WalkNavigator::targetBaselinkPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  tf::vector3MsgToTF(msg->vector, target_baselink_pos_);
}

void WalkNavigator::targetBaselinkDeltaPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  tf::Vector3 delta_pos;
  tf::vector3MsgToTF(msg->vector, delta_pos);
  target_baselink_pos_ += delta_pos;

  ROS_ERROR("get new target baselink");
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::Tiger::WalkNavigator, aerial_robot_navigation::BaseNavigator);

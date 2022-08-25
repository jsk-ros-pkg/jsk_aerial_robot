#include <tiger/navigation/walk_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation::Tiger;

WalkNavigator::WalkNavigator():
  BaseNavigator(),
  target_baselink_pos_(0,0,0),
  target_baselink_vel_(0,0,0),
  target_baselink_rpy_(0,0,0),
  target_leg_ends_(0)
{
}

void WalkNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator);

  tiger_robot_model_ = boost::dynamic_pointer_cast<::Tiger::FullVectoringRobotModel>(robot_model);

  target_baselink_pos_sub_ = nh_.subscribe("walk/baselink/traget/pos", 1, &WalkNavigator::targetBaselinkPosCallback, this);
  target_baselink_delta_pos_sub_ = nh_.subscribe("walk/baselink/traget/delta_pos", 1, &WalkNavigator::targetBaselinkDeltaPosCallback, this);
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

  Eigen::VectorXd leg_angles = Eigen::VectorXd::Zero(3 * leg_num);
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
    // ROS_ERROR_STREAM("leg end" << i + 1 << ": " << x_e << ", " << y_e);
    double b = l1 * l1 - x_e * x_e - y_e * y_e - l2 * l2;
    double b_d = b / (- 2 * l2);
    double d = sqrt(x_e * x_e + y_e * y_e);
    double b_dd = b_d / d;
    if (fabs(b_dd) > 1) {
      ROS_ERROR_STREAM("[Tiger][Navigator] leg " << i + 1 << ": b_dd exceeds the valid scope: " << b_dd);
      leg_angles.segment(3 * i, 3) = Eigen::Vector3d::Zero();
      continue;
    }
    double phi = atan2(y_e, x_e);
    double theta2_d = acos(b_dd) + phi;
    if (theta2_d < 0) {
      theta2_d = - acos(b_dd) + phi;
    }
    double theta1 = atan2(y_e - l2 * sin(theta2_d), x_e - l2 * cos(theta2_d));
    double theta2 = theta2_d - theta1;

    leg_angles.segment(3 * i, 3) = Eigen::Vector3d(angle, theta1, theta2);
  }

  ROS_INFO_STREAM_THROTTLE(1.0, "[Tiger][Navigator] analytical joint angles from leg end and baselink: " << leg_angles.transpose());


  if (getNaviState() == aerial_robot_navigation::START_STATE) {

    // set the target position for baselink
    ROS_INFO("[Walk][Navigator] set initial position and joint angles as target ones");
    target_baselink_pos_ = baselink_pos;
    target_baselink_rpy_ = baselink_rpy;

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

    // set target leg end frame (position)
    target_leg_ends_ = curr_leg_ends;
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

void WalkNavigator::targetBaselinkPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  tf::vector3MsgToTF(msg->vector, target_baselink_pos_);
}

void WalkNavigator::targetBaselinkDeltaPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  tf::Vector3 delta_pos;
  tf::vector3MsgToTF(msg->vector, delta_pos);
  target_baselink_pos_ += delta_pos;
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::Tiger::WalkNavigator, aerial_robot_navigation::BaseNavigator);

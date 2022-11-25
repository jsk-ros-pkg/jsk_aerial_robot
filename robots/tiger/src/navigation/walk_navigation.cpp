#include <tiger/navigation/walk_navigation.h>
#include <tiger/control/walk_control.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation::Tiger;

WalkNavigator::WalkNavigator():
  BaseNavigator(),
  joint_index_map_(0),
  target_baselink_pos_(0,0,0),
  target_baselink_vel_(0,0,0),
  target_baselink_rpy_(0,0,0),
  target_leg_ends_(0),
  target_link_rots_(0),
  reset_baselink_flag_(false),
  reset_leg_ends_flag_(false),
  free_leg_id_(-1),
  raise_leg_flag_(false),
  lower_leg_flag_(false),
  walk_flag_(false),
  walk_leg_id_(-1),
  leg_motion_phase_(WalkPattern::PHASE0)
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

  raise_leg_sub_ = nh_.subscribe("walk/raise_leg", 1, &WalkNavigator::raiseLegCallback, this);
  lower_leg_sub_ = nh_.subscribe("walk/lower_leg", 1, &WalkNavigator::lowerLegCallback, this);
  walk_sub_ = nh_.subscribe("walk/enable", 1, &WalkNavigator::walkCallback, this);

  target_joint_angles_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/nav/target_joint_angles", 1); // for debug
  target_leg_ends_pub_ = nh_.advertise<geometry_msgs::PoseArray>("debug/nav/target_leg_ends", 1); // for debug

  if(walk_simulation_) {
    simulate_baselink_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("ground_truth", 1);
    simulate_joint_angles_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    simulate_flight_config_sub_ = nh_.subscribe("flight_config_cmd", 1, &WalkNavigator::simulateFlightConfigCallback, this);
    simulate_flight_config_pub_ = nh_.advertise<std_msgs::UInt8>("flight_config_ack", 1);

    simulated_joint_state_.position.resize(0);
    simulated_joint_state_.name.resize(0);

    XmlRpc::XmlRpcValue params;
    nh_.getParam("zeros", params);
    for(auto param: params) {
      // ROS_INFO_STREAM(param.first << ": " << param.second);
      simulated_joint_state_.name.push_back(param.first);
      simulated_joint_state_.position.push_back(param.second);
    }
    sensor_msgs::JointState joint_msg = simulated_joint_state_;
    joint_msg.header.stamp = ros::Time::now();
    for(int i = 0; i < tiger_robot_model_->getRotorNum(); i++) {
      joint_msg.name.push_back(std::string("gimbal") + std::to_string(i+1) + std::string("_roll"));
      joint_msg.name.push_back(std::string("gimbal") + std::to_string(i+1) + std::string("_pitch"));
      joint_msg.position.push_back(0);
      joint_msg.position.push_back(0);
    }
    simulate_joint_angles_pub_.publish(joint_msg);
  }
}

void WalkNavigator::walkPattern()
{
  /*
    simple walking pattern:
    - Straight line walk along x axis.
    - Every leg move along x axis with `walk_stride` at each step.
    - Leg will raise (raise inner joint pitch angle), and also move joint yaw angle at the same time.
    - Center link also move with `walk_stride`/`leg_num` at each step.
    - A circle means all leg finish move one time
   */

  if (!walk_flag_) {
    return;
  }

  switch(leg_motion_phase_) {
  case WalkPattern::PHASE0:
    {
      // all legs contact to the ground
      std::string prefix("[Tiger][Walk][Phase0]");

      // start raise leg mode for this leg
      raiseLeg(walk_leg_id_);
      if (walk_leg_id_ == 0 || walk_leg_id_ == 3) {
        // workaround: set larget raise angle for front legs
        raise_angle_ = raise_angle_orig_ * 2;
      }

      // update the foot postion, and thus the joint angles
      target_leg_ends_.at(walk_leg_id_).p += KDL::Vector(walk_stride_, 0, 0); // along x axis

      // reset the timestamp to check joint convergence
      converge_timestamp_ = ros::Time::now().toSec();

      // shift to PHASE1
      ROS_INFO_STREAM(prefix << " shift from PHASE0 to PHASE1 for leg" << walk_leg_id_ + 1);
      leg_motion_phase_ = WalkPattern::PHASE1;

      break;
    }
  case WalkPattern::PHASE1:
    {
      // free leg is raising
      std::string prefix("[Tiger][Walk][Phase1]");

      // check the yaw joint of free leg to lower leg
      int j = 4 * walk_leg_id_;
      double target_angle = target_joint_state_.position.at(j);
      double current_angle = getCurrentJointAngles().at(j);
      double err = target_angle - current_angle;
      if (fabs(err) > move_leg_joint_err_thresh_) {
        // joint angle error too big

        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " the angle error of joint" << j/2 << "_yaw does not converge, " << err << "(" << move_leg_joint_err_thresh_ << ")");

        // reset the timestamp to check joint convergence
        converge_timestamp_ = ros::Time::now().toSec();

        break;
      }

      double t = ros::Time::now().toSec() - converge_timestamp_;
      if (t < walk_pattern_converge_du_) {
        // no reach enough convergence time
        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " not reach enough convergence time for joint, start from " << converge_timestamp_  << ", last " << t);
        break;
      }

      // reach the target joint yaw to lower leg
      raise_angle_ = raise_angle_orig_;
      lowerLeg();

      // reset the timestamp to check joint convergence
      converge_timestamp_ = ros::Time::now().toSec();

      // shift to PHASE2
      ROS_INFO_STREAM(prefix << " shift from PHASE1 to PHASE2 for leg" << walk_leg_id_ + 1);
      leg_motion_phase_ = WalkPattern::PHASE2;

      break;
    }
  case WalkPattern::PHASE2:
    {
      // free leg is lowering
      std::string prefix("[Tiger][Walk][Phase2]");

      if (lower_leg_flag_) {
        // still in lowering phase, skip
        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " still in lowering phase, skip");

        break;
      }

      if (walk_controller_->getContactTransition()) {
        // still in contact transition, skip
        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " finish lowering, but still in ground contact transition, skip");

        break;
      }

      if (walk_debug_) {
        if (walk_debug_legs_ == 0) {
          // only test single leg raise & lower

          // shift to PHASE0
          ROS_INFO_STREAM(prefix << " debug mode, finish walk");
          walk_flag_ = false;
          break;
        }
      }

      // move center link
      walk_move_baselink_ = false;
      if (walk_leg_id_ == 3) {
        // WIP: the leg selection.
        //      move baselink only after leg4 (all front legs are updated)
        // WIP: forward move
        double distance = walk_stride_;
        target_baselink_pos_ += tf::Vector3(distance, 0, 0);
        walk_move_baselink_ = true;
      }

      // shift to PHASE3
      ROS_INFO_STREAM(prefix << " shift from PHASE2 to PHASE3 for leg" << walk_leg_id_ + 1);
      leg_motion_phase_ = WalkPattern::PHASE3;

      break;
    }
  case WalkPattern::PHASE3:
    {
      // baselink is move
      std::string prefix("[Tiger][Walk][Phase3]");

      if (walk_move_baselink_) {

        // move center link
        tf::Vector3 baselink_pos = estimator_->getPos(Frame::BASELINK, estimate_mode_);
        double err = target_baselink_pos_.x() - baselink_pos.x();

        if (fabs(err) > baselink_converge_thresh_) {
          // no reach enough convergence position
          ROS_INFO_STREAM_THROTTLE(0.1, prefix << " no reach enough convergence position, target x: " << target_baselink_pos_.x() << ", current x: " << baselink_pos.x() << ", converge thresh: " << baselink_converge_thresh_);

          // reset the timestamp to check baselink convergence
          converge_timestamp_ = ros::Time::now().toSec();

          break;
        }

        double t = ros::Time::now().toSec() - converge_timestamp_;
        if (t < walk_pattern_converge_du_) {
          // no reach enough convergence time
          ROS_INFO_STREAM_THROTTLE(0.1, prefix << " not reach enough convergence time for baselink move, start from" << converge_timestamp_  << ", last " << t);
          break;
        }
      }

      ROS_INFO_STREAM(prefix << " finish move of leg" << walk_leg_id_ + 1 << " in walk cycle of " << walk_cycle_cnt_);

      // reset the target leg ends to further update the target joint angles
      if (walk_cycle_reset_leg_end_) {
        reset_leg_ends_flag_ = true;
      }

      // reset the target baselink to further update the target joint angles
      if (walk_cycle_reset_baselink_) {
        reset_baselink_flag_ = true;
      }

      // instant update to the target joint angle
      if (walk_simulation_) {
        tiger_robot_model_->updateRobotModel(target_joint_state_);

        for(int i = 0; i < simulated_joint_state_.position.size(); i++) {
          std::string name = simulated_joint_state_.name.at(i);
          auto res = std::find(target_joint_state_.name.begin(), target_joint_state_.name.end(), name);
          int j = std::distance(target_joint_state_.name.begin(), res);
          simulated_joint_state_.position.at(i) = target_joint_state_.position.at(j);
        }
      }

      // move next leg with following rule:
      // front-left (0) -> front-right (3) -> back-right (2) -> back-left (1)

      if (walk_debug_) {
        if ((walk_debug_legs_ == 1 && walk_leg_id_ == 0) ||
            (walk_debug_legs_ == 2 && walk_leg_id_ == 3) ||
            (walk_debug_legs_ == 3 && walk_leg_id_ == 2) ||
            (walk_debug_legs_ == 4 && walk_leg_id_ == 1)) {
          ROS_INFO_STREAM(prefix << " debug mode, finish walk");
          walk_flag_ = false;
          break;
        }
      }

      walk_leg_id_ --;
      if (walk_leg_id_ < 0) walk_leg_id_ += 4;

      // check whether finish one cycle
      if (walk_leg_id_ == 0) {
        walk_cycle_cnt_ ++;
      }
      if (walk_cycle_cnt_ == walk_total_cycle_) {
        ROS_INFO_STREAM(prefix << " finish walk with totla cycle of " << walk_total_cycle_);
        walk_flag_ = false;
      }

      leg_motion_phase_ = WalkPattern::PHASE0; // back to Phase0 to move next leg.

      break;
    }
  default:
    break;
  }
}

void WalkNavigator::resetWalkPattern()
{
  walk_leg_id_ = 0;
  walk_cycle_cnt_ = 0;
  leg_motion_phase_ = WalkPattern::PHASE0;
}

void WalkNavigator::update()
{
  if (getNaviState() == aerial_robot_navigation::START_STATE) {
    if(estimator_->getUnhealthLevel() == Sensor::UNHEALTH_LEVEL3){
      ROS_WARN("Sensor Unhealth, cannot arming");
      setNaviState(ARM_OFF_STATE);
      return;
    }
  }

  BaseNavigator::update();

  // skip before the model initialization
  if (tiger_robot_model_->getStaticVectoringF().size() == 0 ||
      tiger_robot_model_->getStaticJointT().size() == 0) {
    return;
  }

  walkPattern();

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

  auto current_joint_angles = getCurrentJointAngles();

  // target link rotation
  if (target_link_rots_.size() == 0) {
    target_link_rots_.resize(tiger_robot_model_->getRotorNum());
  }

  // get current leg position w.r.t. world frame
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

  // initialize targets
  if (target_leg_ends_.size() == 0) {
    reset_baselink_flag_ = true;
    reset_leg_ends_flag_ = true;
  }

  // reset targets
  if (getNaviState() == aerial_robot_navigation::START_STATE) {
    ROS_INFO("[Walk][Navigator] set initial position and joint angles as target ones");

    reset_baselink_flag_ = true;
    reset_leg_ends_flag_ = true;
  }

  // update target baselink pose
  if (reset_baselink_flag_) {
    resetTargetBaselink(baselink_pos, baselink_rpy);
    reset_baselink_flag_ = false;
  }

  // update target leg end frame (position)
  if (reset_leg_ends_flag_) {
    resetTargetLegEnds(curr_leg_ends);
    reset_leg_ends_flag_ = false;
  }


  // calculate the target joint angles from baselink and end position
  KDL::Frame fw_target_baselink;
  fw_target_baselink.M
    = KDL::Rotation::EulerZYX(target_baselink_rpy_.z(),
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

    // check validity
    double angle_limit = 1.65;
    if(fabs(angle) > angle_limit ) {
      ROS_ERROR_STREAM("[Tiger][Navigator] joint" << i * 2 + 1 << "_yaw exceeds the valid range, angle is " << angle);
      continue;
    }

    if(fabs(theta1) > angle_limit ) {
      ROS_ERROR_STREAM("[Tiger][Navigator] joint" << i * 2 + 1 << "_pitch exceeds the valid range, angle is " << theta1);
      continue;
    }

    if(fabs(theta2) > angle_limit) {
      ROS_ERROR_STREAM("[Tiger][Navigator] joint" << i * 2 + 2 << "_pitch exceeds the valid range, angle is " << theta2);
      continue;
    }


    // set the target joint angles
    if(names.at(4 * i) != std::string("joint") + std::to_string(2*i+1) + std::string("_yaw")) {
      ROS_ERROR_STREAM("[Tiger][Navigator] name order is different. ID" << i << " name is " << names.at(4 * i));
      continue;
    }

    // special process for raising leg
    if (i == free_leg_id_) {

      double current_angle = current_joint_angles.at(4 * i + 1);

      // raise leg
      if (raise_leg_flag_) {
        theta1 -= raise_angle_;

        ROS_WARN_STREAM_ONCE("[Tiger] leg" << i + 1 << " leave the ground, joint" << i * 2 +1 << "_pitch" << ", curr angle: " << current_angle << "; target angle: " << theta1);
      }

      // lower leg
      if (lower_leg_flag_) {

        bool contact = false;
        double t = ros::Time::now().toSec();
        static double start_t = t;
        static double prev_t = t;
        static double prev_v = current_angle;

        if (t - prev_t > check_interval_) {
          // touch detection by
          // - small delta angle around the touch-down target angle
          // - constant angle for a while
          if (theta1 - current_angle < lower_touchdown_thresh_ &&
              fabs(current_angle - prev_v) < constant_angle_thresh_) {
            if (t - start_t > converge_time_thresh_) {
              contact = true;
            }
          }
          else {
            start_t = t;
          }

          prev_v = current_angle;
          prev_t = t;
        }

        if (contact) {
          ROS_WARN_STREAM("[Tiger][Walk][Navigator] leg" << i + 1 << " touches the ground, joint" << i * 2 +1 << "_pitch" << ", curr angle: " << current_angle << "; target angle: " << theta1);
          contactLeg();
        }
      }
    }

    target_angles.at(4 * i) = angle;
    target_angles.at(4 * i + 1) = theta1;
    target_angles.at(4 * i + 3) = theta2;

    ss << "(" << target_angles.at(4 * i) << ", " << target_angles.at(4 * i + 1) << ", "
       << target_angles.at(4 * i + 2) << ", " << target_angles.at(4 * i + 3) << ") ";
  }
  ROS_DEBUG_STREAM_THROTTLE(1.0, "[Tiger][Walk][Navigator] analytical joint angles from leg end and baselink: " << ss.str());

  // get the target link orientation
  auto joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  for(int i = 0; i < joint_index_map_.size(); i++) {
    auto id = joint_index_map_.at(i);
    joint_state.position.at(id) = target_angles.at(i);
  }

  robot_model_for_nav_->updateRobotModel(joint_state);
  const auto& target_seg_tf_map = robot_model_for_nav_->getSegmentsTf();
  for(int i = 0; i < tiger_robot_model_->getRotorNum(); i++) {
    std::string link_name = std::string("link") + std::to_string(i+1);
    KDL::Frame fr_target_link = target_seg_tf_map.at(link_name);
    KDL::Frame fb_target_link = fr_baselink.Inverse() * fr_target_link;
    KDL::Frame fw_target_link = fw_target_baselink * fb_target_link;
    target_link_rots_.at(i) = fw_target_link.M;
  }

  failSafeAction();

  // publish for debug
  std_msgs::Float32MultiArray msg;
  for(int i = 0; i < target_angles.size(); i++) {
    msg.data.push_back(target_angles.at(i));
  }
  target_joint_angles_pub_.publish(msg);

  geometry_msgs::PoseArray poses_msg;
  poses_msg.header.stamp = ros::Time::now();
  for(const auto& f: target_leg_ends_) {
    geometry_msgs::Pose pose;
    tf::poseKDLToMsg(f, pose);
    poses_msg.poses.push_back(pose);
  }
  target_leg_ends_pub_.publish(poses_msg);

  // simulate walk pattern
  if(walk_simulation_) {
    simulate();
  }

  // ROS_INFO_THROTTLE(0.1, "leg1 pos: (%f, %f, %f), leg2 pos: (%f, %f, %f), leg3 pos: (%f, %f, %f), leg4 pos: (%f, %f, %f)",
  //                   target_leg_ends_.at(0).p.x(), target_leg_ends_.at(0).p.y(), target_leg_ends_.at(0).p.z(),
  //                   target_leg_ends_.at(1).p.x(), target_leg_ends_.at(1).p.y(), target_leg_ends_.at(1).p.z(),
  //                   target_leg_ends_.at(2).p.x(), target_leg_ends_.at(2).p.y(), target_leg_ends_.at(2).p.z(),
  //                   target_leg_ends_.at(3).p.x(), target_leg_ends_.at(3).p.y(), target_leg_ends_.at(3).p.z());
}

void WalkNavigator::simulate()
{
  // init baselink
  if (target_leg_ends_.at(0).p.z() < 0) {
    // set the link end conntacting the ground
    double z_offset = - target_leg_ends_.at(0).p.z();

    for (auto& f: target_leg_ends_) {
      f.p += KDL::Vector(0, 0, z_offset);
    }
    target_baselink_pos_ += tf::Vector3(0, 0, z_offset);
  }

  // baselink odometry
  nav_msgs::Odometry odom_msg;
  tf::pointTFToMsg(target_baselink_pos_, odom_msg.pose.pose.position);
  odom_msg.pose.pose.orientation
    = tf::createQuaternionMsgFromRollPitchYaw(target_baselink_rpy_.x(),
                                              target_baselink_rpy_.y(),
                                              target_baselink_rpy_.z());
  simulate_baselink_pose_pub_.publish(odom_msg);

  // joint states
  double r = simulated_joint_lpf_rate_;
  for(int i = 0; i < simulated_joint_state_.position.size(); i++) {
    std::string name = simulated_joint_state_.name.at(i);
    auto res = std::find(target_joint_state_.name.begin(), target_joint_state_.name.end(), name);
    int j = std::distance(target_joint_state_.name.begin(), res);
    double angle = target_joint_state_.position.at(j);
    simulated_joint_state_.position.at(i) =
      (1 - r) * simulated_joint_state_.position.at(i) + r * angle;
  }
  sensor_msgs::JointState joint_msg = simulated_joint_state_;
  joint_msg.header.stamp = ros::Time::now();
  for(int i = 0; i < tiger_robot_model_->getRotorNum(); i++) {
    joint_msg.name.push_back(std::string("gimbal") + std::to_string(i+1) + std::string("_roll"));
    joint_msg.name.push_back(std::string("gimbal") + std::to_string(i+1) + std::string("_pitch"));
    joint_msg.position.push_back(0);
    joint_msg.position.push_back(0);
  }

  simulate_joint_angles_pub_.publish(joint_msg);
}

void WalkNavigator::failSafeAction()
{
  // baselink rotation
  tf::Matrix3x3 rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
  tf::Vector3 zb = rot * tf::Vector3(0,0,1);
  double theta = atan2(sqrt(zb.x() * zb.x() + zb.y() * zb.y()), zb.z());

  if (theta > baselink_rot_thresh_){
    ROS_WARN_STREAM("[Tiger][Walk][Navigation] baselink rotation is abnormal, tool tiled: " << theta);

    if (raise_leg_flag_) {
      lowerLeg();
      ROS_WARN_STREAM("[Tiger][Walk][Navigation] instantly lower the raised leg" << free_leg_id_ + 1);
    }
  }

  // pitch joint of opposite of raise leg
  if (raise_leg_flag_) {
    int leg_num = tiger_robot_model_->getRotorNum() / 2;
    int leg_id = (free_leg_id_ + leg_num / 2) % leg_num;
    int j = 4 * leg_id + 1;
    double target_angle = target_joint_state_.position.at(j);
    double current_angle = getCurrentJointAngles().at(j);
    std::string name = target_joint_state_.name.at(j);

    //ROS_INFO_STREAM("[Tiger][Walk][Navigation] " << name << ", target angle  " << target_angle << ", current angle: " << current_angle);
    if (target_angle - current_angle > opposite_raise_leg_thresh_) {
      ROS_WARN_STREAM("[Tiger][Walk][Navigation] " << name << " is overload because of raising leg, target angle  " << target_angle << ", current angle: " << current_angle);
      ROS_WARN_STREAM("[Tiger][Walk][Navigation] instantly lower the raising leg" << free_leg_id_ + 1);
      lowerLeg();
    }
  }
}

void WalkNavigator::resetTargetBaselink(tf::Vector3 pos, tf::Vector3 rpy)
{
  target_baselink_pos_ = pos;
  target_baselink_rpy_ = rpy;
}

void WalkNavigator::resetTargetLegEnds(std::vector<KDL::Frame> frames)
{
  target_leg_ends_ = frames;
}


void WalkNavigator::setJointIndexMap()
{
  const auto joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  const auto& search_v = joint_state.name;

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
  const auto joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  std::vector<double> angles(0);
  for(const auto id: joint_index_map_) {
    angles.push_back(joint_state.position.at(id));
  }

  return angles;
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


void WalkNavigator::raiseLeg(int leg_id)
{
  free_leg_id_ = leg_id;

  raise_leg_flag_ = true;
  lower_leg_flag_ = false;

  tiger_robot_model_->setFreeleg(free_leg_id_);
  walk_controller_->startRaiseTransition();
}

void WalkNavigator::lowerLeg()
{
  lower_leg_flag_ = true;
  raise_leg_flag_ = false;
  walk_controller_->startLowerLeg();
}

void WalkNavigator::contactLeg()
{
  lower_leg_flag_ = false;
  raise_leg_flag_ = false;
  tiger_robot_model_->resetFreeleg();
  walk_controller_->startContactTransition(free_leg_id_);
  free_leg_id_ = -1;
}

void WalkNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();


  ros::NodeHandle nh_walk(nh_, "navigation/walk");
  getParam<double>(nh_walk, "raise_angle", raise_angle_, 0.2);
  raise_angle_orig_ = raise_angle_;
  getParam<double>(nh_walk, "lower_touchdown_thresh", lower_touchdown_thresh_, 0.05);
  getParam<double>(nh_walk, "constant_angle_thresh", constant_angle_thresh_, 0.02);
  getParam<double>(nh_walk, "check_interval", check_interval_, 0.1);
  getParam<double>(nh_walk, "converge_time_thresh", converge_time_thresh_, 0.5);
  getParam<bool>(nh_walk, "simulation", walk_simulation_, false);
  getParam<double>(nh_walk, "simulated_joint_lpf_rate", simulated_joint_lpf_rate_, 0.5);

  ros::NodeHandle nh_walk_pattern(nh_walk, "pattern");
  getParam<int>(nh_walk_pattern, "total_cycle", walk_total_cycle_, 1);
  getParam<double>(nh_walk_pattern, "stride", walk_stride_, 0.2);
  getParam<double>(nh_walk_pattern, "move_leg_joint_err_thresh", move_leg_joint_err_thresh_, 0.05);
  getParam<double>(nh_walk_pattern, "baselink_converge_thresh", baselink_converge_thresh_, 0.05);
  getParam<double>(nh_walk_pattern, "converge_du", walk_pattern_converge_du_, 0.5);
  getParam<bool>(nh_walk_pattern, "reset_leg_end", walk_cycle_reset_leg_end_, true);
  getParam<bool>(nh_walk_pattern, "reset_baselink", walk_cycle_reset_baselink_, false);
  getParam<bool>(nh_walk_pattern, "debug", walk_debug_, false);
  getParam<int>(nh_walk_pattern, "debug_legs", walk_debug_legs_, 0);


  ros::NodeHandle nh_failsafe(nh_walk, "failsafe");
  getParam<double>(nh_failsafe, "baselink_rot_thresh", baselink_rot_thresh_, 0.2);
  getParam<double>(nh_failsafe, "opposite_raise_leg_thresh", opposite_raise_leg_thresh_, 0.1);


  if (walk_simulation_) {
    // WIP: reset the servo angle bias for simulation
    ros::NodeHandle nh_control(nh_, "controller/walk");
    nh_control.setParam("servo_angle_bias", 0);
  }

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

void WalkNavigator::raiseLegCallback(const std_msgs::UInt8ConstPtr& msg)
{
  raiseLeg(msg->data);
}

void WalkNavigator::lowerLegCallback(const std_msgs::EmptyConstPtr& msg)
{
  lowerLeg();
}

void WalkNavigator::walkCallback(const std_msgs::BoolConstPtr& msg)
{
  resetWalkPattern();
  walk_flag_ = msg->data;

  if (!walk_flag_) {
    // abort process here

    if (raise_leg_flag_) {
      ROS_WARN_STREAM("[Tiger][Walk][Navigation] instantly lower the raised leg" << free_leg_id_ + 1 << " during the walk pattern");
      lowerLeg();
    }
  }
}

void WalkNavigator::simulateFlightConfigCallback(const spinal::FlightConfigCmdConstPtr& msg)
{
  std_msgs::UInt8 ack_msg;
  switch(msg->cmd)
    {
    case spinal::FlightConfigCmd::ARM_ON_CMD:
      ack_msg.data = spinal::FlightConfigCmd::ARM_ON_CMD;
      simulate_flight_config_pub_.publish(ack_msg);
      break;
    case spinal::FlightConfigCmd::ARM_OFF_CMD:
      ack_msg.data = spinal::FlightConfigCmd::ARM_OFF_CMD;
      simulate_flight_config_pub_.publish(ack_msg);
      break;
    case spinal::FlightConfigCmd::FORCE_LANDING_CMD:
      ack_msg.data = spinal::FlightConfigCmd::FORCE_LANDING_CMD;
      simulate_flight_config_pub_.publish(ack_msg);
      break;
    default:
      break;
    }
}

void WalkNavigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  sensor_msgs::Joy joy_cmd;
  if(joy_msg->axes.size() == PS3_AXES && joy_msg->buttons.size() == PS3_BUTTONS) {
      joy_cmd = (*joy_msg);
  }
  else if(joy_msg->axes.size() == PS4_AXES && joy_msg->buttons.size() == PS4_BUTTONS) {
      joy_cmd = ps4joyToPs3joyConvert(*joy_msg);
  }
  else {
      ROS_WARN("the joystick type is not supported (buttons: %d, axes: %d)", (int)joy_msg->buttons.size(), (int)joy_msg->axes.size());
      return;
  }

  static sensor_msgs::Joy prev_joy_cmd = joy_cmd;

  if(joy_cmd.buttons[PS3_BUTTON_REAR_RIGHT_2] == 1) {
    /* all joint servo */
    if (joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] == 1) {
      if (prev_joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] == 1) {
        prev_joy_cmd = joy_cmd;
        return;
      }

      /* servo on */
      ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joints/torque_enable");
      std_srvs::SetBool srv;
      srv.request.data = true;

      if (client.call(srv))
        ROS_INFO("[Tiger][Joy] enable alll joint torque");
      else
        ROS_ERROR("Failed to call service joints/torque_enable");

      client = nh_.serviceClient<std_srvs::SetBool>("gimbals/torque_enable");

      if (client.call(srv))
        ROS_INFO("[Tiger][Joy] enable alll gimbal torque");
      else
        ROS_ERROR("Failed to call service gimbals/torque_enable");

      prev_joy_cmd = joy_cmd;
      return;

    }

    if (joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] == 1) {
      if (prev_joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] == 1) {
        prev_joy_cmd = joy_cmd;
        return;
      }

      /* servo off */
      ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joints/torque_enable");
      std_srvs::SetBool srv;
      srv.request.data = false;

      if (client.call(srv))
        ROS_INFO("[Tiger][Joy] disable the all joint torque");
      else
        ROS_ERROR("Failed to call service joints/torque_enable");

      client = nh_.serviceClient<std_srvs::SetBool>("gimbals/torque_enable");

      if (client.call(srv))
        ROS_INFO("[Tiger][Joy] disable the all gimbal torque");
      else
        ROS_ERROR("Failed to call service gimbals/torque_enable");

      prev_joy_cmd = joy_cmd;
      return;
    }
  } else {
    /* joint pitch servo */
    if (joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] == 1) {
      if (prev_joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] == 1) {
        prev_joy_cmd = joy_cmd;
        return;
      }

      /* servo on */
      ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
      std_srvs::SetBool srv;
      srv.request.data = true;

      if (client.call(srv))
        ROS_INFO("[Tiger][Joy] enable the pitch joint torque");
      else
        ROS_ERROR("Failed to call service joint_pitch/torque_enable");

      prev_joy_cmd = joy_cmd;
      return;

    }

    if (joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] == 1) {
      if (prev_joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] == 1) {
        prev_joy_cmd = joy_cmd;
        return;
      }

      /* servo off */
      ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
      std_srvs::SetBool srv;
      srv.request.data = false;

      if (client.call(srv))
        ROS_INFO("[Tiger][Joy] disable the pitch joint torque");
      else
        ROS_ERROR("Failed to call service joint_pitch/torque_enable");

      prev_joy_cmd = joy_cmd;
      return;
    }
  }


  if(joy_cmd.buttons[PS3_BUTTON_REAR_LEFT_2] == 1) {
    /* walk test */

    if(joy_cmd.buttons[PS3_BUTTON_CROSS_UP] == 1) {
      if (!walk_flag_) {
        walk_flag_ = true;
        resetWalkPattern();
        ROS_INFO("[Tiger][Walk][Navigation][Joy] walk pattern start");
      }
      return;
    }

    if(joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] == 1) {

      // abort process here

      if (walk_flag_) {
        walk_flag_ = false;
        resetWalkPattern();
        ROS_INFO("[Tiger][Walk][Navigation][Joy] walk pattern abort");

        if (raise_leg_flag_) {
          ROS_WARN_STREAM("[Tiger][Walk][Navigation][Joy] instantly lower the raised leg"
                          << free_leg_id_ + 1 << " during the walk pattern");
          lowerLeg();
        }
      }
      return;
    }
  } else {

    /* raise/lower test */
    if(joy_cmd.buttons[PS3_BUTTON_CROSS_UP] == 1) {

      /* raise test */
      if (raise_leg_flag_) {
        return;
      }

      ROS_INFO("[Joy, raise leg1 up test]");
      raiseLeg(0);

      return;
    }

    /* lower test */
    if(joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] == 1) {
      if (lower_leg_flag_) {
        return;
      }

      if (!raise_leg_flag_) {
        return;
      }

      ROS_INFO("[Joy, lower leg1 down test]");
      lowerLeg();

      return;
    }

  }


  /* quick land command */
  if(joy_cmd.buttons[PS3_BUTTON_ACTION_SQUARE] == 1 || joy_cmd.buttons[PS3_BUTTON_CROSS_RIGHT] == 1){

    if(getNaviState() == LAND_STATE) return;

    if(getNaviState() == ARM_ON_STATE)
      {
        setNaviState(STOP_STATE);
        ROS_ERROR("Joy Conrol: not land, but disarm motors directly");
      }
    return;
  }

  BaseNavigator::joyStickControl(joy_msg);

  prev_joy_cmd = joy_cmd;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::Tiger::WalkNavigator, aerial_robot_navigation::BaseNavigator);

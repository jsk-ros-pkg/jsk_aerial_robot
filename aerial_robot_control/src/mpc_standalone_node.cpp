// -*- mode: c++ -*-

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <aerial_robot_control/control/fwddyn_mpc_control_problem.hpp>
#include <aerial_robot_dynamics/robot_model.h>
#include <aerial_robot_dynamics/robot_model_ros.h>

#include <random>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_standalone_node");
  ros::NodeHandle nh;

  aerial_robot_dynamics::PinocchioRobotModelRos pinocchio_robot_model_ros(nh);
  auto pinocchio_robot_model = std::make_shared<aerial_robot_dynamics::PinocchioRobotModel>(
      pinocchio_robot_model_ros.getPinocchioRobotModel()->getRobotDescription(),
      pinocchio_robot_model_ros.getPinocchioRobotModel()->getPinocchioRobotDescription(), true);
  auto pin_model = pinocchio_robot_model->getModel();
  const int rotor_num = pinocchio_robot_model->getRotorNum();
  const int nq = pin_model->nq;
  const int nv = pin_model->nv;
  std::cout << "[mpc_standalone] nq=" << nq << " nv=" << nv << " rotor_num=" << rotor_num << std::endl;

  // --- Load MPC parameters ---
  aerial_robot_control::FwddynMpcControlProblem::Parameters mpc_params;
  ros::NodeHandle mpc_nh(nh, "mpc");
  mpc_nh.param<int>("num_nodes", mpc_params.num_nodes, mpc_params.num_nodes);
  mpc_nh.param<int>("max_iter", mpc_params.max_iter, mpc_params.max_iter);
  mpc_nh.param<int>("max_init_iter", mpc_params.max_init_iter, mpc_params.max_init_iter);
  mpc_nh.param<double>("dt", mpc_params.dt, mpc_params.dt);
  mpc_nh.param<int>("num_threads", mpc_params.num_threads, mpc_params.num_threads);
  mpc_nh.param<int>("solver_type", (int&)mpc_params.solver_type, (int)mpc_params.solver_type);
  mpc_nh.param<double>("control_weight", mpc_params.control_weight, mpc_params.control_weight);
  mpc_nh.param<double>("thrust_reg_weight", mpc_params.thrust_reg_weight, mpc_params.thrust_reg_weight);
  mpc_nh.param<double>("thrust_barrier_weight", mpc_params.thrust_barrier_weight, mpc_params.thrust_barrier_weight);
  mpc_nh.param<double>("delta_thrust_max", mpc_params.delta_thrust_max, mpc_params.delta_thrust_max);

  // load locked joint names
  if (mpc_nh.hasParam("locked_joint_names"))
    mpc_nh.getParam("locked_joint_names", mpc_params.locked_joint_names);

  // CoM tracking weight (3 elements)
  std::vector<double> com_w;
  if (mpc_nh.getParam("com_track_weight", com_w))
  {
    if ((int)com_w.size() == 3)
    {
      Eigen::Vector3d com_track_weight = Eigen::Map<Eigen::Vector3d>(com_w.data());
      mpc_params.com_track_weight = com_track_weight;
    }
    else
      ROS_ERROR_STREAM("[mpc_standalone] com_track_weight size mismatch: expected 3, got " << com_w.size());
  }
  else
    ROS_ERROR("[mpc_standalone] Failed to load com_track_weight");

  // centroidal momentum weight (6 elements)
  std::vector<double> centroidal_momentum_w;
  if (mpc_nh.getParam("centroidal_momentum_weight", centroidal_momentum_w))
  {
    if ((int)centroidal_momentum_w.size() == 6)
    {
      Eigen::VectorXd centroidal_momentum_weight =
          Eigen::Map<Eigen::VectorXd>(centroidal_momentum_w.data(), centroidal_momentum_w.size());
      mpc_params.centroidal_momentum_weight = centroidal_momentum_weight;
    }
    else
      ROS_ERROR_STREAM("[mpc_standalone] centroidal_momentum_weight size mismatch: expected 6, got "
                       << centroidal_momentum_w.size());
  }
  else
    ROS_ERROR("[mpc_standalone] Failed to load centroidal_momentum_weight");

  // state weight (2*nv elements)
  std::vector<double> x_w;
  if (mpc_nh.getParam("x_state_weight", x_w))
  {
    if ((int)x_w.size() == 2 * nv)
    {
      Eigen::VectorXd x_state_weight = Eigen::Map<Eigen::VectorXd>(x_w.data(), x_w.size());
      mpc_params.x_state_weight = x_state_weight;
    }
    else
      ROS_ERROR_STREAM("[mpc_standalone] x_state_weight size mismatch: expected " << 2 * nv << ", got " << x_w.size());
  }
  else
    ROS_ERROR("[mpc_standalone] Failed to load x_state_weight (expected size %d)", 2 * nv);

  double default_joint_vel = 0.1;
  mpc_nh.param<double>("default_joint_vel", default_joint_vel, default_joint_vel);

  mpc_params.print();

  // --- Load x_ref ---
  std::vector<double> x_ref_vec;
  Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(nq + nv);
  if (mpc_nh.getParam("x_ref", x_ref_vec))
  {
    if ((int)x_ref_vec.size() == nq + nv)
    {
      x_ref = Eigen::Map<Eigen::VectorXd>(x_ref_vec.data(), x_ref_vec.size());
    }
    else
      ROS_ERROR_STREAM("[mpc_standalone] x_ref size mismatch: expected " << nq + nv << ", got " << x_ref_vec.size());
  }
  else
    ROS_ERROR("[mpc_standalone] Failed to get x_ref from parameter server (expected size %d)", nq + nv);

  x_ref(3) = 0.0;  // identity quaternion x=0
  x_ref(4) = 0.0;  // identity quaternion y=0
  x_ref(5) = 0.0;  // identity quaternion z=0
  x_ref(6) = 1.0;  // identity quaternion w=1

  // reference configuration for joint locking (locked joints are held at these q values)
  mpc_params.q_ref = x_ref.head(nq);

  // joint name -> pinocchio index maps (skip universe=0 and root_joint=1)
  std::map<std::string, int> joint_name_to_q_idx;
  std::map<std::string, int> joint_name_to_v_idx;
  for (pinocchio::JointIndex i = 2; i < (pinocchio::JointIndex)pin_model->njoints; i++)
  {
    const std::string& jname = pin_model->names[i];
    joint_name_to_q_idx[jname] = pin_model->joints[i].idx_q();
    joint_name_to_v_idx[jname] = pin_model->joints[i].idx_v();
  }

  // per-joint targets: initialized from x_ref, updated by subscriber (only mentioned joints updated)
  struct JointTarget
  {
    double position;
    double velocity;
  };
  std::map<std::string, JointTarget> joint_targets;
  for (const auto& [jname, q_idx] : joint_name_to_q_idx)
    joint_targets[jname] = { x_ref(q_idx), 0.0 };

  ros::Subscriber target_joint_state_sub = nh.subscribe<sensor_msgs::JointState>(
      "final_target_joint_state", 1, [&](const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
          auto it = joint_targets.find(msg->name[i]);
          if (it == joint_targets.end() || i >= msg->position.size())
            continue;
          it->second.position = msg->position[i];
          it->second.velocity =
              (i < msg->velocity.size() && std::isfinite(msg->velocity[i]) && msg->velocity[i] > 0.0) ?
                  msg->velocity[i] :
                  0.0;
        }
      });

  // Initial state
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nq + nv + rotor_num);
  x0.head(nq) = x_ref.head(nq);

  // CoM target (hover at 1 m height)
  pinocchio::Data pin_data(*pin_model);
  pinocchio::centerOfMass(*pin_model, pin_data, x0.head(nq));
  const Eigen::Vector3d com_target(pin_data.com[0](0), pin_data.com[0](1), 1.0);
  std::cout << "[mpc_standalone] Initial CoM: " << pin_data.com[0].transpose()
            << ", target CoM: " << com_target.transpose() << std::endl;

  // --- Initialize and initial-solve MPC ---
  aerial_robot_control::FwddynMpcControlProblem mpc_problem;
  mpc_problem.initialize(pinocchio_robot_model, mpc_params);
  mpc_problem.buildMPCProblem(x0, com_target, x_ref);
  std::cout << "[mpc_standalone] Initial state: " << x0.transpose() << std::endl;
  std::cout << "[mpc_standalone] Initial input guess: " << mpc_problem.us()[0].transpose() << std::endl;

  ROS_INFO("[mpc_standalone] Initial solve (%d iter)...", mpc_params.max_init_iter);
  bool solved = mpc_problem.solveMPC(mpc_params.max_init_iter, true, false);
  ROS_INFO_STREAM("[mpc_standalone] Initial solve: " << (solved ? "success" : "failure"));

  bool noise_enable;
  double noise_pos_std, noise_rpy_std, noise_vel_std, noise_angvel_std;
  double noise_joint_pos_std, noise_joint_vel_std;
  nh.param<bool>("mpc_standalone/noise/enable", noise_enable, false);
  nh.param<double>("mpc_standalone/noise/pos_std", noise_pos_std, 0.01);              // [m]
  nh.param<double>("mpc_standalone/noise/rpy_std", noise_rpy_std, 0.01);              // [rad]
  nh.param<double>("mpc_standalone/noise/vel_std", noise_vel_std, 0.01);              // [m/s]
  nh.param<double>("mpc_standalone/noise/angvel_std", noise_angvel_std, 0.01);        // [rad/s]
  nh.param<double>("mpc_standalone/noise/joint_pos_std", noise_joint_pos_std, 0.01);  // [rad]
  nh.param<double>("mpc_standalone/noise/joint_vel_std", noise_joint_vel_std, 0.01);  // [rad/s]

  std::default_random_engine rng(std::random_device{}());
  std::normal_distribution<double> ndist(0.0, 1.0);  // unit normal; scaled per component

  ROS_INFO("[mpc_standalone] noise: enable=%s  pos=%.4f rpy=%.4f vel=%.4f angvel=%.4f jpos=%.4f jvel=%.4f",
           noise_enable ? "true" : "false", noise_pos_std, noise_rpy_std, noise_vel_std, noise_angvel_std,
           noise_joint_pos_std, noise_joint_vel_std);

  std::string tf_ns = ros::this_node::getNamespace();
  if (!tf_ns.empty() && tf_ns[0] == '/')
    tf_ns = tf_ns.substr(1);

  // local pinocchio Data for CoM computation (separate from the shared data in robot_model)
  pinocchio::Data pin_data_local(*pin_model);

  tf::TransformBroadcaster tf_broadcaster;
  auto thrust_rate_pub = nh.advertise<std_msgs::Float64MultiArray>("mpc/thrust_rate", 10);
  auto target_cog_pos_pub = nh.advertise<geometry_msgs::PointStamped>("mpc_target_cog_pos", 10);
  auto traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("mpc/predicted_trajectory", 10);
  auto joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  std::vector<ros::Publisher> rotor_wrench_pubs;
  for (int i = 0; i < rotor_num; ++i)
  {
    rotor_wrench_pubs.push_back(
        nh.advertise<geometry_msgs::WrenchStamped>("mpc/rotor_" + std::to_string(i + 1) + "_wrench", 10));
  }

  ROS_INFO("[mpc_standalone] MPC loop at %.2f Hz (dt=%.3f s)", 1.0 / mpc_params.dt, mpc_params.dt);

  ros::Rate rate(1.0 / mpc_params.dt);
  while (ros::ok())
  {
    // advance simulated state with ideal model dynamics
    const auto& xs = mpc_problem.xs();
    const auto& us = mpc_problem.us();
    const auto u0 =
        !us.empty() ? us.at(0) : Eigen::VectorXd::Zero(rotor_num + nv);  // default to zero input if us is empty

    if (xs.size() >= 2)
    {
      if (!xs[1].array().isFinite().all())
        ROS_WARN_THROTTLE(1.0, "[mpc_standalone] NaN in xs[1], keeping previous x0");
      else
        x0 = mpc_problem.expandState(xs[1]);
    }
    if (!u0.array().isFinite().all())
      ROS_WARN_THROTTLE(1.0, "[mpc_standalone] NaN in us[0], ignoring control input for state update");

    // add zero-mean Gaussian noise to the state for robustness evaluation
    if (noise_enable)
    {
      // position
      for (int i = 0; i < 3; ++i)
        x0(i) += noise_pos_std * ndist(rng);
      // orientation: perturb as small rotation [droll, dpitch, dyaw], then re-normalise.
      // guard against zero-vector noise to avoid NaN from normalized()
      {
        Eigen::Vector3d angle_noise(noise_rpy_std * ndist(rng), noise_rpy_std * ndist(rng), noise_rpy_std * ndist(rng));
        const double angle = angle_noise.norm();
        Eigen::Quaterniond q(x0(6), x0(3), x0(4), x0(5));  // w, x, y, z
        if (angle > 1e-10)
          q = q * Eigen::Quaterniond(Eigen::AngleAxisd(angle, angle_noise / angle));
        q.normalize();
        x0(3) = q.x();
        x0(4) = q.y();
        x0(5) = q.z();
        x0(6) = q.w();
      }
      // linear velocity
      for (int i = 0; i < 3; ++i)
        x0(nq + i) += noise_vel_std * ndist(rng);
      // angular velocity
      for (int i = 0; i < 3; ++i)
        x0(nq + 3 + i) += noise_angvel_std * ndist(rng);
      // joint positions and velocities (skip free-flyer: q[7..nq), v[6..nv))
      for (int i = 7; i < nq; ++i)
        x0(i) += noise_joint_pos_std * ndist(rng);
      for (int i = 6; i < nv; ++i)
        x0(nq + i) += noise_joint_vel_std * ndist(rng);
    }

    mpc_problem.setInitialState(x0);

    // interpolate x_ref joint positions toward joint_targets (always runs for all joints)
    {
      const double dt = mpc_params.dt;
      for (const auto& [jname, target] : joint_targets)
      {
        const int q_idx = joint_name_to_q_idx.at(jname);
        const int v_idx = joint_name_to_v_idx.at(jname);
        const double vel = (target.velocity > 0.0) ? target.velocity : default_joint_vel;
        const double error = target.position - x_ref(q_idx);
        const double step = vel * dt;
        if (std::abs(error) <= step)
        {
          x_ref(q_idx) = target.position;
          x_ref(nq + v_idx) = 0.0;
        }
        else
        {
          const double sign = (error > 0.0) ? 1.0 : -1.0;
          x_ref(q_idx) += sign * step;
          x_ref(nq + v_idx) = sign * vel;
        }
      }
    }

    mpc_problem.slideHorizon();
    // uniform references (fixed CoM target / single x_ref) for all num_nodes+1 nodes
    const std::vector<Eigen::Vector3d> com_traj(mpc_params.num_nodes + 1, com_target);
    const std::vector<Eigen::VectorXd> x_ref_traj(mpc_params.num_nodes + 1, x_ref);
    mpc_problem.setReferences(com_traj, x_ref_traj);
    mpc_problem.solveMPC(mpc_params.max_iter, true, false);

    const ros::Time stamp = ros::Time::now();
    const auto& xs_new = mpc_problem.xs();
    const auto& us_new = mpc_problem.us();

    // TF: broadcast root frame (skip if x0 contains NaN/Inf)
    if (x0.head(7).array().isFinite().all())
    {
      tf::Transform root_transform;
      root_transform.setOrigin(tf::Vector3(x0(0), x0(1), x0(2)));
      tf::Quaternion root_quat(x0(3), x0(4), x0(5), x0(6));
      root_quat.normalize();
      root_transform.setRotation(root_quat);
      tf_broadcaster.sendTransform(tf::StampedTransform(root_transform, stamp, "world", tf_ns + "/root"));
    }
    else
      ROS_WARN_THROTTLE(1.0, "[mpc_standalone] NaN/Inf in x0, skipping root TF broadcast");

    // TF: broadcast predicted trajectory frames (skip individual frames that contain NaN)
    for (size_t i = 0; i < xs_new.size(); ++i)
    {
      if ((int)xs_new[i].size() < 7 || !xs_new[i].head(7).array().isFinite().all())
        continue;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(xs_new[i](0), xs_new[i](1), xs_new[i](2)));
      tf::Quaternion quat(xs_new[i](3), xs_new[i](4), xs_new[i](5), xs_new[i](6));
      quat.normalize();
      transform.setRotation(quat);
      tf_broadcaster.sendTransform(
          tf::StampedTransform(transform, stamp, "world", tf_ns + "/mpc_optimized_root_" + std::to_string(i)));
    }

    // joint_states (skip if x0 contains NaN/Inf to avoid publishing invalid joint positions)
    if (x0.array().isFinite().all())
    {
      sensor_msgs::JointState joint_state_msg;
      joint_state_msg.header.stamp = stamp;
      for (pinocchio::JointIndex i = 2; i < (pinocchio::JointIndex)pin_model->njoints; i++)
      {
        const std::string& joint_name = pin_model->names[i];
        const int q_idx = pin_model->joints[i].idx_q();
        const int v_idx = pin_model->joints[i].idx_v();
        joint_state_msg.name.push_back(joint_name);
        joint_state_msg.position.push_back(x0(q_idx));
        joint_state_msg.velocity.push_back(x0(nq + pin_model->joints[i].idx_v()));
        if (u0.size() == rotor_num + nv - 6)                           // ignore root
          joint_state_msg.effort.push_back(us[0](rotor_num + v_idx));  // skip rotor
      }
      joint_states_pub.publish(joint_state_msg);
    }
    else
      ROS_WARN_THROTTLE(1.0, "[mpc_standalone] NaN/Inf in x0, skipping joint_states publish");

    // optimized thrust rates
    if (!us_new.empty())
    {
      std_msgs::Float64MultiArray rate_msg;
      rate_msg.data.resize(rotor_num);
      for (int i = 0; i < rotor_num; ++i)
        rate_msg.data[i] = us_new[0](i);
      thrust_rate_pub.publish(rate_msg);
    }

    // optimized rotor wrenches
    for (int i = 0; i < rotor_num; ++i)
    {
      geometry_msgs::WrenchStamped wrench_msg;
      wrench_msg.header.stamp = stamp;
      wrench_msg.header.frame_id = tf_ns + "/thrust" + std::to_string(i + 1);
      wrench_msg.wrench.force.x = 0.0;
      wrench_msg.wrench.force.y = 0.0;
      wrench_msg.wrench.force.z = x0(nq + nv + i);  // thrust for rotor i
      wrench_msg.wrench.torque.x = 0.0;
      wrench_msg.wrench.torque.y = 0.0;
      wrench_msg.wrench.torque.z = pinocchio_robot_model->getRotorDirection(i) * pinocchio_robot_model->getMFRate() *
                                   x0(nq + nv + i);  // yaw torque for rotor i
      rotor_wrench_pubs[i].publish(wrench_msg);
    }

    // target CoM position
    {
      geometry_msgs::PointStamped pt;
      pt.header.stamp = stamp;
      pt.header.frame_id = "world";
      pt.point.x = com_target(0);
      pt.point.y = com_target(1);
      pt.point.z = com_target(2);
      target_cog_pos_pub.publish(pt);
    }

    // CoM trajectory: line strip of predicted CoM positions + sphere at current CoM.
    // Skip any state that contains NaN/Inf to prevent Ogre from crashing in rviz.
    if (x0.head(nq).array().isFinite().all())
    {
      visualization_msgs::MarkerArray markers;

      // compute current CoM from x0
      pinocchio::centerOfMass(*pin_model, pin_data_local, x0.head(nq), false);
      const Eigen::Vector3d cur_com = pin_data_local.com[0];

      visualization_msgs::Marker line;
      line.header.frame_id = "world";
      line.header.stamp = stamp;
      line.ns = "mpc_com_traj";
      line.id = 0;
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.action = visualization_msgs::Marker::ADD;
      line.scale.x = 0.01;
      line.color.r = 0.0f;
      line.color.g = 1.0f;
      line.color.b = 0.0f;
      line.color.a = 1.0f;
      for (const auto& x : xs_new)
      {
        if (!x.head(nq).array().isFinite().all())
          continue;
        pinocchio::centerOfMass(*pin_model, pin_data_local, x.head(nq), false);
        geometry_msgs::Point p;
        p.x = pin_data_local.com[0](0);
        p.y = pin_data_local.com[0](1);
        p.z = pin_data_local.com[0](2);
        line.points.push_back(p);
      }
      markers.markers.push_back(line);

      // current CoM as a red sphere
      visualization_msgs::Marker cur;
      cur.header = line.header;
      cur.ns = "mpc_com_traj";
      cur.id = 1;
      cur.type = visualization_msgs::Marker::SPHERE;
      cur.action = visualization_msgs::Marker::ADD;
      cur.pose.position.x = cur_com(0);
      cur.pose.position.y = cur_com(1);
      cur.pose.position.z = cur_com(2);
      cur.pose.orientation.w = 1.0;
      cur.scale.x = cur.scale.y = cur.scale.z = 0.08;
      cur.color.r = 1.0f;
      cur.color.g = 0.0f;
      cur.color.b = 0.0f;
      cur.color.a = 1.0f;
      markers.markers.push_back(cur);

      traj_marker_pub.publish(markers);
    }
    else
      ROS_WARN_THROTTLE(1.0, "[mpc_standalone] NaN/Inf in x0, skipping trajectory marker publish");

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}


// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <assemble_quadrotors/optimal_design.h>

int cnt = 0;

void calcRotorConfiguration(const std::vector<double>& x, const int unit_rotor_num, const double pos_bound, const double m_f_rate, std::vector<Eigen::Vector3d>& p, std::vector<Eigen::Vector3d>& u, std::vector<Eigen::Vector3d>& v, std::vector<double>& direct)
{
  p.clear();
  u.clear();
  v.clear();
  direct.clear();

  Eigen::Vector3d  unit1_center_pos(pos_bound, 0, 0);
  // x: [ang1, ang2] * n/2

  for(int i = 0; i < unit_rotor_num; i++) {
    double direct_i = std::pow(-1, i);
    Eigen::Vector3d p_i;
    switch(i) {
    case 0:
      p_i = unit1_center_pos + Eigen::Vector3d(-pos_bound/2, -pos_bound/2, 0);
      break;
    case 1:
      p_i = unit1_center_pos + Eigen::Vector3d(+pos_bound/2, -pos_bound/2, 0);
      break;
    case 2:
      p_i = unit1_center_pos + Eigen::Vector3d(+pos_bound/2, +pos_bound/2, 0);
      break;
    case 3:
      p_i = unit1_center_pos + Eigen::Vector3d(-pos_bound/2, +pos_bound/2, 0);
      break;
    default:
      break;
    }

    // http://fnorio.com/0098spherical_trigonometry1/spherical_trigonometry1.html
    double ang1 = x.at(2 *i);
    double ang2 = x.at(2 *i + 1);
    Eigen::Vector3d u_i(sin(ang1) * cos(ang2), sin(ang1) * sin(ang2), cos(ang1));
    Eigen::Vector3d v_i = p_i.cross(u_i) - m_f_rate * direct_i * u_i;
    //std::cout<< "m_f_rate: " << m_f_rate <<  "; pi: " << p_i.transpose() << "; u_i: " << u_i.transpose() << "; vi: " << v_i.transpose() << std::endl;
    p.push_back(p_i); // append rotor position
    u.push_back(u_i); // append rotor force normal
    v.push_back(v_i); // append rotor torque normal
    direct.push_back(direct_i); // append propeller rotating direction

    // the mirror rotor
    // simplest rule: rotate the unit around the z axis
    double direct_i_mirror = std::pow(-1, i); // append propeller rotating direction (same direction)
    Eigen::Vector3d p_i_mirror = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * p_i;
    Eigen::Vector3d u_i_mirror = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * u_i;
    Eigen::Vector3d v_i_mirror = p_i_mirror.cross(u_i_mirror) - m_f_rate * direct_i_mirror * u_i_mirror;
    p.push_back(p_i_mirror);
    u.push_back(u_i_mirror);
    v.push_back(v_i_mirror);
    direct.push_back(direct_i_mirror);
    //std::cout<< "m_f_rate: " << m_f_rate <<  "; pi: " << p_i_mirror.transpose() << "; u_i: " << u_i_mirror.transpose() << "; vi: " << v_i_mirror.transpose() << std::endl;
  }
}


// temporarily copy https://github.com/jsk-ros-pkg/jsk_aerial_robot/blob/master/aerial_robot_model/src/transformable_aerial_robot_model/stability.cpp#L33-L40
double calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk)
{
  Eigen::Vector3d uixuj = ui.cross(uj);
  if (uixuj.norm() < 0.00001) {
    return 0.0;
  }
  return uixuj.dot(uk) / uixuj.norm();
}

// copy and modified from https://github.com/jsk-ros-pkg/jsk_aerial_robot/blob/master/aerial_robot_model/src/transformable_aerial_robot_model/stability.cpp#L56-L86
double calcFeasibleControlFDists(const std::vector<Eigen::Vector3d>& u, const double thrust_max, const double mass)
{
  const int rotor_num = u.size();
  Eigen::VectorXd fc_f_dists = Eigen::VectorXd::Zero(rotor_num * (rotor_num - 1));
  Eigen::Vector3d gravity_force(0, 0, mass * 9.8); // considering the gravity

  int index = 0;
  for (int i = 0; i < rotor_num; ++i) {
    const Eigen::Vector3d& u_i = u.at(i);
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;
      const Eigen::Vector3d& u_j = u.at(j);
      double dist_ij = 0.0;

      if(u.at(i).cross(u.at(j)).norm() < 1e-5) {
        // assume u_i and u_j has same direction, so this is not plane can generate
        dist_ij = 1e6;
      }

      for (int k = 0; k < rotor_num; ++k) {
        if (i == k || j == k) continue;
        const Eigen::Vector3d& u_k = u.at(k);
        double u_triple_product = calcTripleProduct(u_i, u_j, u_k);
        dist_ij += std::max(0.0, u_triple_product * thrust_max);
      }
      Eigen::Vector3d uixuj = u_i.cross(u_j);
      fc_f_dists(index) = fabs(dist_ij - (uixuj.dot(gravity_force) / uixuj.norm())); // considering the gravity
      index++;
    }
  }
  return  fc_f_dists.minCoeff();
}

// copy and modified from https://github.com/jsk-ros-pkg/jsk_aerial_robot/blob/master/aerial_robot_model/src/transformable_aerial_robot_model/stability.cpp#L88-L114
double calcFeasibleControlTDists(const std::vector<Eigen::Vector3d>& v, const double thrust_max)
{
  const int rotor_num = v.size();
  Eigen::VectorXd fc_t_dists = Eigen::VectorXd::Zero(rotor_num * (rotor_num - 1));

  int index = 0;
  for (int i = 0; i < rotor_num; ++i) {
    const Eigen::Vector3d& v_i = v.at(i);
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;

      const Eigen::Vector3d& v_j = v.at(j);
      double dist_ij = 0.0;

      if(v.at(i).cross(v.at(j)).norm() < 1e-5) {
        // assume v_i and v_j has same direction, so this is not plane can generate
        dist_ij = 1e6;
      }
      else {
        for (int k = 0; k < rotor_num; ++k) {
          if (i == k || j == k) continue;
          const Eigen::Vector3d& v_k = v.at(k);
          double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
          dist_ij += std::max(0.0, v_triple_product * thrust_max);
        }
      }
      fc_t_dists(index) = dist_ij;
      index++;
    }
  }
  //std::cout << fc_t_dists.transpose() << std::endl;
  return fc_t_dists.minCoeff();
}


double objectiveFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  OptimalDesign *planner = reinterpret_cast<OptimalDesign*>(ptr);

  // get the p, u, v and direction
  std::vector<Eigen::Vector3d> p;
  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  std::vector<double> direct;
  calcRotorConfiguration(x, planner->unit_rotor_num_, planner->pos_bound_, planner->m_f_rate_, p, u, v, direct);

  // get the fc_f_min and fc_t_min
  double fc_f_min = calcFeasibleControlFDists(u, planner->max_thrust_, planner->unit_mass_ * planner->units_num_);
  double fc_t_min = calcFeasibleControlTDists(v, planner->max_thrust_);

  if (cnt % 10000 == 0) {
    std::stringstream ss;
    ss << "fc_f_min: " << fc_f_min << "; fc_t_min: " << fc_t_min;
    // ss << "; u: ";
    // for(auto& u_i : u) ss << u_i.transpose() << ", ";
    std::cout << ss.str() << std::endl;
  }

   cnt++;

  return planner->fc_f_min_weight_ * fc_f_min + planner->fc_t_min_weight_ * fc_t_min;
}


OptimalDesign::OptimalDesign(ros::NodeHandle nh, ros::NodeHandle nhp)
{
  // intialize variables
  nhp.param("unit_rotor_num", unit_rotor_num_, 4);
  nhp.param("unit_mass", unit_mass_, 0.5); // [kg]
  nhp.param("max_thrust", max_thrust_, 5.0); // [N]
  nhp.param("fc_fmin_weight", fc_f_min_weight_, 1.0);
  nhp.param("fc_tmin_weight", fc_t_min_weight_, 1.0);
  nhp.param("pos_bound", pos_bound_, 0.25); // [m]
  nhp.param("m_f_rate", m_f_rate_, 0.01); // [Nm/N]

  units_num_ = 2; // we only consider two units

  visualize_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rotor_configuration_markers", 1);
  feasible_control_info_pub_ = nh.advertise<aerial_robot_msgs::FeasibleControlConvexInfo>("feasible_control_convex_info", 1);

  ros::Duration(1.0).sleep();


  // design the optimiazation problem
  // x: [ang1, ang2] * n
  // p_x: fixed
  // n: the number of rotors in unit module, which means the half of the two-assemble model
  // p_x, p_y, p_z: 3D position
  // ang1, ang2: 2D angles describing the rotor normal
  nlopt::opt optimizer_solver(nlopt::GN_ISRES, 2 * unit_rotor_num_); // chose the proper optimization model
  optimizer_solver.set_max_objective(objectiveFunc, this); // register "this" as the second arg in objectiveFunc

  // set bounds
  std::vector<double> lb(2 * unit_rotor_num_);
  std::vector<double> ub(2 * unit_rotor_num_);
  for(int i = 0; i < unit_rotor_num_; i++) {
    lb.at(2 * i) = 0; // lower bound of ang1
    ub.at(2 * i) = 0.4; // 0.4rad = 20deg upper bound of ang1 => no downward rotor

    lb.at(2 * i + 1) = 0; // lower bound of ang1
    ub.at(2 * i + 1) = 2 * M_PI; // upper bound of ang1
  }
  optimizer_solver.set_lower_bounds(lb);
  optimizer_solver.set_upper_bounds(ub);

  optimizer_solver.set_xtol_rel(1e-4); //1e-4
  int max_eval;
  nhp.param("max_eval", max_eval, 100000);
  optimizer_solver.set_maxeval(max_eval);
  double max_val;
  std::vector<double> opt_x(2 * unit_rotor_num_, 0.01);

  bool search_flag;
  nhp.param("search_flag", search_flag, true);
  if (search_flag) {
      nlopt::result result = optimizer_solver.optimize(opt_x, max_val);
      if (result != nlopt::SUCCESS)
        ROS_WARN_STREAM("the optimize solution does not succeed, result is " << result);
    }

  std::stringstream ss;
  ss << "opt x: ";
  for(auto& x_i: opt_x) ss << x_i << ", ";
  std::cout << ss.str() << std::endl;

  // get the p, u, v and direction
  std::vector<Eigen::Vector3d> p;
  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  std::vector<double> direct;
  calcRotorConfiguration(opt_x, unit_rotor_num_, pos_bound_, m_f_rate_, p, u, v, direct);

  // get the fc_f_min and fc_t_min
  double fc_f_min = calcFeasibleControlFDists(u, max_thrust_, unit_mass_ * units_num_);
  double fc_t_min = calcFeasibleControlTDists(v, max_thrust_);


  ss << "final fc_f_min: " << fc_f_min << "; fc_t_min: " << fc_t_min;
  ss << "; u: ";
  for(auto& u_i : u) ss << u_i.transpose() << ", ";

  std::cout << ss.str() << std::endl;


  aerial_robot_msgs::FeasibleControlConvexInfo msg;
  msg.header.stamp = ros::Time::now();

  for(const auto& u_i: u) {
    geometry_msgs::Vector3 vec;
    vec.x = u_i.x() * max_thrust_;
    vec.y = u_i.y() * max_thrust_;
    vec.z = u_i.z() * max_thrust_;
    msg.u.push_back(vec);
  }
  msg.fc_f_min = fc_f_min;

  for(const auto& v_i: v) {
    geometry_msgs::Vector3 vec;
    vec.x = v_i.x() * max_thrust_;
    vec.y = v_i.y() * max_thrust_;
    vec.z = v_i.z() * max_thrust_;
    msg.v.push_back(vec);
  }
  msg.fc_t_min = fc_t_min;

  msg.total_mass = units_num_ * unit_mass_;

  feasible_control_info_pub_.publish(msg);


  visualization_msgs::MarkerArray marker_msg;
  for (int j = 0; j < unit_rotor_num_ * 2; j++) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = std::string("world");
    marker.ns = std::string("drone");
    marker.id = 3 * j;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);
    point.x = p.at(j).x();
    point.y = p.at(j).y();
    point.z = p.at(j).z();
    marker.points.push_back(point);
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker_msg.markers.push_back(marker);

    marker.id = 3 * j + 1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = p.at(j).x();
    marker.pose.position.y = p.at(j).y();
    marker.pose.position.z = p.at(j).z();
    double pitch = atan2(sqrt(u.at(j).x() * u.at(j).x() + u.at(j).y() * u.at(j).y()), u.at(j).z());
    double yaw = atan2(u.at(j).y(), u.at(j).x());
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pitch, yaw);
    marker.scale.x = pos_bound_/2;
    marker.scale.y = pos_bound_/2;
    marker.scale.z = 0.02;
    marker.color.b = 1.0;
    marker.color.r = 0.0;
    marker.color.a = 1.0;
    marker_msg.markers.push_back(marker);

    marker.id = 3 * j + 2;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = p.at(j).x() + u.at(j).x() * 0.01;
    marker.pose.position.y = p.at(j).y() + u.at(j).y() * 0.01;
    marker.pose.position.z = p.at(j).z() + u.at(j).z() * 0.01;
    pitch = atan2(sqrt(u.at(j).x() * u.at(j).x() + u.at(j).y() * u.at(j).y()), u.at(j).z());
    yaw = atan2(u.at(j).y(), u.at(j).x());
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pitch, yaw);
    marker.scale.x = pos_bound_/10;
    marker.scale.y = pos_bound_/10;
    marker.scale.z = 0.01;
    marker.color.b = 1.0;
    marker.color.r = 0.0;
    marker.color.a = 1.0;
    marker_msg.markers.push_back(marker);

  }
  visualize_pub_.publish(marker_msg);

  ros::Duration(1.0).sleep();
}


int main (int argc, char **argv)
{
  ros::init (argc, argv, "aeria_robot_base");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~"); // node handle with private namespace

  OptimalDesign optimizer(nh, nhp);

  return 0;
}

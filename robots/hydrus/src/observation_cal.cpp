#include <hydrus/observation_cal.h>

ObstacleCalculator::ObstacleCalculator(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh), call_(0) {

  std::string file;
  double shift_x, shift_y;
  pnh_.getParam("cfg_path", file);
  pnh_.getParam("shift_x", shift_x);
  pnh_.getParam("shift_y", shift_y);
  //   file = file + ".csv";
  std::cout << "file name is " << file << std::endl;
  std::ifstream ifs(file);
  if (!ifs) {
    std::cerr << "cannot open file" << std::endl;
    std::exit(1);
  }
  std::string line;

  while (std::getline(ifs, line)) {
    std::vector<std::string> strvec = split(line, ',');
    Eigen::Vector3d tree_pos;
    tree_pos(0) = stof(strvec.at(1))+shift_x;
    // std::cout << "shift_x is: " << shift_x << std::endl;
    // std::cout << "tree_pos(0) is: " << tree_pos(0) << std::endl;
    tree_pos(1) = stof(strvec.at(2))+shift_y;
    tree_pos(2) = 0;
    positions_.push_back(tree_pos);
    radius_list_.push_back(stof(strvec.at(8)));
  }

  odom_sub_ = nh_.subscribe("/hydrus/uav/cog/odom", 1,
                            &ObstacleCalculator::CalculatorCallback, this);
  obs_pub_ = nh_.advertise<aerial_robot_msgs::ObstacleArray>(
      "/hydrus/polar_pixel", 1);

  theta_list_ = {5,15,25,35,45,60,75,90,105, 120, 134};//should change Theta_Cuts if you change this theta's num
  acc_theta_list_ = {1,4,7,10};
  // phi_list_ = {5};

  hydrus_theta_ = {90,90,90};
  hydrus_l_ = 0.6;
  hydrus_r_ = 0.2;

  set_collision_point();
}

void ObstacleCalculator::CalculatorCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {

  Eigen::Vector3d pos;
  tf::pointMsgToEigen(msg->pose.pose.position, pos);
  Eigen::Quaternion<Scalar> quat;
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, quat);
  Eigen::Matrix3d R = quat.toRotationMatrix();
  Eigen::Matrix3d R_T = R.transpose();
  Eigen::Vector3d poll_v(R_T(0, 2), R_T(1, 2), R_T(2, 2));

  Eigen::Vector3d vel;
  tf::vectorMsgToEigen(msg->twist.twist.linear, vel);
  Eigen::Vector3d omega;
  tf::vectorMsgToEigen(msg->twist.twist.angular, omega);

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> converted_positions;

  for (const auto &tree_pos : positions_) {
    Eigen::Vector3d converted_pos = R_T * (tree_pos - pos);
    converted_positions.push_back(converted_pos);
  }

  std::vector<Scalar> sphericalboxel =
      getsphericalboxel(converted_positions, poll_v, theta_list_);

  std::vector<Scalar> acc_sphericalboxel =
      getsphericalboxel(converted_positions, poll_v, acc_theta_list_);

  get_hydrus_sphericalboxel(converted_positions, poll_v, R_T, vel, omega);

  aerial_robot_msgs::ObstacleArray obstacle_msg;
  //   obstacle_msg.header.stamp = ros::Time(state.t);

  obstacle_msg.header = msg->header;
  for (size_t i = 0; i < sphericalboxel.size(); ++i) {
    obstacle_msg.boxel.push_back(sphericalboxel[i]);
  }
  for (size_t i = 0; i < acc_sphericalboxel.size(); ++i) {
    obstacle_msg.acc_boxel.push_back(acc_sphericalboxel[i]);
  }
  for (size_t i = 0; i < Vision::Corner_Num; ++i) {
    obstacle_msg.C_vel_obs.push_back(C_vel_obs_distance_[i]);
  }
  for (size_t i = 0; i < Vision::Rotor_Num; ++i) {
    obstacle_msg.R_vel_obs.push_back(R_vel_obs_distance_[i]);
  }

  obs_pub_.publish(obstacle_msg);

  //   if (call == 400) {
  //     std::cout << "v: " << std::endl;
  //     std::cout << v << std::endl;
  //     call = 0;
  //   } else {
  //     call += 1;
  //   }
}

std::vector<Scalar> ObstacleCalculator::getsphericalboxel(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &converted_positions,
    const Eigen::Vector3d &v, const std::vector<Scalar> &theta_list) {
  std::vector<Scalar> obstacle_obs;
  size_t size = theta_list.size();
  for (int t = -(int)size; t < (int)size; ++t) {
    Scalar theta = (t >= 0) ? theta_list[t] : -theta_list[(-t) - 1];  //[deg]
    // for (int p = -Vision::Phi_Cuts / 2; p < Vision::Phi_Cuts / 2; ++p) {
    //   Scalar phi = (p >= 0) ? phi_list_[p] : -phi_list_[(-p) - 1];  //[deg]

      Scalar tcell = theta * (M_PI / 180);
      // Scalar pcell = phi* (M_PI / 180);
      Scalar closest_distance = getClosestDistance(converted_positions, v, tcell, 0);
      obstacle_obs.push_back(closest_distance);
    // }
  }
  return obstacle_obs;
}

Scalar ObstacleCalculator::getClosestDistance(
    const std::vector<Vector<3>, Eigen::aligned_allocator<Vector<3>>> &converted_positions,
    const Eigen::Vector3d &v, Scalar tcell, Scalar fcell) {
  Eigen::Vector3d Cell = getCartesianFromAng(tcell, fcell);
  Scalar rmin = max_detection_range_;

  for (int i = 0; i < positions_.size(); i++) {
    Eigen::Vector3d pos = converted_positions[i];
    Scalar radius = radius_list_[i];
    Eigen::Vector3d alpha = Cell.cross(v);
    Eigen::Vector3d beta = pos.cross(v);
    Scalar a = std::pow(alpha.norm(), 2);
    if (a == 0) continue;
    Scalar b = alpha.dot(beta);
    Scalar c = std::pow(beta.norm(), 2) - std::pow(radius, 2);
    Scalar D = std::pow(b, 2) - a * c;
    if (0 <= D) {
      Scalar dist = (b - std::sqrt(D)) / a;
      if (0 < dist){
      rmin = std::min(dist, rmin);
      }
    }
  }
   return rmin / max_detection_range_;
}

Eigen::Vector3d ObstacleCalculator::getCartesianFromAng(Scalar theta,
                                                        Scalar phi) {
  Eigen::Vector3d cartesian(std::cos(theta) * std::cos(phi),
                            std::sin(theta) * std::cos(phi), std::sin(phi));
  return cartesian;
}

std::vector<std::string> ObstacleCalculator::split(std::string &input,
                                                   char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void ObstacleCalculator::get_hydrus_sphericalboxel(
  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &converted_positions,
  const Eigen::Vector3d &v, const Eigen::Matrix3d &R_T,
  const Eigen::Vector3d &vel, const Eigen::Vector3d &omega) {
  for (size_t i = 0; i < C_list_.size(); i++) {
    Vector<3> b_p_ref{C_list_[i](0), C_list_[i](1), 0};
    std::vector<Vector<3>, Eigen::aligned_allocator<Vector<3>>> pos_from_corner;
    for (Vector<3> pos : converted_positions) {
      pos_from_corner.push_back(pos - b_p_ref);
    }

    Eigen::Matrix3d R = R_T.transpose();
    Vector<3> w_vel =
      vel + omega.cross(R * b_p_ref);
    Vector<3> w_vel_2d = {w_vel[0], w_vel[1], 0};
    if (w_vel_2d.norm()>1E-3){
    Vector<3> body_vel = R_T * w_vel_2d;
    Scalar vel_theta = std::atan2(body_vel[1], body_vel[0]);
    Scalar vel_phi =
      std::atan(body_vel[2] /
                std::sqrt(std::pow(body_vel[0], 2) + std::pow(body_vel[1], 2)));

    C_vel_obs_distance_[i] =
      getClosestDistance(pos_from_corner, v, vel_theta, vel_phi) * max_detection_range_;
      // if (C_vel_obs_distance_[i]<-10000){
      //   ROS_INFO("body_vel: %lf,%lf,%lf",body_vel[0],body_vel[1],body_vel[2]);
      //   ROS_INFO("angle: %lf,%lf",vel_theta,vel_phi);
      //   ROS_INFO("C_vel_obs_distance_[i]: %lf",C_vel_obs_distance_[i]);
      }
  else{
    C_vel_obs_distance_[i] = max_detection_range_;
  }
  }
  for (size_t i = 0; i < R_list_.size(); i++) {
    Vector<3> b_p_ref{R_list_[i](0), R_list_[i](1), 0};
    std::vector<Vector<3>, Eigen::aligned_allocator<Vector<3>>> pos_from_corner;
    for (Vector<3> pos : converted_positions) {
      pos_from_corner.push_back(pos - b_p_ref);
    }

    Eigen::Matrix3d R = R_T.transpose();
    Vector<3> w_vel =
      vel + omega.cross(R * b_p_ref);
    Vector<3> w_vel_2d = {w_vel[0], w_vel[1], 0};

    if (w_vel_2d.norm()>1E-3){
    Vector<3> body_vel = R_T * w_vel_2d;
    Scalar vel_theta = std::atan2(body_vel[1], body_vel[0]);
    Scalar vel_phi =
      std::atan(body_vel[2] /
                std::sqrt(std::pow(body_vel[0], 2) + std::pow(body_vel[1], 2)));
    R_vel_obs_distance_[i] =
      getClosestDistance(pos_from_corner, v, vel_theta, vel_phi) * max_detection_range_;
    }
    else{
    R_vel_obs_distance_[i] = max_detection_range_;
    }
  }
  
}

bool ObstacleCalculator::set_collision_point() {
  Scalar theta_1 = hydrus_theta_[0] * M_PI / 180;
  Scalar theta_2 = hydrus_theta_[1] * M_PI / 180;
  Scalar theta_3 = hydrus_theta_[2] * M_PI / 180;

  Eigen::Vector2d bR2(0.0, 0.0);
  Eigen::Vector2d bC2(-hydrus_l_ / 2, 0.0);
  Eigen::Vector2d bC2TobR1(hydrus_l_ / 2 * (-std::cos(theta_1)),
                           hydrus_l_ / 2 * (std::sin(theta_1)));
  Eigen::Vector2d bR1 = bC2 + bC2TobR1;
  Eigen::Vector2d bC1 = bC2 + 2 * bC2TobR1;

  Eigen::Vector2d bC3(hydrus_l_ / 2, 0.0);
  Eigen::Vector2d bC3TobR3(hydrus_l_ / 2 * (std::cos(theta_2)),
                           hydrus_l_ / 2 * (std::sin(theta_2)));
  Eigen::Vector2d bR3 = bC3 + bC3TobR3;
  Eigen::Vector2d bC4 = bC3 + 2 * bC3TobR3;

  Eigen::Vector2d bC4TobR4(hydrus_l_ / 2 * (std::cos(theta_2 + theta_3)),
                           hydrus_l_ / 2 * (std::sin(theta_2 + theta_3)));
  Eigen::Vector2d bR4 = bC4 + bC4TobR4;
  Eigen::Vector2d bC5 = bC4 + 2 * bC4TobR4;

  Eigen::Vector2d CG = (bR1 + bR2 + bR3 + bR4) / 4;

  Eigen::Vector2d C1 = bC1 - CG;
  C_list_.push_back(C1);
  Eigen::Vector2d R1 = bR1 - CG;
  R_list_.push_back(R1);
  Eigen::Vector2d C2 = bC2 - CG;
  C_list_.push_back(C2);
  Eigen::Vector2d R2 = bR2 - CG;
  R_list_.push_back(R2);
  Eigen::Vector2d C3 = bC3 - CG;
  C_list_.push_back(C3);
  Eigen::Vector2d R3 = bR3 - CG;
  R_list_.push_back(R3);
  Eigen::Vector2d C4 = bC4 - CG;
  C_list_.push_back(C4);
  Eigen::Vector2d R4 = bR4 - CG;
  R_list_.push_back(R4);
  Eigen::Vector2d C5 = bC5 - CG;
  C_list_.push_back(C5);

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "observation_conversion");
  // The third argument to init() is the name of the node
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ObstacleCalculator calculator(nh, pnh);

  ros::spin();

  return 0;
}
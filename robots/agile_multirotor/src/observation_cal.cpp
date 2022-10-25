#include <observation_cal.h>

ObstacleCalculator::ObstacleCalculator(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh), call_(0) {

  std::string file;
  double spawn_x, spawn_y;
  pnh_.getParam("cfg_path", file);
  pnh_.getParam("spawn_x", spawn_x);
  pnh_.getParam("spawn_y", spawn_y);
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
    tree_pos(0) = stof(strvec.at(1))+spawn_x;
    // std::cout << "spawn_x is: " << spawn_x << std::endl;
    // std::cout << "tree_pos(0) is: " << tree_pos(0) << std::endl;
    tree_pos(1) = stof(strvec.at(2))+spawn_y;
    tree_pos(2) = 0;
    positions_.push_back(tree_pos);
    radius_list_.push_back(stof(strvec.at(8)));
  }

  odom_sub_ = nh_.subscribe("/multirotor/uav/cog/odom", 1,
                            &ObstacleCalculator::CalculatorCallback, this);
  obs_pub_ = nh_.advertise<aerial_robot_msgs::ObstacleArray>(
      "/multirotor/polar_pixel", 1);
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

  std::vector<Eigen::Vector3d> converted_positions;

  for (const auto &tree_pos : positions_) {
    Eigen::Vector3d converted_pos = R_T * (tree_pos - pos);
    converted_positions.push_back(converted_pos);
  }

  Vector<Cuts *Cuts> sphericalboxel =
      getsphericalboxel(converted_positions, poll_v);

  aerial_robot_msgs::ObstacleArray obstacle_msg;
  //   obstacle_msg.header.stamp = ros::Time(state.t);

  obstacle_msg.header = msg->header;
  for (size_t i = 0; i < Cuts * Cuts; ++i) {
    obstacle_msg.boxel.push_back(sphericalboxel[i]);
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

Vector<Cuts * Cuts> ObstacleCalculator::getsphericalboxel(
    const std::vector<Eigen::Vector3d> &converted_positions,
    const Eigen::Vector3d &v) {
  Vector<Cuts * Cuts> obstacle_obs;
  for (int t = -Cuts / 2; t < Cuts / 2; ++t) {
    for (int p = -Cuts / 2; p < Cuts / 2; ++p) {
      Scalar tcell = (t + 0.5) * (M_PI / 2 / Cuts);
      Scalar pcell = (p + 0.5) * (M_PI / 2 / Cuts);
      obstacle_obs[(t + Cuts / 2) * Cuts + (p + Cuts / 2)] =
          getClosestDistance(converted_positions, v, tcell, pcell);
    }
  }
  return obstacle_obs;
}

Scalar ObstacleCalculator::getClosestDistance(
    const std::vector<Eigen::Vector3d> &converted_positions,
    const Eigen::Vector3d &v, Scalar tcell, Scalar fcell) {
  Eigen::Vector3d Cell = getCartesianFromAng(tcell, fcell);
  Scalar rmin = max_detection_range_;
  Eigen::Vector3d pos;
  //   Scalar radius;
  Scalar radius = radius_list_[0];
  // object type is set to first object

  for (int i = 0; i < positions_.size(); i++) {
    pos = converted_positions[i];
    // radius = radius_list_[i];
    Eigen::Vector3d alpha = Cell.cross(v);
    Eigen::Vector3d beta = pos.cross(v);
    Scalar a = std::pow(alpha.norm(), 2);
    if (a == 0)
      continue;
    Scalar b = alpha.dot(beta);
    Scalar c = std::pow(beta.norm(), 2) - std::pow(radius, 2);
    Scalar D = std::pow(b, 2) - a * c;
    if (0 <= D) {
      Scalar dist = (b - std::sqrt(D)) / a;
      rmin = std::min(dist, rmin);
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "observation_conversion");
  // The third argument to init() is the name of the node
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ObstacleCalculator calculator(nh, pnh);

  ros::spin();

  return 0;
}
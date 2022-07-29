#include <observation_cal.h>

int getIndex(std::vector<std::string> v, std::string value) {
  for (int i = 0; i < v.size(); i++) {
    if (v[i].compare(value) == 0)
      return i;
  }
  return -1;
}

std::vector<std::string> split(std::string &input, char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void ObstacleCalculator::CalculatorCallback(
    const gazebo_msgs::ModelStates::ConstPtr &model_states) {
  // ROS_INFO("tree x: [%lf]", tree_model_pose.position.x);

  int drone_model_index = getIndex(model_states->name, "multirotor::root");
  // ROS_INFO("sub_target_tree1 index: [%d]", tree_model_index);
  geometry_msgs::Pose pose_msg = model_states->pose[drone_model_index];

  Eigen::Vector3d pos;
  tf::pointMsgToEigen(pose_msg.position, pos);
  //   Eigen::Vector3d quad_pos(pos_msg.position.x, pos_msg.position.y,
  //   pose_msg.position.z);
  Eigen::Quaternion<Scalar> quat;
  tf::quaternionMsgToEigen(pose_msg.orientation, quat);
  Eigen::Matrix3d R = quat.toRotationMatrix();
  Eigen::Matrix3d R_T = R.transpose();
  Eigen::Vector3d v(R_T(0, 2), R_T(1, 2), R_T(2, 2));

  std::vector<Eigen::Vector3d> converted_positions;

  for (const auto &tree_pos : positions) {
    Eigen::Vector3d converted_pos = R_T * (tree_pos - pos);
    converted_positions.push_back(converted_pos);
  }

  Vector<Cuts *Cuts> sphericalboxel = getsphericalboxel(converted_positions, v);

  //   if (call == 400) {
  //     std::cout << "v: " << std::endl;
  //     std::cout << v << std::endl;
  //     call = 0;
  //   } else {
  //     call += 1;
  //   }

  //     int tree_model_index = getIndex(model_states->name,
  //     "sub_target_tree1::cylinder1::link");
  // geometry_msgs::Quaternion    geometry_msgs::Pose tree_pose =
  // model_states->pose[tree_model_index];

  //     std_msgs::Bool msg;
  //     msg.data = is_collision;
  //     pub.publish(msg);
}

Vector<Cuts * Cuts> ObstacleCalculator::getsphericalboxel(
    const std::vector<Eigen::Vector3d> &converted_positions,
    const Eigen::Vector3d &v) {
  Vector<Cuts * Cuts> obstacle_obs;
  for (int t = -Cuts / 2; t < Cuts / 2; ++t) {
    for (int p = -Cuts / 2; p < Cuts / 2; ++p) {
      Scalar tcell = (t + 0.5) * (PI / 2 / Cuts);
      Scalar pcell = (p + 0.5) * (PI / 2 / Cuts);
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
  Scalar radius;

  for (int i = 0; i < N_obstacle; i++) {
    pos = converted_positions[i];
    radius = radius_list[i];
    Eigen::Vector3d alpha = Cell.cross(v);
    Eigen::Vector3d beta = pos.cross(v);
    Scalar a = std::pow(alpha.norm(), 2);
    if (a == 0)
      continue;
    Scalar b = alpha.dot(beta);
    Scalar c = std::pow(beta.norm(), 2) - std::pow(radius, 2);
    Scalar D = std::pow(b, 2) - 4 * a * c;
    if (0 <= D) {
      Scalar dist = (b - std::sqrt(D)) / (2 * a);
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

ObstacleCalculator::ObstacleCalculator() {
  // pub = nh.advertise<std_msgs::Bool>("/is_collision", 1);
  sub = nh.subscribe("/gazebo/link_states", 1,
                     &ObstacleCalculator::CalculatorCallback, this);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "observation_conversion");
  // The third argument to init() is the name of the node
  ros::NodeHandle pnh("~");
  call = 0;

  std::string file;
  pnh.getParam("cfg_path", file);
  //   file = file + ".csv";
  std::cout << "file name is " << file << std::endl;
  std::ifstream ifs(file);
  if (!ifs) {
    std::cerr << "cannot open file" << std::endl;
    std::exit(1);
  }
  std::string line;
  N_obstacle = 0;
  while (std::getline(ifs, line)) {
    std::vector<std::string> strvec = split(line, ',');
    Eigen::Vector3d tree_pos;
    tree_pos(0) = stof(strvec.at(1));
    tree_pos(1) = stof(strvec.at(2));
    tree_pos(2) = 0;
    positions.push_back(tree_pos);
    radius_list.push_back(stof(strvec.at(8)));
    N_obstacle++;
  }

  ObstacleCalculator calculator;
  while (ros::ok()) {
    // ROS_INFO("in while loop");
    ros::spinOnce();
  }

  return 0;
}
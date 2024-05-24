#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <string>
#include <vector>
// #include <tf/tf.h>
#include <Eigen/Core>
#include <cstdlib>
#include <eigen_conversions/eigen_msg.h>
// https://github.com/ros/geometry/blob/noetic-devel/eigen_conversions/include/eigen_conversions/eigen_msg.h
#include <aerial_robot_msgs/ObstacleArray.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <std_msgs/Float64MultiArray.h>

using Scalar = double;
static constexpr int Dynamic = Eigen::Dynamic;
template <int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template <int rows = Dynamic>
using Vector = Matrix<rows, 1>;


enum Vision : int {
  Theta_Cuts = 26,
  ACC_Theta_Cuts = 8,
  // Phi_Cuts = 2,
  Corner_Num = 5,
  Rotor_Num = 4,
};


class ObstacleCalculator {
public:
  ObstacleCalculator(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~ObstacleCalculator() {}

  template <typename T>
  T getsphericalboxel(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &converted_positions,
                    const Vector<3> &poll_y, const Eigen::Vector3d &poll_z, const std::vector<Scalar> &theta_list, Eigen::Vector3d &pos);
  template <typename T>
  T getsphericalboxel(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &converted_positions,
                    const Vector<3> &poll_y, const Eigen::Vector3d &poll_z, const std::vector<Scalar> &theta_list, Eigen::Vector3d &pos, Eigen::Vector3d &vel);
  Scalar
  getClosestDistance(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &converted_positions,
                     const Vector<3> &poll_y, const Eigen::Vector3d &poll_z, Scalar tcell, Scalar fcell, Eigen::Vector3d &quad_pos);

  void get_common_sphericalboxel(
  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &converted_positions,
  const Vector<3> &poll_y, const Eigen::Vector3d &poll_z, const Eigen::Matrix3d &R_T,const Eigen::Vector3d &vel, const Eigen::Vector3d &omega, Eigen::Vector3d &pos);
  Scalar calc_dist_from_wall(Scalar sign, const Vector<3>& Cell, const Vector<3> &poll_y, Eigen::Vector3d &quad_pos) const;

  Eigen::Vector3d getCartesianFromAng(Scalar t, Scalar f);
  bool set_collision_point();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber marker_sub_;
  ros::Subscriber record_sub_;
  ros::Publisher obs_pub_;
  ros::Publisher obs_min_dist_pub_;

  std::vector<Eigen::Vector3d> positions_;
  std::vector<Scalar> radius_list_;
  int call_;

  std::vector<Scalar> theta_list_, acc_theta_list_;
  // std::vector<Scalar> phi_list_;

  std::vector<Eigen::Vector2d> C_list_;
  std::vector<Eigen::Vector2d> R_list_;
  Vector<Corner_Num> C_vel_obs_distance_;
  Vector<Rotor_Num> R_vel_obs_distance_;

  std::vector<Scalar> common_theta_;
  Scalar common_l_;
  Scalar common_r_;

  Scalar wall_pos_;
  
  const Scalar max_detection_range_ = 10;
  bool print_yaw_;
  Scalar vel_calc_boundary_;

  double shift_x_, shift_y_;
  double body_r_;
  bool record_marker_;
  bool from_hokuyo_;
  bool have_hokuyo_data_;


  std::vector<std::string> split(std::string &input, char delimiter);

  void CalculatorCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void VisualizationMarkerCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);
  void RecordMarkerCallback(const std_msgs::Empty::ConstPtr &msg);
};

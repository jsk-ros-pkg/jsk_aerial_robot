#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
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
#include <sstream>
#include <std_msgs/Float64MultiArray.h>

using Scalar = double;
static constexpr int Dynamic = Eigen::Dynamic;
template <int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template <int rows = Dynamic> using Vector = Matrix<rows, 1>;

enum Vision : int {
  Theta_Cuts = 16,
  Phi_Cuts = 2,
};

Scalar max_detection_range_ = 20;

class ObstacleCalculator {
public:
  ObstacleCalculator(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~ObstacleCalculator() {}

  Vector<Vision::Theta_Cuts * Vision::Phi_Cuts>
  getsphericalboxel(const std::vector<Eigen::Vector3d> &converted_positions,
                    const Eigen::Vector3d &v);
  Scalar
  getClosestDistance(const std::vector<Eigen::Vector3d> &converted_positions,
                     const Eigen::Vector3d &v, Scalar tcell, Scalar fcell);

  Eigen::Vector3d getCartesianFromAng(Scalar t, Scalar f);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Publisher obs_pub_;

  std::vector<Eigen::Vector3d> positions_;
  std::vector<Scalar> radius_list_;
  int call_;

  std::vector<Scalar> theta_list_;
  std::vector<Scalar> phi_list_;

  std::vector<std::string> split(std::string &input, char delimiter);

  void CalculatorCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

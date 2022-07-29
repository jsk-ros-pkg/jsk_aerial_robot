#include <cmath>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <vector>
// #include <tf/tf.h>
#include <Eigen/Core>
#include <cstdlib>
#include <eigen_conversions/eigen_msg.h>
// https://github.com/ros/geometry/blob/noetic-devel/eigen_conversions/include/eigen_conversions/eigen_msg.h
#include <fstream>
#include <sstream>
#include <std_msgs/Float64MultiArray.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define PI 3.14159265359

using Scalar = double;
static constexpr int Dynamic = Eigen::Dynamic;
template <int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template <int rows = Dynamic> using Vector = Matrix<rows, 1>;

enum Vision : int {
  Cuts = 8,
};

Scalar max_detection_range_ = 20;

class ObstacleCalculator {
public:
  ObstacleCalculator();
  ~ObstacleCalculator() {}
  void CalculatorCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);

  Vector<Cuts * Cuts>
  getsphericalboxel(const std::vector<Eigen::Vector3d> &converted_positions,
                    const Eigen::Vector3d &v);
  Scalar
  getClosestDistance(const std::vector<Eigen::Vector3d> &converted_positions,
                     const Eigen::Vector3d &v, Scalar tcell, Scalar fcell);

  Eigen::Vector3d getCartesianFromAng(Scalar t, Scalar f);

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
};

std::vector<Eigen::Vector3d> positions;
std::vector<Scalar> radius_list;
int call;
int N_obstacle;

int getIndex(std::vector<std::string> v, std::string value);
std::vector<std::string> split(std::string &input, char delimiter);
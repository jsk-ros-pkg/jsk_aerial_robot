#ifndef MSG_UTILS_H
#define MSG_UTILS_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <Eigen/Dense>

namespace msg_utils {
  std_msgs::Float32MultiArray EigenMatrix2Float32MultiArray(const Eigen::MatrixXd& mat);

  Eigen::MatrixXd Float32MultiArray2EigenMatrix(const std_msgs::Float32MultiArrayConstPtr& msg);

  std_msgs::Float32MultiArray Vector2Float32MultiArray(const std::vector<double>& vec);

  std::vector<double> Float32MultiArray2Vector(const std_msgs::Float32MultiArrayConstPtr& msg);

}

#endif /* ifndef MSG_UTILS_H */

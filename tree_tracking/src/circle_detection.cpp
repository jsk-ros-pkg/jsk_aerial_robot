#include "tree_tracking/circle_detection.h"

namespace CircleDetection {
  void circleFitting(const std::vector<tf::Vector3>& points, tf::Vector3& tree_center_location, double& tree_radius, double& regulation)
  {
    double val[8] = {0,0,0,0,0,0,0,0};
    for (std::vector<tf::Vector3>::const_iterator it = points.begin(); it != points.end(); it++) {
      double x = it->getX(); double y = it->getY();
      val[0] += x;
      val[1] += y;
      val[2] += x * x;
      val[3] += y * y;
      val[4] += x * y;
      val[5] += -(x * x * x + x * y * y);
      val[6] += -(x * x * y + y * y * y);
    }
    val[7] = points.size();

    tf::Matrix3x3 m(val[2], val[4], val[0], val[4], val[3], val[1], val[0], val[1], val[7]);
    tf::Vector3 v(val[5], val[6], -val[2] - val[3]);
    tf::Vector3 solution = m.inverse() * v;
    tree_center_location.setValue(solution.x() * -0.5, solution.y() * -0.5, 0);
    tree_radius = std::sqrt(solution.x() * solution.x() / 4 + solution.y() * solution.y() / 4 - solution.z());
    double variation = 0.0;
    for (std::vector<tf::Vector3>::const_iterator it = points.begin(); it != points.end(); it++) {
      double dx = it->getX() - tree_center_location.x(); double dy = it->getY() - tree_center_location.y();
      double err = dx * dx + dy * dy - tree_radius * tree_radius;
      variation += err * err;
    }
    regulation = variation / (points.size() * tree_radius * tree_radius * tree_radius * tree_radius);
  }
}

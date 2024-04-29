#ifndef UTIL_CIRCLE_DETECTION_H
#define UTIL_CIRCLE_DETECTION_H

#include <tf/transform_broadcaster.h>
#include <vector>

namespace CircleDetection {
  void circleFitting(const std::vector<tf::Vector3>& points, tf::Vector3& tree_center_location, double& tree_radius, double& regulation);
}
#endif

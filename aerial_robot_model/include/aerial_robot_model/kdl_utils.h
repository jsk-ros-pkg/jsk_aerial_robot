#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_kdl.h>
#include <sensor_msgs/JointState.h>
#include <map>

namespace aerial_robot_model {
  inline bool isValidRotation(const KDL::Rotation& m)
  {
    double x, y, z, w;
    m.GetQuaternion(x, y, z, w);
    if(std::fabs(1 - Eigen::Quaterniond(w, x, y, z).squaredNorm() > 1e-6)) return true;
    else return false;
  }

  inline geometry_msgs::TransformStamped kdlToMsg(const KDL::Frame& frame)
  {
    return tf2::kdlToTransform(frame);
  }

  inline geometry_msgs::PointStamped kdlToMsg(const KDL::Vector& vector)
  {
    tf2::Stamped<KDL::Vector> kdl_vec;
    kdl_vec.setData(vector);
    geometry_msgs::PointStamped ps;
    tf2::convert(kdl_vec, ps);
    return ps;
  }

  inline Eigen::Affine3d kdlToEigen(const KDL::Frame& frame)
  {
    Eigen::Affine3d out;
    tf::transformKDLToEigen(frame, out);
    return out;
  }

  inline Eigen::Vector3d kdlToEigen(const KDL::Vector& vector)
  {
    Eigen::Vector3d out;
    tf::vectorKDLToEigen(vector, out);
    return out;
  }

  inline Eigen::Matrix3d kdlToEigen(const KDL::RotationalInertia& inertia)
  {
    return Eigen::Map<const Eigen::Matrix3d>(inertia.data);
  }

  inline tf2::Transform kdlToTf2(const KDL::Frame& frame)
  {
    tf2::Transform out;
    tf2::convert(tf2::kdlToTransform(frame).transform, out);
    return out;
  }

  inline tf2::Vector3 kdlToTf2(const KDL::Vector& vector)
  {
    tf2::Stamped<tf2::Vector3> tf2_vec;
    tf2::convert(kdlToMsg(vector), tf2_vec);
    return tf2_vec;
  }
} //namespace aerial_robot_model

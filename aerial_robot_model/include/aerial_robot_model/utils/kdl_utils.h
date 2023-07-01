#pragma once

#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_kdl.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <vector>
#include <kdl/rotationalinertia.hpp>

namespace aerial_robot_model {

  namespace {
    template <class T1, class T2, class Callback> std::vector<T1> convertVector(const std::vector<T2>& in, Callback callback)
    {
      std::vector<T1> out;
      out.reserve(in.size());
      for(const auto& elem : in)
        out.push_back(callback(elem));
      return out;
    }
  }

  inline bool isValidRotation(const KDL::Rotation& m)
  {
    double x, y, z, w;
    m.GetQuaternion(x, y, z, w);
    if(std::fabs(1 - Eigen::Quaterniond(w, x, y, z).squaredNorm())  < 1e-6) return true;
    else return false;
  }

  inline geometry_msgs::TransformStamped kdlToMsg(const KDL::Frame& in)
  {
    return tf2::kdlToTransform(in);
  }

  inline geometry_msgs::PointStamped kdlToMsg(const KDL::Vector& in)
  {
    tf2::Stamped<KDL::Vector> tmp;
    tmp.setData(in);
    geometry_msgs::PointStamped out;
    tf2::convert(tmp, out);
    return out;
  }

  inline std::vector<geometry_msgs::PointStamped> kdlToMsg(const std::vector<KDL::Vector>& in)
  {
    return convertVector<geometry_msgs::PointStamped, KDL::Vector>(in,
                                                                   [](const KDL::Vector& in)->geometry_msgs::PointStamped {
                                                                     return kdlToMsg(in);
                                                                   });
  }

  inline Eigen::Affine3d kdlToEigen(const KDL::Frame& in)
  {
    Eigen::Affine3d out;
    tf::transformKDLToEigen(in, out);
    return out;
  }

  inline Eigen::Vector3d kdlToEigen(const KDL::Vector& in)
  {
    Eigen::Vector3d out;
    tf::vectorKDLToEigen(in, out);
    return out;
  }

  inline Eigen::Matrix3d kdlToEigen(const KDL::RotationalInertia& in)
  {
    return Eigen::Map<const Eigen::Matrix3d>(in.data);
  }

  inline Eigen::Matrix3d kdlToEigen(const KDL::Rotation& in)
  {
    return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(in.data);
  }

  inline std::vector<Eigen::Vector3d> kdlToEigen(const std::vector<KDL::Vector>& in)
  {
    return convertVector<Eigen::Vector3d, KDL::Vector>(in,
                                                       [](const KDL::Vector& in)->Eigen::Vector3d {
                                                         return kdlToEigen(in);
                                                       });
  }

  inline std::vector<Eigen::Matrix3d> kdlToEigen(const std::vector<KDL::Rotation>& in)
  {
    return convertVector<Eigen::Matrix3d, KDL::Rotation>(in,
                                                         [](const KDL::Rotation& in)->Eigen::Matrix3d {
                                                           return kdlToEigen(in);
                                                         });
  }

  inline tf2::Transform kdlToTf2(const KDL::Frame& in)
  {
    tf2::Transform out;
    tf2::convert(tf2::kdlToTransform(in).transform, out);
    return out;
  }

  inline tf2::Vector3 kdlToTf2(const KDL::Vector& in)
  {
    tf2::Stamped<tf2::Vector3> out;
    tf2::convert(kdlToMsg(in), out);
    return out;
  }

  inline std::vector<tf2::Vector3> kdlToTf2(const std::vector<KDL::Vector>& in)
  {
    return convertVector<tf2::Vector3, KDL::Vector>(in,
                                                    [](const KDL::Vector& in)->tf2::Vector3 {
                                                      return kdlToTf2(in);
                                                    });
  }

} //namespace aerial_robot_model

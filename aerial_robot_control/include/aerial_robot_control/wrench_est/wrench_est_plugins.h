//
// Created by li-jinjie on 24-10-23.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_PLUGINS_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_PLUGINS_H

#include "aerial_robot_control/wrench_est/wrench_est_base.h"
#include "aerial_robot_control/nmpc/i_term.h"
#include <cmath>

namespace aerial_robot_control
{

class WrenchEstITerm : public WrenchEstBase
{
public:
  WrenchEstITerm() = default;

  void initialize(ros::NodeHandle nh, double ctrl_loop_du) override
  {
    setCtrlLoopDu(ctrl_loop_du);

    ros::NodeHandle i_term_nh(nh, "controller/i_term");

    double fx_limit, fy_limit, fz_limit, mx_limit, my_limit, mz_limit;
    getParam<double>(i_term_nh, "limit_fx", fx_limit, 5.0);
    getParam<double>(i_term_nh, "limit_fy", fy_limit, 5.0);
    getParam<double>(i_term_nh, "limit_fz", fz_limit, 5.0);
    getParam<double>(i_term_nh, "limit_mx", mx_limit, 1.0);
    getParam<double>(i_term_nh, "limit_my", my_limit, 1.0);
    getParam<double>(i_term_nh, "limit_mz", mz_limit, 1.0);

    double i_gain_x, i_gain_y, i_gain_z, i_gain_roll, i_gain_pitch, i_gain_yaw;
    getParam<double>(i_term_nh, "i_gain_x", i_gain_x, 1.0);
    getParam<double>(i_term_nh, "i_gain_y", i_gain_y, 1.0);
    getParam<double>(i_term_nh, "i_gain_z", i_gain_z, 1.0);
    getParam<double>(i_term_nh, "i_gain_roll", i_gain_roll, 0.5);
    getParam<double>(i_term_nh, "i_gain_pitch", i_gain_pitch, 0.5);
    getParam<double>(i_term_nh, "i_gain_yaw", i_gain_yaw, 0.5);

    double freq = 1.0 / getCtrlLoopDu();
    pos_i_term_[0].initialize(i_gain_x, fx_limit, freq);  // x
    pos_i_term_[1].initialize(i_gain_y, fy_limit, freq);  // y
    pos_i_term_[2].initialize(i_gain_z, fz_limit, freq);  // z

    pos_i_term_[3].initialize(i_gain_roll, mx_limit, freq);   // roll
    pos_i_term_[4].initialize(i_gain_pitch, my_limit, freq);  // pitch
    pos_i_term_[5].initialize(i_gain_yaw, mz_limit, freq);    // yaw
  }

  void update(const tf::Vector3& pos_ref, const tf::Quaternion& q_ref, const tf::Vector3& pos,
              const tf::Quaternion& q) override
  {
    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();

    double qwr = q_ref.w();
    double qxr = q_ref.x();
    double qyr = q_ref.y();
    double qzr = q_ref.z();

    double qe_w = qw * qwr + qx * qxr + qy * qyr + qz * qzr;
    double qe_x = qwr * qx - qw * qxr + qyr * qz - qy * qzr;
    double qe_y = qwr * qy - qw * qyr - qxr * qz + qx * qzr;
    double qe_z = qxr * qy - qx * qyr + qwr * qz - qw * qzr;

    double sign_qe_w = qe_w > 0 ? 1 : -1;

    double fx_w_i_term = pos_i_term_[0].update(pos.x() - pos_ref.x());
    double fy_w_i_term = pos_i_term_[1].update(pos.y() - pos_ref.y());
    double fz_w_i_term = pos_i_term_[2].update(pos.z() - pos_ref.z());
    double mx_cog_i_term = pos_i_term_[3].update(sign_qe_w * qe_x);
    double my_cog_i_term = pos_i_term_[4].update(sign_qe_w * qe_y);
    double mz_cog_i_term = pos_i_term_[5].update(sign_qe_w * qe_z);

    setDistForceW(fx_w_i_term, fy_w_i_term, fz_w_i_term);
    setDistTorqueCOG(mx_cog_i_term, my_cog_i_term, mz_cog_i_term);
  }

private:
  ITerm pos_i_term_[6]; // x, y, z, roll, pitch, yaw
};

class WrenchEstAcc : public aerial_robot_control::WrenchEstBase
{
public:
  WrenchEstAcc() = default;

  void initialize(ros::NodeHandle nh, double ctrl_loop_du) override
  {
    ROS_INFO("WrenchEstAcc initialize");
  }

  void update(const tf::Vector3& pos_ref, const tf::Quaternion& q_ref, const tf::Vector3& pos,
              const tf::Quaternion& q) override
  {
    // do nothing
  }

private:
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_PLUGINS_H

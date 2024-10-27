//
// Created by li-jinjie on 24-10-24.
//
#include "aerial_robot_control/wrench_est/wrench_est_i_term.h"
#include <ros/ros.h>

namespace aerial_robot_control
{

void WrenchEstITerm::initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator,
                                boost::shared_ptr<aerial_robot_navigation::BaseNavigator>& navigator,
                                double ctrl_loop_du)
{
  WrenchEstBase::initialize(nh, robot_model, estimator, navigator, ctrl_loop_du);

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

  /* init dynamic reconfigure */
  reconf_servers_.push_back(boost::make_shared<ITermDynamicConfig>(i_term_nh));
  reconf_servers_.back()->setCallback(boost::bind(&WrenchEstITerm::cfgCallback, this, _1, _2));
}

void WrenchEstITerm::update(const tf::Vector3& pos_ref, const tf::Quaternion& q_ref, const tf::Vector3& pos,
                            const tf::Quaternion& q)
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

void WrenchEstITerm::cfgCallback(ITermConfig& config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;

  if (config.i_gain_flag)
  {
    try
    {
      switch (level)
      {
        case Levels::RECONFIGURE_I_GAIN_X: {
          pos_i_term_[0].setIGain(config.i_gain_x);
          ROS_INFO_STREAM("change i_gain_x for '" << config.i_gain_x << "'");
          break;
        }
        case Levels::RECONFIGURE_I_GAIN_Y: {
          pos_i_term_[1].setIGain(config.i_gain_y);
          ROS_INFO_STREAM("change i_gain_y for '" << config.i_gain_y << "'");
          break;
        }
        case Levels::RECONFIGURE_I_GAIN_Z: {
          pos_i_term_[2].setIGain(config.i_gain_z);
          ROS_INFO_STREAM("change i_gain_z for '" << config.i_gain_z << "'");
          break;
        }
        case Levels::RECONFIGURE_I_GAIN_ROLL: {
          pos_i_term_[3].setIGain(config.i_gain_roll);
          ROS_INFO_STREAM("change i_gain_roll for '" << config.i_gain_roll << "'");
          break;
        }
        case Levels::RECONFIGURE_I_GAIN_PITCH: {
          pos_i_term_[4].setIGain(config.i_gain_pitch);
          ROS_INFO_STREAM("change i_gain_pitch for '" << config.i_gain_pitch << "'");
          break;
        }
        case Levels::RECONFIGURE_I_GAIN_YAW: {
          pos_i_term_[5].setIGain(config.i_gain_yaw);
          ROS_INFO_STREAM("change i_gain_yaw for '" << config.i_gain_yaw << "'");
          break;
        }
        default:
          break;
      }
    }
    catch (std::invalid_argument& e)
    {
      ROS_ERROR_STREAM("I Term config failed: " << e.what());
    }
  }
}

}  // namespace aerial_robot_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::WrenchEstITerm, aerial_robot_control::WrenchEstBase)
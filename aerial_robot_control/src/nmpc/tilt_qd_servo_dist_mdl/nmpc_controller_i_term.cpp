//
// Created by lijinjie on 23/11/29.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_controller_i_term.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoNMPCwITerm::initParams()
{
  TiltQdServoNMPC::initParams();
  wrench_est_loader_ptr_ = boost::make_shared<pluginlib::ClassLoader<aerial_robot_control::WrenchEstBase>>("aerial_robot_control", "aerial_robot_control::WrenchEstBase");

  try
  {
    wrench_est_ptr_ = wrench_est_loader_ptr_->createInstance("aerial_robot_control::WrenchEstITerm");
    wrench_est_ptr_->initialize(nh_, ctrl_loop_du_);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

}

void nmpc::TiltQdServoNMPCwITerm::calcDisturbWrench()
{
  /* get the current state */
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Quaternion q = estimator_->getQuat(Frame::COG, estimate_mode_);

  /* get the target state */
  tf::Vector3 target_pos;
  tf::Quaternion target_q;
  if (is_traj_tracking_)
  {
    target_pos.setX(x_u_ref_.x.data.at(0));
    target_pos.setY(x_u_ref_.x.data.at(1));
    target_pos.setZ(x_u_ref_.x.data.at(2));

    target_q.setW(x_u_ref_.x.data.at(6));
    target_q.setX(x_u_ref_.x.data.at(7));
    target_q.setY(x_u_ref_.x.data.at(8));
    target_q.setZ(x_u_ref_.x.data.at(9));
  }
  else
  {
    target_pos = navigator_->getTargetPos();

    tf::Vector3 target_rpy = navigator_->getTargetRPY();
    target_q.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
  }

  /* update I term */
  wrench_est_ptr_->update(target_pos, target_q, pos, q);

  /* get the disturbance wrench */
  dist_force_w_ = wrench_est_ptr_->getDistForceW();
  dist_torque_cog_ = wrench_est_ptr_->getDistTorqueCOG();
}

void nmpc::TiltQdServoNMPCwITerm::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
  TiltQdServoNMPC::cfgNMPCCallback(config, level);

  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;

  if (config.i_gain_flag)
  {
    try
    {
      switch (level)
      {
        case Levels::RECONFIGURE_NMPC_I_GAIN_X: {
          pos_i_term_[0].setIGain(config.i_gain_x);
          ROS_INFO_STREAM("change i_gain_x for NMPC '" << config.i_gain_x << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_I_GAIN_Y: {
          pos_i_term_[1].setIGain(config.i_gain_y);
          ROS_INFO_STREAM("change i_gain_y for NMPC '" << config.i_gain_y << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_I_GAIN_Z: {
          pos_i_term_[2].setIGain(config.i_gain_z);
          ROS_INFO_STREAM("change i_gain_z for NMPC '" << config.i_gain_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_I_GAIN_ROLL: {
          pos_i_term_[3].setIGain(config.i_gain_roll);
          ROS_INFO_STREAM("change i_gain_roll for NMPC '" << config.i_gain_roll << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_I_GAIN_PITCH: {
          pos_i_term_[4].setIGain(config.i_gain_pitch);
          ROS_INFO_STREAM("change i_gain_pitch for NMPC '" << config.i_gain_pitch << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_I_GAIN_YAW: {
          pos_i_term_[5].setIGain(config.i_gain_yaw);
          ROS_INFO_STREAM("change i_gain_yaw for NMPC '" << config.i_gain_yaw << "'");
          break;
        }
        default:
          break;
      }
    }
    catch (std::invalid_argument& e)
    {
      ROS_ERROR_STREAM("NMPC config failed: " << e.what());
    }
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltQdServoNMPCwITerm, aerial_robot_control::ControlBase);

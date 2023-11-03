#include <rolling/control/rolling_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::setControllerParams(std::string ns)
{
  ros::NodeHandle param_nh(nh_, ns);

  ros::NodeHandle xy_nh(param_nh, "xy");
  ros::NodeHandle x_nh(param_nh, "x");
  ros::NodeHandle y_nh(param_nh, "y");
  ros::NodeHandle z_nh(param_nh, "z");

  ros::NodeHandle roll_pitch_nh(param_nh, "roll_pitch");
  ros::NodeHandle roll_nh(param_nh, "roll");
  ros::NodeHandle pitch_nh(param_nh, "pitch");
  ros::NodeHandle yaw_nh(param_nh, "yaw");

  double limit_sum, limit_p, limit_i, limit_d;
  double limit_err_p, limit_err_i, limit_err_d;
  double p_gain, i_gain, d_gain;

  auto loadParam = [&, this](ros::NodeHandle nh)
                   {
                     getParam<double>(nh, "limit_sum", limit_sum, 1.0e6);
                     getParam<double>(nh, "limit_p", limit_p, 1.0e6);
                     getParam<double>(nh, "limit_i", limit_i, 1.0e6);
                     getParam<double>(nh, "limit_d", limit_d, 1.0e6);
                     getParam<double>(nh, "limit_err_p", limit_err_p, 1.0e6);
                     getParam<double>(nh, "limit_err_i", limit_err_i, 1.0e6);
                     getParam<double>(nh, "limit_err_d", limit_err_d, 1.0e6);

                     getParam<double>(nh, "p_gain", p_gain, 0.0);
                     getParam<double>(nh, "i_gain", i_gain, 0.0);
                     getParam<double>(nh, "d_gain", d_gain, 0.0);
                   };

  /* xy params */
  if(xy_nh.hasParam("p_gain"))
    {
      loadParam(xy_nh);
      pid_controllers_.at(X).setGains(p_gain, i_gain, d_gain);
      pid_controllers_.at(Y).setGains(p_gain, i_gain, d_gain);
    }
  else
    {
      loadParam(x_nh);
      pid_controllers_.at(X).setGains(p_gain, i_gain, d_gain);
      loadParam(y_nh);
      pid_controllers_.at(Y).setGains(p_gain, i_gain, d_gain);
    }

  /* z params */
  loadParam(z_nh);
  pid_controllers_.at(Z).setGains(p_gain, i_gain, d_gain);

  /* roll pitch params */
  if(roll_pitch_nh.hasParam("p_gain"))
    {
      loadParam(roll_pitch_nh);
      pid_controllers_.at(ROLL).setGains(p_gain, i_gain, d_gain);
      pid_controllers_.at(PITCH).setGains(p_gain, i_gain, d_gain);
    }
  else
    {
      loadParam(roll_nh);
      pid_controllers_.at(ROLL).setGains(p_gain, i_gain, d_gain);
      loadParam(pitch_nh);
      pid_controllers_.at(PITCH).setGains(p_gain, i_gain, d_gain);
    }

  /* yaw param */
  loadParam(yaw_nh);
  pid_controllers_.at(YAW).setGains(p_gain, i_gain, d_gain);
}


void RollingController::rosoutControlParams(std::string ns)
{
  ros::NodeHandle param_nh(nh_, ns);

  ros::NodeHandle xy_nh(param_nh, "xy");
  ros::NodeHandle x_nh(param_nh, "x");
  ros::NodeHandle y_nh(param_nh, "y");
  ros::NodeHandle z_nh(param_nh, "z");

  ros::NodeHandle roll_pitch_nh(param_nh, "roll_pitch");
  ros::NodeHandle roll_nh(param_nh, "roll");
  ros::NodeHandle pitch_nh(param_nh, "pitch");
  ros::NodeHandle yaw_nh(param_nh, "yaw");

  double p_gain, i_gain, d_gain;
  auto loadParam = [&, this](ros::NodeHandle nh)
                   {

                     getParam<double>(nh, "p_gain", p_gain, 0.0);
                     getParam<double>(nh, "i_gain", i_gain, 0.0);
                     getParam<double>(nh, "d_gain", d_gain, 0.0);
                   };

  /* xy params */
  if(xy_nh.hasParam("p_gain"))
    {
      loadParam(xy_nh);
      ROS_WARN_STREAM(ns << " param for xy: [ " <<  p_gain << ", " <<  i_gain << ", " << d_gain << "]");
    }
  else
    {
      loadParam(x_nh);
      ROS_WARN_STREAM(ns << " param for x: [ " <<  p_gain << ", " <<  i_gain << ", " << d_gain << "]");
      loadParam(y_nh);
      ROS_WARN_STREAM(ns << " param for y: [ " <<  p_gain << ", " <<  i_gain << ", " << d_gain << "]");
    }

  /* z params */
  loadParam(z_nh);
      ROS_WARN_STREAM(ns << " param for z: [ " <<  p_gain << ", " <<  i_gain << ", " << d_gain << "]");

  /* roll pitch params */
  if(roll_pitch_nh.hasParam("p_gain"))
    {
      loadParam(roll_pitch_nh);
      ROS_WARN_STREAM(ns << " param for RP: [ " <<  p_gain << ", " <<  i_gain << ", " << d_gain << "]");
    }
  else
    {
      loadParam(roll_nh);
      ROS_WARN_STREAM(ns << " param for R: [ " <<  p_gain << ", " <<  i_gain << ", " << d_gain << "]");
      loadParam(pitch_nh);
      ROS_WARN_STREAM(ns << " param for P: [ " <<  p_gain << ", " <<  i_gain << ", " << d_gain << "]");
    }

  /* yaw param */
  loadParam(yaw_nh);
  ROS_WARN_STREAM(ns << " param for Y: [ " <<  p_gain << ", " <<  i_gain << ", " << d_gain << "]");
}

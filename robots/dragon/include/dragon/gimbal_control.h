// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

/* ros */
#include <ros/ros.h>

/* ros msg */
#include <std_msgs/UInt8.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <spinal/DesireCoord.h>

/* basic transform control */
#include <dragon/transform_control.h>
/* basic control class */
#include <aerial_robot_base/control/flatness_pid_controller.h>

namespace control_plugin
{
  class DragonGimbal : public control_plugin::FlatnessPid
  {
  public:
    DragonGimbal();
    ~DragonGimbal(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    BasicEstimator* estimator, Navigator* navigator,
                    double ctrl_loop_rate);
    bool update();
    void reset()
    {
      FlatnessPid::reset();
      //yaw_i_term_.resize(1);
      //target_yaw_.resize(1);

      yaw_i_term_.assign(1, 0);
      target_yaw_.assign(1, 0);

      level_flag_ = false;
      landing_flag_ = false;
    }
    void sendCmd();
  private:
    boost::shared_ptr<DragonTransformController> kinematics_;
    ros::Publisher gimbal_control_pub_;
    ros::Publisher joint_control_pub_;
    ros::Publisher gimbal_target_force_pub_;
    ros::Publisher curr_desire_tilt_pub_;
    ros::Publisher  roll_pitch_pid_pub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber final_desire_tilt_sub_;

    void servoTorqueProcess();
    void landingProcess();
    void gimbalControl();
    void desireTilt();
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& state);
    void rosParamInit();

    void baselinkTiltCallback(const spinal::DesireCoordConstPtr & msg);
    void fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg);

    sensor_msgs::JointState joint_state_;
    Eigen::MatrixXd P_xy_;

    /* desire tilt */
    std::vector<double> target_gimbal_angles_;
    tf::Vector3 curr_desire_tilt_, final_desire_tilt_;

    bool simulation_;

    /* pitch roll control */
    double pitch_roll_control_rate_thresh_;
    tf::Vector3 pitch_roll_gains_;
    double pitch_roll_limit_;
    tf::Vector3 pitch_roll_terms_limits_;
    double roll_i_term_, pitch_i_term_;
    double gimbal_control_stamp_;

    /* landing process */
    bool level_flag_;
    bool landing_flag_;

    bool real_machine_;
    bool servo_torque_;

    bool control_verbose_;

    /* rosparam */
    double height_thresh_;
    string joints_torque_control_srv_name_;
    double tilt_thresh_;
    double tilt_pub_interval_;

    /* cfg */
    dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>* roll_pitch_pid_server_;
    dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>::CallbackType dynamic_reconf_func_roll_pitch_pid_;
    void cfgPitchRollPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level);

    dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>* yaw_pid_server_;
    dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>::CallbackType dynamic_reconf_func_yaw_pid_;
    void cfgYawPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level);

    /* psuedo inverse */
    /* https://gist.github.com/javidcf/25066cf85e71105d57b6 */
    template <class MatT>
    Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
    pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
    {
      typedef typename MatT::Scalar Scalar;
      auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
      const auto &singularValues = svd.singularValues();
      Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
      singularValuesInv.setZero();
      for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
          {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
          }
        else
          {
            singularValuesInv(i, i) = Scalar{0};
          }
      }
      return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
    }
  };
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_plugin::DragonGimbal, control_plugin::ControlBase);
#endif

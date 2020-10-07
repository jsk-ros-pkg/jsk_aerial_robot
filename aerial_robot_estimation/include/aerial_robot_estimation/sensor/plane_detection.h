// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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

#pragma once

#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <kalman_filter/kf_pos_vel_acc_plugin.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>

namespace sensor_plugin
{
  class PlaneDetection : public sensor_plugin::SensorBase
  {
  public:

    PlaneDetection();
    ~PlaneDetection(){}

    void initialize(ros::NodeHandle nh,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    string sensor_name, int index) override;

  private:
    /* ros */
    ros::Subscriber plane_sub_;

    std::string plane_detection_sub_topic_name_;
    aerial_robot_msgs::States plane_detection_state_;

    /* ros param */
    double eps_angle_;
    double distance_diff_thresh_;
    double plane_noise_sigma_;
    double min_height_, max_height_;
    double max_camera_angle_;

    /* estimation */
    double raw_plane_pos_z_;
    double prev_raw_plane_pos_z_;
    double raw_plane_vel_z_;

    bool findValidPlane(const jsk_recognition_msgs::ModelCoefficientsArray& msg);
    bool isCameraAngleValid();
    void estimateProcess() override;
    void rosParamInit();
    void planeCallback(const jsk_recognition_msgs::ModelCoefficientsArray& msg);
  };

};

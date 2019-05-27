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

#ifndef BASIC_STATE_ESTIMATION_H_
#define BASIC_STATE_ESTIMATION_H_

/* ros */
#include <ros/ros.h>

/* kinematics model */
#include <aerial_robot_model/transformable_aerial_robot_model.h>

#include <pluginlib/class_loader.h>
/* kf plugin */
#include <kalman_filter/kf_base_plugin.h>

/* ros msg */
#include <aerial_robot_msgs/States.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

/* util */
#include <assert.h>
#include <iostream>
#include <vector>
#include <utility>
#include <map>
#include <array>
#include <deque>
#include <fnmatch.h>

using namespace std;

/* bool: status(activate or not)
   tf::Vector3:  [x, dx, ddx]
*/
using StateWithStatus = std::pair<int, tf::Vector3>;
/* egomotion, experiment, ground truth */
using AxisState = std::array<StateWithStatus, 3>;
using SensorFuser = std::vector< std::pair<std::string, boost::shared_ptr<kf_plugin::KalmanFilter> > >;

namespace Frame
{
  enum {COG = 0, BASELINK = 1,};
};

namespace State
{
  enum
    {
      X_COG, //x axis of COG
      Y_COG, //y axis of COG
      Z_COG, //z axis of COG
      X_BASE, //x axis of Baselink
      Y_BASE, //y axis of Baselink
      Z_BASE, //y axis of Baselink
      ROLL_COG, //roll of CoG in world coord
      PITCH_COG, //pitch of CoG in world coord
      YAW_COG, //yaw of CoG in world coord
      ROLL_BASE, //roll of baselink in world coord
      PITCH_BASE, //pitch of baselink in world coord
      YAW_BASE, //yaw of baselink in world coord
      TOTAL_NUM,
    };
};

/* pre-definition */
namespace sensor_plugin
{
  class  SensorBase;
};


class StateEstimator
{

public:
  StateEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private);

  virtual ~StateEstimator()
  {
    update_thread_.interrupt();
    update_thread_.join();
  }

  //mode
  static constexpr int NONE = -1;
  static constexpr int EGOMOTION_ESTIMATE = 0;
  static constexpr int EXPERIMENT_ESTIMATE = 1;
  static constexpr int GROUND_TRUTH = 2;

  static constexpr float G = 9.797;

  boost::shared_ptr<aerial_robot_model::RobotModel>  getRobotModel() {return kinematics_model_;}

  int getStateStatus(uint8_t axis, uint8_t estimate_mode)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    assert(axis < State::TOTAL_NUM);
    return state_[axis][estimate_mode].first;
  }

  void setStateStatus( uint8_t axis, uint8_t estimate_mode, bool status)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    assert(axis < State::TOTAL_NUM);
    if(status) state_[axis][estimate_mode].first ++;
    else
      {
        if(state_[axis][estimate_mode].first > 0)
          state_[axis][estimate_mode].first --;
        else
          ROS_ERROR("wrong status update for axis: %d, estimate mode: %d", axis, estimate_mode);
      }
  }

  /* axis: state axis (11) */
  AxisState getState( uint8_t axis)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    assert(axis < State::TOTAL_NUM);
    return state_[axis];
  }

  tf::Vector3 getState( uint8_t axis,  uint8_t estimate_mode)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);

    assert(estimate_mode == EGOMOTION_ESTIMATE || estimate_mode == EXPERIMENT_ESTIMATE || estimate_mode == GROUND_TRUTH);

    return state_[axis][estimate_mode].second;
  }
  void setState( uint8_t axis,  int estimate_mode,  tf::Vector3 state)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);

    assert(estimate_mode == EGOMOTION_ESTIMATE || estimate_mode == EXPERIMENT_ESTIMATE || estimate_mode == GROUND_TRUTH);

    state_[axis][estimate_mode].second = state;
  }

  void setState( uint8_t axis,  int estimate_mode,  uint8_t state_mode,  float value)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);

    assert(estimate_mode == EGOMOTION_ESTIMATE || estimate_mode == EXPERIMENT_ESTIMATE || estimate_mode == GROUND_TRUTH);

    (state_[axis][estimate_mode].second)[state_mode] = value;
  }

  tf::Vector3 getPos(int frame, int estimate_mode)
  {
    return tf::Vector3((state_[State::X_COG + frame * 3][estimate_mode].second)[0],
                       (state_[State::Y_COG + frame * 3][estimate_mode].second)[0],
                       (state_[State::Z_COG + frame * 3][estimate_mode].second)[0]);
  }
  void setPos(int frame, int estimate_mode, tf::Vector3 pos)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    for(int i = 0; i < 3; i ++ )
      (state_[i + frame * 3][estimate_mode].second)[0] = pos[i];
  }

  tf::Vector3 getVel(int frame, int estimate_mode)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    return tf::Vector3((state_[State::X_COG + frame * 3][estimate_mode].second)[1],
                       (state_[State::Y_COG + frame * 3][estimate_mode].second)[1],
                       (state_[State::Z_COG + frame * 3][estimate_mode].second)[1]);
  }

  void setVel(int frame, int estimate_mode, tf::Vector3 vel)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    for(int i = 0; i < 3; i ++ )
      (state_[i + frame * 3][estimate_mode].second)[1] = vel[i];
  }

  tf::Matrix3x3 getOrientation(int frame, int estimate_mode)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    tf::Matrix3x3 r;
    r.setRPY((state_[State::ROLL_COG + frame * 3][estimate_mode].second)[0],
             (state_[State::PITCH_COG + frame * 3][estimate_mode].second)[0],
             (state_[State::YAW_COG + frame * 3][estimate_mode].second)[0]);
    return r;
  }

  void setEuler(int frame, int estimate_mode, tf::Vector3 euler)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    (state_[State::ROLL_COG + frame * 3][estimate_mode].second)[0] = euler[0];
    (state_[State::PITCH_COG + frame * 3][estimate_mode].second)[0] = euler[1];
    (state_[State::YAW_COG + frame * 3][estimate_mode].second)[0] = euler[2];
  }

  tf::Vector3 getAngularVel(int frame, int estimate_mode)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    return tf::Vector3((state_[State::ROLL_COG + frame * 3][estimate_mode].second)[1],
                       (state_[State::PITCH_COG + frame * 3][estimate_mode].second)[1],
                       (state_[State::YAW_COG + frame * 3][estimate_mode].second)[1]);
  }

  void setAngularVel(int frame, int estimate_mode, tf::Vector3 omega)
  {
    boost::lock_guard<boost::mutex> lock(state_mutex_);
    (state_[State::ROLL_COG + frame * 3][estimate_mode].second)[1] = omega[0];
    (state_[State::PITCH_COG + frame * 3][estimate_mode].second)[1] = omega[1];
    (state_[State::YAW_COG + frame * 3][estimate_mode].second)[1] = omega[2];
  }

  inline void setQueueSize(const int& qu_size) {qu_size_ = qu_size;}
  void updateQueue(const double& timestamp, const double& roll, const double& pitch, const tf::Vector3& omega)
  {
    tf::Matrix3x3 r_ee, r_ex, r_gt;
    r_ee.setRPY(roll, pitch, (getState(State::YAW_BASE, StateEstimator::EGOMOTION_ESTIMATE))[0]);
    r_ex.setRPY(roll, pitch, (getState(State::YAW_BASE, StateEstimator::EXPERIMENT_ESTIMATE))[0]);
    r_gt.setRPY(roll, pitch, (getState(State::YAW_BASE, StateEstimator::GROUND_TRUTH))[0]);

    {
      boost::lock_guard<boost::mutex> lock(queue_mutex_);
      timestamp_qu_.push_back(timestamp);
      rot_ee_qu_.push_back(r_ee);
      rot_ex_qu_.push_back(r_ex);
      rot_gt_qu_.push_back(r_gt);
      omega_qu_.push_back(omega);

      if(timestamp_qu_.size() > qu_size_)
        {
          timestamp_qu_.pop_front();
          rot_ee_qu_.pop_front();
          rot_ex_qu_.pop_front();
          rot_gt_qu_.pop_front();
          omega_qu_.pop_front();
        }
    }
  }

  bool findRotOmege(const double& timestamp, const int& mode, tf::Matrix3x3& r, tf::Vector3& omega)
  {
    boost::lock_guard<boost::mutex> lock(queue_mutex_);

    if(timestamp_qu_.size() == 0)
      {
        ROS_ERROR("estimation: no valid queue for timestamp to find proper r and omega");
        return false;
      }

    if(timestamp < timestamp_qu_.front())
      {
        ROS_ERROR("estimation: sensor timestamp %f is earlier than the oldest timestamp %f in queue",
                  timestamp, timestamp_qu_.front());
        return false;
      }

    if(timestamp > timestamp_qu_.back())
      {
        ROS_ERROR("estimation: sensor timestamp %f is later than the latest timestamp %f in queue",
                  timestamp, timestamp_qu_.front());
        return false;
      }

    size_t candidate_index = (timestamp_qu_.size() - 1) * (timestamp - timestamp_qu_.front()) / (timestamp_qu_.back() - timestamp_qu_.front());

    if(timestamp > timestamp_qu_.at(candidate_index))
      {
        for(auto it = timestamp_qu_.begin() + candidate_index; it != timestamp_qu_.end(); ++it)
          {
            /* future timestamp, escape */
            if(*it > timestamp)
              {
                if (fabs(*it - timestamp) < fabs(*(it - 1) - timestamp))
                  candidate_index = std::distance(timestamp_qu_.begin(), it);
                else
                  candidate_index = std::distance(timestamp_qu_.begin(), it-1);

                //ROS_INFO("find timestamp sensor vs imu: [%f, %f], candidate: %d", timestamp, timestamp_qu_.at(candidate_index), candidate_index);
                break;
              }
          }
      }
    else
      {
        for(auto it = timestamp_qu_.rbegin() + (timestamp_qu_.size() - 1 - candidate_index); it != timestamp_qu_.rend(); ++it)
          {
            /* future timestamp, escape */
            if(*it < timestamp)
              {
                if (fabs(*it - timestamp) < fabs(*(it - 1) - timestamp))
                  candidate_index = timestamp_qu_.size() - 1 - std::distance(timestamp_qu_.rbegin(), it);
                else
                  candidate_index = timestamp_qu_.size() - 1 - std::distance(timestamp_qu_.rbegin(), it-1);

                //ROS_INFO("reverse find timestamp sensor vs imu: [%f, %f], %d", timestamp, timestamp_qu_.at(candidate_index) , candidate_index);
                break;
              }
          }
      }

    omega = omega_qu_.at(candidate_index);
    switch(mode)
      {
      case StateEstimator::EGOMOTION_ESTIMATE:
        r = rot_ee_qu_.at(candidate_index);
        break;
      case StateEstimator::EXPERIMENT_ESTIMATE:
        r = rot_ex_qu_.at(candidate_index);
        break;
      case StateEstimator::GROUND_TRUTH:
        r = rot_gt_qu_.at(candidate_index);
        break;
      default:
        ROS_ERROR("estimation search state with timestamp: wrong mode %d", mode);
        return false;
      }

    return true;
  }

  inline const double getImuLatestTimeStamp()
  {
    boost::lock_guard<boost::mutex> lock(queue_mutex_);
    return timestamp_qu_.back();
  }


  inline void setSensorFusionFlag(bool flag){sensor_fusion_flag_ = flag;  }
  inline bool getSensorFusionFlag(){return sensor_fusion_flag_; }

  //start flying flag (~takeoff)
  virtual bool getFlyingFlag() {  return  flying_flag_;}
  virtual void setFlyingFlag(bool flag){  flying_flag_ = flag;}
  //start landing mode
  virtual bool getLandingMode() {  return  landing_mode_flag_;}
  virtual void setLandingMode(bool flag){  landing_mode_flag_ = flag;}
  // landed flag (acc_z check, ground shock)
  virtual bool getLandedFlag() {  return  landed_flag_;}
  virtual void setLandedFlag(bool flag){  landed_flag_ = flag;}
  /* when takeoff, should use the undescend mode be true */
  inline void setUnDescendMode(bool flag){un_descend_flag_ = flag;  }
  inline bool getUnDescendMode(){return un_descend_flag_; }
  /* landing height is set for landing to different terrain */
  virtual void setLandingHeight(float landing_height){ landing_height_ = landing_height;}
  virtual float getLandingHeight(){ return landing_height_;}
  /* att control mode is for user to manually control x and y motion by attitude control */
  inline void setForceAttControlFlag (bool flag) {force_att_control_flag_ = flag; }
  inline bool getForceAttControlFlag () {return force_att_control_flag_;}

  const std::map<std::string, KDL::Frame>& getSegmentsTf()
  {
    boost::lock_guard<boost::mutex> lock(kinematics_mutex_);
    return segments_tf_;
  }
  void setSegmentsTf(const std::map<std::string, KDL::Frame>& segments_tf)
  {
    boost::lock_guard<boost::mutex> lock(kinematics_mutex_);
    segments_tf_ = segments_tf;
  }

  inline std::string getBaselinkName() const {return baselink_name_;}
  inline tf::Transform getCog2Baselink(){return cog2baselink_transform_;}
  inline tf::Transform getBaselink2Cog(){return cog2baselink_transform_.inverse();}

  const SensorFuser& getFuser(int mode)
  {
    assert(mode >= 0 && mode < 2);
    return fuser_[mode];
  }

  inline int getEstimateMode() {return estimate_mode_;}
  inline void setEstimateMode(int estimate_mode) {estimate_mode_ = estimate_mode;}

  /* sensor unhealth level */
  static constexpr uint8_t UNHEALTH_LEVEL1 = 1; // do nothing
  static constexpr uint8_t UNHEALTH_LEVEL2 = 2; // change estimation mode
  static constexpr uint8_t UNHEALTH_LEVEL3 = 3; // force landing


  /* set unhealth level */
  void setUnhealthLevel(uint8_t unhealth_level)
  {
    if(unhealth_level > unhealth_level_)
      unhealth_level_ = unhealth_level;

    /* TODO: should write the solution for the unhealth sensor  */
  }

  inline uint8_t getUnhealthLevel() { return unhealth_level_; }

  const boost::shared_ptr<sensor_plugin::SensorBase> getImuHandler() const { return imu_handler_;}
  const boost::shared_ptr<sensor_plugin::SensorBase> getAltHandler() const { return alt_handler_;}
  const boost::shared_ptr<sensor_plugin::SensorBase> getVoHandler() const { return vo_handler_;}
  const boost::shared_ptr<sensor_plugin::SensorBase> getGpsHandler() const { return gps_handler_;}

protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher full_state_pub_, baselink_odom_pub_, cog_odom_pub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber cog2baselink_transform_sub_;
  ros::Subscriber estimate_height_mode_sub_;
  tf::TransformBroadcaster br_;

  boost::thread update_thread_;

  vector< boost::shared_ptr<sensor_plugin::SensorBase> > sensors_;
  boost::shared_ptr< pluginlib::ClassLoader<sensor_plugin::SensorBase> > sensor_plugin_ptr_;
  boost::shared_ptr<sensor_plugin::SensorBase> imu_handler_;
  boost::shared_ptr<sensor_plugin::SensorBase> alt_handler_;
  boost::shared_ptr<sensor_plugin::SensorBase> vo_handler_;
  boost::shared_ptr<sensor_plugin::SensorBase> gps_handler_;

  /* mutex */
  boost::mutex state_mutex_;
  boost::mutex kinematics_mutex_;
  boost::mutex queue_mutex_;
  /* ros param */
  bool param_verbose_;
  int estimate_mode_; /* main estimte mode */
  string cog2baselink_transform_sub_name_;
  double update_rate_;

  /* robot kinematics  */
  boost::shared_ptr<aerial_robot_model::RobotModel> kinematics_model_;
  std::map<std::string, KDL::Frame> segments_tf_;
  std::string baselink_name_;
  tf::Transform cog2baselink_transform_; // TODO: should be calculated from the aboved "kinemtaics_model_"

  /* 9: x_w, y_w, z_w, roll_w, pitch_w, yaw_cog_w, x_b, y_b, yaw_board_w */
  array<AxisState, State::TOTAL_NUM> state_;

  /* for calculate the sensor to baselink with the consideration of time delay */
  int qu_size_;
  deque<double> timestamp_qu_;
  deque<tf::Matrix3x3> rot_ee_qu_, rot_ex_qu_, rot_gt_qu_;
  deque<tf::Vector3> omega_qu_;

  /* sensor fusion */
  boost::shared_ptr< pluginlib::ClassLoader<kf_plugin::KalmanFilter> > sensor_fusion_loader_ptr_;
  bool sensor_fusion_flag_;
  array<SensorFuser, 2> fuser_; //0: egomotion; 1: experiment

  /* sensor (un)health level */
  uint8_t unhealth_level_;

  /* height related var */
  bool flying_flag_;
  bool landing_mode_flag_;
  bool landed_flag_;
  bool un_descend_flag_;
  float landing_height_;
  bool force_att_control_flag_;

  /* update the kinematics model based on joint state */
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& state)
  {
    auto segments_tf = kinematics_model_->fullForwardKinematics(*state); // do not need inertial and cog calculation right now
    setSegmentsTf(segments_tf);
  }

  /* use subscribe method to get the cog-baselink offset */
  /* only the position offset, no vel and acc */
  void transformCallback(const geometry_msgs::TransformStampedConstPtr & msg)
  {
    // TODO: should be calculated from the aboved "kinemtaics_model_"

    /* get the transform from CoG to base_link: {CoG} -> {baselink} */
    tf::transformMsgToTF(msg->transform, cog2baselink_transform_);
  }

  void statePublish();
  void rosParamInit();
  bool pattern_match(std::string &pl, std::string &pl_candidate);

  void update()
  {
    ros::Rate loop_rate(update_rate_);
    while(ros::ok())
      {
        statePublish();
        ros::spinOnce();
        loop_rate.sleep();
      }
  }
};


#endif

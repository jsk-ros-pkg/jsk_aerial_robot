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

#pragma once

#include <aerial_robot_model/model/aerial_robot_model.h>
#include <aerial_robot_msgs/States.h>
#include <array>
#include <assert.h>
#include <boost/enable_shared_from_this.hpp>
#include <deque>
#include <fnmatch.h>
#include <geometry_msgs/TransformStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <iostream>
#include <kalman_filter/kf_base_plugin.h>
#include <map>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <utility>
#include <vector>

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
      TOTAL_NUM,
    };

  namespace CoG
  {
    enum
      {
       Rot = 10,
      };
  };

  namespace Base
  {
    enum
      {
       Rot = 11,
      };
  };

};

namespace Sensor
{
  enum
    {
      UNHEALTH_LEVEL1 = 1, // do nothing
      UNHEALTH_LEVEL2, // change estimation mode
      UNHEALTH_LEVEL3, // force landing
    };
};

/* pre-definition */
namespace sensor_plugin
{
  class  SensorBase;
};


namespace aerial_robot_estimation
{
  //mode
  static constexpr int NONE = -1;
  static constexpr int EGOMOTION_ESTIMATE = 0;
  static constexpr int EXPERIMENT_ESTIMATE = 1;
  static constexpr int GROUND_TRUTH = 2;

  static constexpr float G = 9.797;

  class StateEstimator: public boost::enable_shared_from_this<StateEstimator>
  {

  public:
    StateEstimator();

    virtual ~StateEstimator()
    {
    }

    void initialize(ros::NodeHandle nh, ros::NodeHandle nh_private, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model);

    int getStateStatus(uint8_t axis, uint8_t estimate_mode)
    {
      boost::lock_guard<boost::mutex> lock(state_mutex_);
      if(axis < State::TOTAL_NUM)
        return state_[axis][estimate_mode].first;
      else if(axis == State::Base::Rot)
        return base_rot_status_.at(estimate_mode);
      else if(axis == State::CoG::Rot)
        return cog_rot_status_.at(estimate_mode);
      else
        return 0;
    }

    void setStateStatus(uint8_t axis, uint8_t estimate_mode, bool status)
    {
      boost::lock_guard<boost::mutex> lock(state_mutex_);

      int* incre_status;
      if(axis < State::TOTAL_NUM)
        incre_status = &(state_[axis][estimate_mode].first);
      else if(axis == State::Base::Rot)
        incre_status = &(base_rot_status_.at(estimate_mode));
      else if(axis == State::CoG::Rot)
        incre_status = &(cog_rot_status_.at(estimate_mode));
      else
        return;

      if(status) (*incre_status) ++;
      else
        {
          if(*incre_status > 0) (*incre_status) --;
          else
            ROS_WARN("wrong status update for axis: %d, estimate mode: %d", axis, estimate_mode);
        }
    }

    AxisState getState( uint8_t axis)
    {
      boost::lock_guard<boost::mutex> lock(state_mutex_);
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

    void setState(uint8_t axis, int estimate_mode, uint8_t state_mode, float value)
    {
      boost::lock_guard<boost::mutex> lock(state_mutex_);

      assert(estimate_mode == EGOMOTION_ESTIMATE || estimate_mode == EXPERIMENT_ESTIMATE || estimate_mode == GROUND_TRUTH);

      (state_[axis][estimate_mode].second)[state_mode] = value;
    }

    tf::Vector3 getPos(int frame, int estimate_mode)
    {
      boost::lock_guard<boost::mutex> lock(state_mutex_);
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

      if(frame == Frame::COG)
        return cog_rots_.at(estimate_mode);
      else if(frame == Frame::BASELINK)
        return base_rots_.at(estimate_mode);

      return tf::Matrix3x3::getIdentity();
    }

    void setOrientation(int frame, int estimate_mode, tf::Matrix3x3 rot)
    {
      boost::lock_guard<boost::mutex> lock(state_mutex_);

      if(frame == Frame::COG)
        cog_rots_.at(estimate_mode) = rot;
      else if(frame == Frame::BASELINK)
        base_rots_.at(estimate_mode) = rot;
    }

    void setOrientationWxB(int frame, int estimate_mode, tf::Vector3 v);
    void setOrientationWzB(int frame, int estimate_mode, tf::Vector3 v);

    tf::Vector3 getEuler(int frame, int estimate_mode)
    {
      tf::Matrix3x3 rot = getOrientation(frame, estimate_mode);
      tfScalar r = 0, p = 0, y = 0;
      rot.getRPY(r, p, y);

      return tf::Vector3(r, p, y);
    }

    tf::Vector3 getAngularVel(int frame, int estimate_mode)
    {
      boost::lock_guard<boost::mutex> lock(state_mutex_);

      if(frame == Frame::COG)
        return cog_omegas_.at(estimate_mode);
      else if(frame == Frame::BASELINK)
        return base_omegas_.at(estimate_mode);

      return tf::Vector3(0,0,0);
    }

    void setAngularVel(int frame, int estimate_mode, tf::Vector3 omega)
    {
      boost::lock_guard<boost::mutex> lock(state_mutex_);

      if(frame == Frame::COG)
        cog_omegas_.at(estimate_mode) = omega;
      else if(frame == Frame::BASELINK)
        base_omegas_.at(estimate_mode) = omega;
    }

    inline void setQueueSize(const int& qu_size) {qu_size_ = qu_size;}
    void updateQueue(const double timestamp, const tf::Matrix3x3 r_ee, const tf::Matrix3x3 r_ex, const tf::Vector3 omega)
    {
      boost::lock_guard<boost::mutex> lock(queue_mutex_);
      timestamp_qu_.push_back(timestamp);
      rot_ee_qu_.push_back(r_ee);
      rot_ex_qu_.push_back(r_ex);
      omega_qu_.push_back(omega);

      if(timestamp_qu_.size() > qu_size_)
        {
          timestamp_qu_.pop_front();
          rot_ee_qu_.pop_front();
          rot_ex_qu_.pop_front();
          omega_qu_.pop_front();
        }
    }

    bool findRotOmega(const double timestamp, const int mode, tf::Matrix3x3& r, tf::Vector3& omega, bool verbose = true)
    {
      boost::lock_guard<boost::mutex> lock(queue_mutex_);

      if(timestamp_qu_.size() == 0)
        {
          ROS_WARN_COND(verbose, "estimation: no valid queue for timestamp to find proper r and omega");

          return false;
        }

      if(timestamp < timestamp_qu_.front())
        {
          ROS_WARN_COND(verbose, "estimation: sensor timestamp %f is earlier than the oldest timestamp %f in queue",
                        timestamp, timestamp_qu_.front());
          return false;
        }

      if(timestamp > timestamp_qu_.back())
        {
          ROS_WARN_COND(verbose, "estimation: sensor timestamp %f is later than the latest timestamp %f in queue",
                        timestamp, timestamp_qu_.back());

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
        case EGOMOTION_ESTIMATE:
          r = rot_ee_qu_.at(candidate_index);
          break;
        case EXPERIMENT_ESTIMATE:
          r = rot_ex_qu_.at(candidate_index);
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
    /* when takeoff, should use the undescend mode be true */
    inline void setUnDescendMode(bool flag){un_descend_flag_ = flag;  }
    inline bool getUnDescendMode(){return un_descend_flag_; }
    /* att control mode is for user to manually control x and y motion by attitude control */
    inline void setForceAttControlFlag (bool flag) {force_att_control_flag_ = flag; }
    inline bool getForceAttControlFlag () {return force_att_control_flag_;}
    /* latitude & longitude value for GPS based navigation */
    inline void setCurrGpsPoint(const geographic_msgs::GeoPoint point) {curr_wgs84_poiont_ = point;}
    inline const geographic_msgs::GeoPoint& getCurrGpsPoint() const {return curr_wgs84_poiont_;}
    inline const bool hasGroundTruthOdom() const {return has_groundtruth_odom_; }
    inline void receiveGroundTruthOdom(bool flag) {has_groundtruth_odom_ = flag; }
    inline const bool hasRefinedYawEstimate(int i) const {return has_refined_yaw_estimate_.at(i); }
    inline void SetRefinedYawEstimate(int i, bool flag) {has_refined_yaw_estimate_.at(i) = flag; }


    const SensorFuser& getFuser(int mode)
    {
      assert(mode >= 0 && mode < 2);
      return fuser_[mode];
    }

    inline int getEstimateMode() {return estimate_mode_;}
    inline void setEstimateMode(int estimate_mode) {estimate_mode_ = estimate_mode;}

    /* set unhealth level */
    void setUnhealthLevel(uint8_t unhealth_level)
    {
      if(unhealth_level > unhealth_level_)
        unhealth_level_ = unhealth_level;

      /* TODO: should write the solution for the unhealth sensor  */
    }

    inline uint8_t getUnhealthLevel() { return unhealth_level_; }

    const vector<boost::shared_ptr<sensor_plugin::SensorBase> >& getImuHandlers() const { return imu_handlers_;}
    const vector<boost::shared_ptr<sensor_plugin::SensorBase> >& getAltHandlers() const { return alt_handlers_;}
    const vector<boost::shared_ptr<sensor_plugin::SensorBase> >& getVoHandlers() const { return vo_handlers_;}
    const vector<boost::shared_ptr<sensor_plugin::SensorBase> >& getGpsHandlers() const { return gps_handlers_;}
    const vector<boost::shared_ptr<sensor_plugin::SensorBase> >& getPlaneDetectionHandlers() const { return plane_detection_handlers_;}

    const boost::shared_ptr<sensor_plugin::SensorBase> getImuHandler(int i) const { return imu_handlers_.at(i);}
    const boost::shared_ptr<sensor_plugin::SensorBase> getAltHandlers(int i) const { return alt_handlers_.at(i);}
    const boost::shared_ptr<sensor_plugin::SensorBase> getVoHandlers(int i) const { return vo_handlers_.at(i);}
    const boost::shared_ptr<sensor_plugin::SensorBase> getGpsHandlers(int i) const { return gps_handlers_.at(i);}
    const boost::shared_ptr<sensor_plugin::SensorBase> getPlaneDetectionHandlers(int i) const { return plane_detection_handlers_.at(i);}

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher full_state_pub_, baselink_odom_pub_, cog_odom_pub_;
    tf2_ros::TransformBroadcaster br_;
    ros::Timer state_pub_timer_;

    vector< boost::shared_ptr<sensor_plugin::SensorBase> > sensors_;
    boost::shared_ptr< pluginlib::ClassLoader<sensor_plugin::SensorBase> > sensor_plugin_ptr_;
    vector<boost::shared_ptr<sensor_plugin::SensorBase> > imu_handlers_;
    vector<boost::shared_ptr<sensor_plugin::SensorBase> > alt_handlers_;
    vector<boost::shared_ptr<sensor_plugin::SensorBase> > vo_handlers_;
    vector<boost::shared_ptr<sensor_plugin::SensorBase> > gps_handlers_;
    vector<boost::shared_ptr<sensor_plugin::SensorBase> > plane_detection_handlers_;

    /* mutex */
    boost::mutex state_mutex_;
    boost::mutex queue_mutex_;
    /* ros param */
    bool param_verbose_;
    int estimate_mode_; /* main estimte mode */

    /* robot model (kinematics)  */
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
    std::string tf_prefix_;

    /* 6: x_w, y_w, z_w, x_b, y_b */
    /* TODO: check to vector3 */
    array<AxisState, 6> state_;
    array<tf::Matrix3x3, 3> base_rots_, cog_rots_; // TODO: should abolish the different between orientation of baselink and that of CoG
    array<tf::Vector3, 3> base_omegas_, cog_omegas_; // TODO: should abolish the different between orientation of baselink and that of CoG
    array<int, 3> base_rot_status_, cog_rot_status_;

    bool has_groundtruth_odom_; // whether receive entire groundthtruth odometry (e.g., for simulation mode)

    std::map<int, bool> has_refined_yaw_estimate_; // whether receive refined yaw estimation data (e.g., vio) for each estimate mode

    /* for calculate the sensor to baselink with the consideration of time delay */
    int qu_size_;
    deque<double> timestamp_qu_;
    deque<tf::Matrix3x3> rot_ee_qu_, rot_ex_qu_;
    deque<tf::Vector3> omega_qu_;

    /* sensor fusion */
    boost::shared_ptr< pluginlib::ClassLoader<kf_plugin::KalmanFilter> > sensor_fusion_loader_ptr_;
    bool sensor_fusion_flag_;
    array<SensorFuser, 2> fuser_; //0: egomotion; 1: experiment

    /* sensor (un)health level */
    uint8_t unhealth_level_;

    /* height related var */
    bool flying_flag_;
    bool un_descend_flag_;
    bool force_att_control_flag_;

    /* latitude & longitude point */
    geographic_msgs::GeoPoint curr_wgs84_poiont_;

    void statePublish(const ros::TimerEvent & e);
    void rosParamInit();
  };
};

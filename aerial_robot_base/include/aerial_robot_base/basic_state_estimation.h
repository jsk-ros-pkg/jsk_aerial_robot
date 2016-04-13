#ifndef BASIC_STATE_ESTIMATION_H
#define BASIC_STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/States.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <vector>

#include <Eigen/Core>

#include <kalman_filter/kf_base_plugin.h>
#include <pluginlib/class_loader.h>

class BasicEstimator
{
 public:
 BasicEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private)
   : nh_(nh, "estimator"), nhp_(nh_private, "estimator")
    {
      full_states_pub_ = nh_.advertise<aerial_robot_base::States>("full_states", 1); 


      gt_state_.resize(9);
      ee_state_.resize(9);
      ex_state_.resize(9);

      for(int i = 0; i < 9; i ++)
        {
          for(int j = 0; j < 3; j++)
            {            
              (gt_state_[i])[j] = 0; //ground truth
              (ee_state_[i])[j] = 0; //egomtion estimate
              (ex_state_[i])[j] = 0; //experiment
            }
        }

      state_pos_z_offset_ = 1.0; //1m
      sys_stamp_ = ros::Time::now();//removed this

    }


  virtual ~BasicEstimator(){}

  //mode
  static const uint8_t X_W = 0; //x in world coord
  static const uint8_t Y_W = 1; //y in world coord
  static const uint8_t Z_W = 2; //z in world coord
  static const uint8_t ROLL_W = 3; //roll in world coord
  static const uint8_t PITCH_W = 4; //pitch in world coord
  static const uint8_t YAW_W_COG = 5; //yaw of cog in world coord
  static const uint8_t X_B = 6; //x in board coord
  static const uint8_t Y_B = 7; //y in board coord
  static const uint8_t YAW_W_B = 8; //yaw of board in world coord

  inline ros::Time getSystemTimeStamp(){ return sys_stamp_;}
  inline void setSystemTimeStamp(ros::Time sys_stamp){ sys_stamp_ = sys_stamp;}

  inline float getGTState(int axis, int mode){ return (gt_state_[axis])[mode];}
  inline void setGTState(int axis, int mode, float value){ (gt_state_[axis])[mode] = value;}

  inline float getEEState(int axis, int mode){ return (ee_state_[axis])[mode];}
  inline void setEEState(int axis, int mode, float value){ (ee_state_[axis])[mode] = value;}

  inline float getEXState(int axis, int mode){ return (ex_state_[axis])[mode];}
  inline void setEXState(int axis, int mode, float value){ (ex_state_[axis])[mode] = value;}

  inline void setSensorFusionFlag(bool flag){sensor_fusion_flag_ = flag; }
  inline bool getSensorFusionFlag(){return sensor_fusion_flag_; }

  virtual float getPosZOffset() {  return  state_pos_z_offset_;}
  virtual void setPosZOffset(float pos_z_offset){  state_pos_z_offset_ = pos_z_offset;}

  inline boost::shared_ptr<kf_base_plugin::KalmanFilter> getFuserEgomotion(int no) { return fuser_egomotion_[no];}
  inline boost::shared_ptr<kf_base_plugin::KalmanFilter> getFuserExperiment(int no) { return fuser_experiment_[no];}
  inline std::string  getFuserEgomotionName(int no) { return fuser_egomotion_name_[no];}
  inline std::string  getFuserExperimentName(int no) { return fuser_experiment_name_[no];}
  inline std::string  getFuserEgomotionPluginName(int no) { return fuser_egomotion_plugin_name_[no];}
  inline std::string  getFuserExperimentPluginName(int no) { return fuser_experiment_plugin_name_[no];}

  inline int  getFuserEgomotionId(int no) { return fuser_egomotion_id_[no];}
  inline int  getFuserExperimentId(int no) { return fuser_experiment_id_[no];}
  inline int  getFuserEgomotionNo() { return fuser_egomotion_no_;}
  inline int  getFuserExperimentNo() { return fuser_experiment_no_;}


 protected:  

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher full_states_pub_;

  ros::Time sys_stamp_;

  // xw, yw, zw, rollw, pitchw, yaww of cog, xb, yb, yaww of board (9) 
  // (0): x, (1): dx, (2); ddx
  std::vector< Eigen::Vector3d>  gt_state_; //ground truth
  std::vector< Eigen::Vector3d>  ee_state_; //egomotion estimate in world coord
  std::vector< Eigen::Vector3d>  ex_state_; //experiment in world coord

  float state_pos_z_offset_; 

  //tf::TransformBroadcaster* br_;
  boost::shared_ptr< pluginlib::ClassLoader<kf_base_plugin::KalmanFilter> > sensor_fusion_loader_ptr_;
  //pluginlib::ClassLoader<kf_base_plugin::KalmanFilter>  sensor_fusion_loader_;

  // sensor fusion
  bool sensor_fusion_flag_;
  int fuser_egomotion_no_, fuser_experiment_no_;
  std::vector<std::string> fuser_egomotion_plugin_name_;
  std::vector<std::string> fuser_experiment_plugin_name_;
  std::vector<std::string> fuser_egomotion_name_;
  std::vector<std::string> fuser_experiment_name_;
  std::vector<int> fuser_egomotion_id_;
  std::vector<int> fuser_experiment_id_;
  std::vector< boost::shared_ptr<kf_base_plugin::KalmanFilter> > fuser_egomotion_;
  std::vector< boost::shared_ptr<kf_base_plugin::KalmanFilter> > fuser_experiment_;


};


#endif

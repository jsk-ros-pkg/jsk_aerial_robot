// -*- mode: c++ -*-

#pragma once

#include <beetle/beetle_navigation.h>
#include <ninja/model/ninja_robot_model.h>
#include <geometry_msgs/Pose.h>
#include <regex>
#include <aerial_robot_control/control/utils/pid.h>
#include <std_msgs/Bool.h>
#include <kalman_filter/lpf_filter.h>
#include <angles/angles.h>

namespace aerial_robot_navigation
{
  enum module_joint
    {
     PITCH,
     YAW
    };

    enum convergence_func
    {
     CONSTANT,
     FRAC,
     EXP
    };

  enum ninja_control_frame
    {
     LEFT_DOCK = LOCAL_FRAME + 1,
     RIGHT_DOCK
    };

  class ModuleData
  {
  public:
    ModuleData(int id): id_(id), des_joint_pos_(), est_joint_pos_(), goal_joint_pos_(), start_joint_pos_(), module_tree_(), first_joint_processed_time_(), joint_process_coef_(){}
    ModuleData(): id_(1), des_joint_pos_(), est_joint_pos_(), goal_joint_pos_(), start_joint_pos_(), module_tree_(), first_joint_processed_time_(), joint_process_coef_(){}
    int id_;
    KDL::JntArray des_joint_pos_;
    KDL::JntArray est_joint_pos_;
    KDL::Tree module_tree_;

    std::vector<double> first_joint_processed_time_;
    std::vector<double> joint_process_coef_;
    KDL::JntArray goal_joint_pos_;
    KDL::JntArray start_joint_pos_;

  };
  class NinjaNavigator : public BeetleNavigator
  {
  public:
    NinjaNavigator();
    ~NinjaNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;
    void update() override;

    /*accessor for target rotation*/
    void setTargetComRot(KDL::Rotation target_com_rot){ target_com_rot_ = target_com_rot;}
    void setGoalComRot(KDL::Rotation goal_com_rot){ goal_com_rot_ = goal_com_rot;}
    inline void setGoalComRoll( float value)
    {
      setGoalComRot(KDL::Rotation::RPY(value,0,0));
    }
    inline void setGoalComPitch( float value)
    {
      setGoalComRot(KDL::Rotation::RPY(0,value,0));
    }
    inline void setGoalComYaw( float value)
    {
      setGoalComRot(KDL::Rotation::RPY(0,0,value));
    }
    inline void setTargetComRPY( tf::Vector3 value)
    {
      setGoalComRot(KDL::Rotation::RPY(value.x(),value.y(),value.z()));
    }
    
    void setTargetCoMPoseFromCurrState();
    void setTargetJointPosFromCurrState();
    void setCoM2Base(const KDL::Frame com2base){com2base_ = com2base;}

    /*accessor for current pos*/
    inline void setCurrComPos( tf::Vector3 value)
    {
      curr_com_pos_ = value;
    }
    
    /*accessor for target vel*/
    inline void setTargetVelCandX( float value)
    {
      std::lock_guard<std::mutex> lock(mutex_cand_vel_);
      target_vel_candidate_.setX(value);
    }
    inline void setTargetVelCandY( float value)
    {
      std::lock_guard<std::mutex> lock(mutex_cand_vel_);
      target_vel_candidate_.setY(value);
    }
    inline void setTargetVelCandZ( float value)
    {
      std::lock_guard<std::mutex> lock(mutex_cand_vel_);
      target_vel_candidate_.setZ(value);
    }
    inline void setTargetVelCand( tf::Vector3 value)
    {
      target_vel_candidate_ = value;
    }

    /*accessor for error vel*/
    inline void setCurrVelCand( tf::Vector3 value)
    {
      curr_vel_candidate_ = value;
    }    

    /*accessor for error vel*/
    inline void setErrVelCand( tf::Vector3 value)
    {
      err_vel_candidate_ = value;
    }

    /*accessor for current rpy*/
    inline void setCurrComRPY( tf::Vector3 value)
    {
      curr_com_rpy_ = value;
    }

    /*accessor for target omega*/
    inline void setTargetOmegaCandX( float value)
    {
      std::lock_guard<std::mutex> lock(mutex_cand_omega_);
      target_omega_candidate_.setX(value);
    }
    inline void setTargetOmegaCandY( float value)
    {
      std::lock_guard<std::mutex> lock(mutex_cand_omega_);
      target_omega_candidate_.setY(value);
    }
    inline void setTargetOmegaCandZ( float value)
    {
      std::lock_guard<std::mutex> lock(mutex_cand_omega_);
      target_omega_candidate_.setZ(value);
    }
    inline void setTargetOmegaCand( tf::Vector3 value)
    {
      target_omega_candidate_ = value;
    }

    /*accessor for curr omega*/
    inline void setCurrOmegaCand( tf::Vector3 value)
    {
      curr_omega_candidate_ = value;
    }

    /*accessor for error omega*/
    inline void setErrOmegaCand( tf::Vector3 value)
    {
      err_omega_candidate_ = value;
    }
    
        
    inline void setFinalTargetPosCandX( float value){  target_final_pos_candidate_.setX(value);}
    inline void setFinalTargetPosCandY( float value){  target_final_pos_candidate_.setY(value);}
    inline void setFinalTargetPosCandZ( float value){  target_final_pos_candidate_.setZ(value);}
    inline void setFinalTargetPosCand( tf::Vector3 value){  target_final_pos_candidate_ = value ;}
    
    void morphingProcess();

    void calcComStateProcess();

    bool getFreeJointFlag(){return free_joint_flag_;}
    std::vector<double> getJointPosErr(){return joint_pos_errs_;}
    template<class T> T getCom2Base();
    inline tf::Vector3 getTargetFinalPosCand() {return target_final_pos_candidate_;}
    inline tf::Vector3 getTargetFinalRPYCand()
    {
      double target_roll, target_pitch, target_yaw;
      target_com_rot_.GetEulerZYX(target_yaw, target_pitch, target_roll);
      return tf::Vector3(target_roll, target_pitch, target_yaw);
    }
    inline tf::Vector3 getCurrComPos() {return curr_com_pos_;}
    inline tf::Vector3 getCurrComRPY() {return curr_com_rpy_;}
    inline tf::Vector3 getCurrComVel() {return curr_vel_candidate_;}
    inline tf::Vector3 getCurrComOmega() {return curr_omega_candidate_;}
    inline tf::Vector3 getTargetVelCand() {return target_vel_candidate_;}
    inline tf::Vector3 getTargetOmegaCand() {return target_omega_candidate_;}
    inline tf::Vector3 getErrVelCand() {return err_vel_candidate_;}
    inline tf::Vector3 getErrOmegaCand() {return err_omega_candidate_;}

  protected:
    std::mutex mutex_com2base_;
    
    void calcCenterOfMoving() override;
    void rosParamInit() override;
    void updateEntSysState();
    void updateMyState();
    void updateAssemblyTree();

    void assemblyNavCallback(const aerial_robot_msgs::FlightNavConstPtr & msg) override;
    void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg) override;
    void naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg) override;

  private:
    void convertTargetPosFromCoG2CoM() override;
    void setGoalCoMRotCallback(const spinal::DesireCoordConstPtr & msg);
    void assemblyJointPosCallback(const sensor_msgs::JointStateConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& state);
    void moduleJointsCallback(const sensor_msgs::JointStateConstPtr& state);
    void jointsCtrlCallback(const sensor_msgs::JointStateConstPtr& state);
    void comRotationProcess();
    void comMovingProcess();

    ros::Publisher target_com_pose_pub_;
    boost::shared_ptr<NinjaRobotModel> ninja_robot_model_;
    ros::Subscriber target_com_rot_sub_;
    ros::Subscriber target_joints_pos_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber joint_ctrl_sub_;
    ros::Subscriber target_com_pos_sub_;
    ros::Publisher joint_control_pub_;
    ros::Publisher dock_joints_pos_pub_;
    map<string, ros::Subscriber> module_joints_subs_;

    tf::Vector3 target_final_pos_candidate_;
    tf::Vector3 target_vel_candidate_;
    tf::Vector3 target_omega_candidate_;
    tf::Vector3 curr_com_pos_;
    tf::Vector3 curr_com_rpy_;
    tf::Vector3 curr_omega_candidate_;
    tf::Vector3 curr_vel_candidate_;
    tf::Vector3 err_omega_candidate_;
    tf::Vector3 err_vel_candidate_;
    

    double asm_vel_nav_threshold_;
    double asm_nav_vel_limit_;
    int asm_xy_control_mode_;
    bool asm_vel_based_waypoint_;
    double asm_teleop_reset_duration_;
    double asm_teleop_reset_time_;

    std::map<int, ModuleData> assembled_modules_data_;
    std::vector<double> joint_pos_errs_;
    std::map<int, KDL::JntArray> all_modules_joints_pos_;
    KDL::JntArray my_crr_joints_pos_;
    KDL::JntArray my_tgt_joints_pos_;

    KDL::Rotation goal_com_rot_;
    KDL::Rotation target_com_rot_;
    KDL::Frame com2base_;
    KDL::Frame test_frame_;
    KDL::Frame curr_com_pose_;
    KDL::Frame prev_com_pose_;

    int module_joint_num_;
    double default_morphing_vel_;
    double joint_pos_chnage_thresh_;
    int joint_process_func_;

    bool free_joint_flag_;

    double morphing_process_interval_;
    double prev_morphing_stamp_;
    double prev_calc_err_stamp_;
    bool morphing_flag_;

    double com_roll_change_thresh_;
    double com_pitch_change_thresh_;
    double com_yaw_change_thresh_;

    bool disassembly_flag_;

    std::mutex mutex_cand_vel_;
    std::mutex mutex_cand_omega_;

    double pseudo_cog_com_dist_;
    double pseudo_radius_change_rate_;

    bool yaw_teleop_flag_;
    double pure_vel_control_init_z_;

    bool pure_vel_control_flag_;
    bool end_efct_mode_;

    IirFilter lpf_vel_;
    IirFilter lpf_omega_;
    bool lpf_init_flag_;
    
  };
  template<> inline KDL::Frame NinjaNavigator::getCom2Base()
  {
    std::lock_guard<std::mutex> lock(mutex_com2base_);
    return com2base_;
  }

  template<> inline Eigen::Affine3d NinjaNavigator::getCom2Base()
  {
    return aerial_robot_model::kdlToEigen(NinjaNavigator::getCom2Base<KDL::Frame>());
  }  
};

// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/gimbalrotor_navigation.h>
#include <beetle/model/beetle_robot_model.h>
#include <diagnostic_msgs/KeyValue.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <algorithm>
#include <cctype>

namespace aerial_robot_navigation
{
  /* target frmae*/
  enum target_framea
    {
     COG,
     BASE_LINK,
     CONTACT_POINT,
     CENTER_OF_MOVING
    };
  class BeetleNavigator : public GimbalrotorNavigator
  {
  public:
    BeetleNavigator();
    ~BeetleNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    void update() override;

    inline void setTargetPosCandX( float value){  target_pos_candidate_.setX(value);}
    inline void setTargetPosCandY( float value){  target_pos_candidate_.setY(value);}
    inline void setTargetPosCandZ( float value){  target_pos_candidate_.setZ(value);}
    inline void setTargetPosCand( tf::Vector3 value){  target_pos_candidate_ = value ;}
    inline tf::Vector3 getTargetPosCand() {return target_pos_candidate_;}
    template<class T> T getCog2CoM();
    bool getCurrentAssembled(){return current_assembled_;}
    int getModuleState(){return module_state_;}
    int getReconfigFlag(){return reconfig_flag_;}
    int getMyID(){return my_id_;}
    std::string getMyName(){return my_name_;}
    int getLeaderID(){return leader_id_;}
    std::vector<int> getModuleIDs(){return assembled_modules_ids_;}
    bool getControlFlag(){return control_flag_;}
    int getModuleNum(){return module_num_;}

    void setCog2CoM(const KDL::Frame Cog2CoM){Cog2CoM_ = Cog2CoM;}
    void setModuleNum(const int module_num){module_num_ = module_num;}
    void setAssemblyFlag(const int key, const bool value){
      assembly_flags_[key] = value;
    }
    void setControlFlag(const bool control_flag){control_flag_ = control_flag;}
    void setLeaderID(const int leader_id){
      leader_id_ = leader_id;
      leader_fix_flag_ = true;
    }

    void setLeaderFixFlag(const bool leader_fix_flag){
      leader_fix_flag_ = leader_fix_flag;
    }

    std::map<int, bool> getAssemblyFlags(){return assembly_flags_;}
    int getMaxModuleNum(){return max_modules_num_;}

    virtual void calcCenterOfMoving();  


  protected:
    ros::Publisher cog_com_dist_pub_;
    std::mutex mutex_cog2com_;
    KDL::Frame Cog2CoM_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformBroadcaster br_;
    int max_modules_num_ = 4; //TODO: get the value from rosparam
    int pre_assembled_modules_;
    int my_id_;
    std::string my_name_;
    int leader_id_;
    std::map<int, bool> assembly_flags_;
    bool reconfig_flag_;
    bool current_assembled_;
    bool control_flag_;
    bool leader_fix_flag_;
    std::vector<int> assembled_modules_ids_;
    int module_state_;
    int module_num_;

    void rosParamInit() override;
    virtual void convertTargetPosFromCoG2CoM();

  private:
    tf::Vector3 target_pos_candidate_;
    tf::Vector3 pre_target_pos_;
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Subscriber assembly_nav_sub_;
    ros::Subscriber assembly_target_rot_sub_;

    double max_target_roll_pitch_rate_;

    bool roll_pitch_control_flag_;
    bool pre_assembled_ ; 

    void naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg) override;
    void assemblyNavCallback(const aerial_robot_msgs::FlightNavConstPtr & msg);
    void setAssemblyFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg);
    void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg) override; 
    void rotateContactPointFrame();
    boost::shared_ptr<BeetleRobotModel> beetle_robot_model_;
    map<string, ros::Subscriber> assembly_flag_subs_;
    void assemblyFlagCallback(const diagnostic_msgs::KeyValue & msg);
  };

  template<> inline KDL::Frame BeetleNavigator::getCog2CoM()
  {
    std::lock_guard<std::mutex> lock(mutex_cog2com_);
    return Cog2CoM_;
  }
};

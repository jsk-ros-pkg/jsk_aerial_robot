// **** the dynamicxel motor number is opposite to the joint num, should be fixed

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <dynamixel_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <hydrus_transform_control/ServoState.h>
#include <hydrus_transform_control/ServoControl.h>
#include <string>

typedef struct{
  ros::Subscriber joint_state_sub;
  std::string joint_state_sub_name;
  ros::Publisher joint_ctrl_pub;
  std::string joint_ctrl_pub_name;
  std::string joint_name;
  float current_angle;
  float target_joint_angle;
  uint16_t target_servo_angle; // same with "target_joint_angle", but different resolution.
  int angle_sgn;
  double angle_offset;
  double angle_scale;
  double angle_max;
  double angle_min;
  uint16_t prev_servo_angle; /* the previous servo angle in 14bit */
  bool moving; /* for dynamixel_msgs */
  uint8_t load; /* for dynamixel_msgs */
  uint8_t temp; /* for dynamixel_msgs */
  uint8_t error; /* for dynamixel_msgs */
}JointInfo;


class HydrusJoints
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  int joint_num_;
  int  bridge_mode_;
  std::vector<JointInfo> joint_module_;

  std::string joints_torque_control_srv_name_;
  ros::ServiceServer joints_torque_control_srv_;
  ros::ServiceClient joints_torque_control_client_;

  ros::Subscriber servo_angle_sub_; //current servo angles from MCU
  ros::Publisher servo_ctrl_pub_; //target servo angles to MCU
  ros::Publisher servo_config_cmd_pub_; //config command to MCU
  std::string servo_sub_name_, servo_pub_name_, servo_config_cmd_pub_name_;

  ros::Publisher joints_state_pub_;
  ros::Subscriber joints_ctrl_sub_;
  std::string joints_ctrl_sub_name_;

  ros::Publisher dynamixel_msg_pub_;
  std::string dynamixel_msg_pub_name_;

  ros::ServiceServer overload_check_activate_srv_;
  std::string overload_check_activate_srv_name_;

  bool torque_flag_;
  bool start_flag_;
  uint16_t servo_on_mask_, servo_full_on_mask_;

  double moving_check_rate_;
  int moving_angle_thresh_;

  bool overload_check_;

  double bridge_rate_;
  ros::Timer  bridge_timer_;

public:
  static const uint8_t DYNAMIXEL_HUB_MODE = 0;
  static const uint8_t MCU_MODE = 1;
  static const uint8_t OVERLOAD_FLAG = 0x20;

  HydrusJoints(ros::NodeHandle nh, ros::NodeHandle nhp)
    : nh_(nh),nhp_(nhp)
  {
    nhp_.param("joint_num", joint_num_, 3);
    nhp_.param("bridge_mode", bridge_mode_, 0);

    servo_on_mask_ = 0;
    servo_full_on_mask_ = 0;
    torque_flag_ = true;

    overload_check_ = true; /* check overload automatically */

    joint_module_.resize(joint_num_);

    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;

        nhp_.param(std::string("joint") + joint_no.str() + std::string("_angle_max"), joint_module_[i].angle_max, 1.57); //real angle
        nhp_.param(std::string("joint") + joint_no.str() + std::string("_angle_min"), joint_module_[i].angle_min, -1.57); //real angle
        nhp_.param(std::string("joint") + joint_no.str() + std::string("_angle_sgn"), joint_module_[i].angle_sgn, 1);
        nhp_.param(std::string("joint") + joint_no.str() + std::string("_angle_offset"), joint_module_[i].angle_offset, 0.0);
        nhp_.param(std::string("joint") + joint_no.str() + std::string("_angle_scale"), joint_module_[i].angle_scale, 1.0);

        nhp_.param(std::string("joint") + joint_no.str() + std::string("_name"), joint_module_[i].joint_name, std::string("joint") + joint_no.str());

        ROS_INFO("joint%d attribute: angle_max: %f, angle_min: %f, angle_scale: %f, angle_sng: %d, angle_offset: %f", i + 1, joint_module_[i].angle_max, joint_module_[i].angle_min, joint_module_[i].angle_scale, joint_module_[i].angle_sgn, joint_module_[i].angle_offset);

        if(bridge_mode_ == DYNAMIXEL_HUB_MODE)
          {
            joint_module_[i].joint_ctrl_pub_name = std::string("/j") + joint_no.str()  + std::string("_controller/command");
            joint_module_[i].joint_ctrl_pub = nh_.advertise<std_msgs::Float64>(joint_module_[i].joint_ctrl_pub_name, 1);

            joint_module_[i].joint_state_sub_name = std::string("/j") + joint_no.str()  + std::string("_controller/state");
            joint_module_[i].joint_state_sub = nh_.subscribe<dynamixel_msgs::JointState>(joint_module_[i].joint_state_sub_name, 1, boost::bind(&HydrusJoints::jointCallback, this, _1, i));
          }

        servo_full_on_mask_ |= (1 << i);
      }

    nhp_.param("joints_torque_control_srv_name", joints_torque_control_srv_name_, std::string("/joints_controller/torque_enable"));
    joints_torque_control_srv_ =  nh_.advertiseService(joints_torque_control_srv_name_, &HydrusJoints::jointsTorqueEnableCallback, this);
    joints_torque_control_client_ = nh_.serviceClient<dynamixel_controllers::TorqueEnable>(joints_torque_control_srv_name_);

    nhp_.param("joints_ctrl_sub_name", joints_ctrl_sub_name_, std::string("hydrus/joints_ctrl"));
    joints_ctrl_sub_ = nh_.subscribe<sensor_msgs::JointState>(joints_ctrl_sub_name_, 1, &HydrusJoints::jointsCtrlCallback, this, ros::TransportHints().tcpNoDelay());
    joints_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

    if(bridge_mode_ == MCU_MODE)
      {
        nhp_.param("servo_sub_name", servo_sub_name_, std::string("servo_states"));
        nhp_.param("servo_pub_name", servo_pub_name_, std::string("target_servo_states"));
        nhp_.param("servo_config_cmd_pub_name", servo_config_cmd_pub_name_, std::string("servo_config_cmd"));

        nhp_.param("moving_check_rate", moving_check_rate_, 10.0);
        nhp_.param("moving_angle_thresh", moving_angle_thresh_, 2);
        nhp_.param("dynamixel_msg_pub_name", dynamixel_msg_pub_name_, std::string("joint_motors"));
        nhp_.param("overload_check_activate_srv_name", overload_check_activate_srv_name_, std::string("overload_check_activate"));

        servo_angle_sub_ = nh_.subscribe<hydrus_transform_control::ServoState>(servo_sub_name_, 1, &HydrusJoints::servoStateCallback, this, ros::TransportHints().tcpNoDelay());
        servo_ctrl_pub_ = nh_.advertise<hydrus_transform_control::ServoControl>(servo_pub_name_, 1);
        servo_config_cmd_pub_ = nh_.advertise<std_msgs::UInt8>(servo_config_cmd_pub_name_, 1);

        /* dynamixel msg */
        dynamixel_msg_pub_ = nh_.advertise<dynamixel_msgs::MotorStateList>(dynamixel_msg_pub_name_, 1);

        /* overload check activate flag */
        overload_check_activate_srv_ = nh_.advertiseService(overload_check_activate_srv_name_, &HydrusJoints::overloadCheckActivateCallback, this);

      }

    nhp_.param("bridge_rate", bridge_rate_, 40.0);
    bridge_timer_ = nhp_.createTimer(ros::Duration(1.0 / bridge_rate_), &HydrusJoints::bridgeFunc, this);
  }

  ~HydrusJoints()
  {
  }

  void setCurrentAngle(double curr_angle, int i)
  {
    servo_on_mask_ |= (1 << i);

    joint_module_[i].current_angle = joint_module_[i].angle_scale * joint_module_[i].angle_sgn * (curr_angle - joint_module_[i].angle_offset);
  }

  void jointCallback(const dynamixel_msgs::JointStateConstPtr& msg, int i)
  {
    setCurrentAngle(msg->current_pos, i);
  }

  void servoStateCallback(const hydrus_transform_control::ServoStateConstPtr& state_msg)
  {
    static ros::Time prev_time = ros::Time::now();
    for(int i = 0; i < joint_num_; i ++)
      {
        setCurrentAngle(state_msg->angle[i], i);
        joint_module_[i].error = state_msg->error[i];
        joint_module_[i].temp = state_msg->temp[i];
        joint_module_[i].load = state_msg->load[i];
      }

    /* check moving */
    if(ros::Time::now().toSec() - prev_time.toSec() > (1 / moving_check_rate_))
      {
        for(int i = 0; i < joint_num_; i ++)
          {
            if(abs(state_msg->angle[i] - joint_module_[i].prev_servo_angle) > moving_angle_thresh_)
              {
                joint_module_[i].moving = true;
                joint_module_[i].prev_servo_angle = state_msg->angle[i];
              }
            else joint_module_[i].moving = false;
          }

        prev_time = ros::Time::now();
      }

    /* check overload */
    if(overload_check_)
      {
        for(int i = 0; i < joint_num_; i ++)
          {
            if(OVERLOAD_FLAG & state_msg->error[i])
              {
                ROS_WARN("motor: %d, overload", i+1);

                dynamixel_controllers::TorqueEnable srv;
                srv.request.torque_enable = false;
                if (joints_torque_control_client_.call(srv))
                  return;
                else
                  ROS_ERROR("Failed to call service joints_torque_control_client");
              }
          }
      }
  }

  void jointsCtrlCallback(const sensor_msgs::JointStateConstPtr& joints_ctrl_msg)
  {
    hydrus_transform_control::ServoControl target_angle_msg;

    for(int i = 0; i < joint_num_; i ++)
      {
        joint_module_[i].target_joint_angle = joints_ctrl_msg->position[i];
        /* TODO: please confirm hydrus3 model */
        double target_angle = joint_module_[i].target_joint_angle * joint_module_[i].angle_sgn / joint_module_[i].angle_scale  + joint_module_[i].angle_offset;

        if(bridge_mode_ == DYNAMIXEL_HUB_MODE)
          {
            std_msgs::Float64 command;
            command.data = target_angle;
            joint_module_[i].joint_ctrl_pub.publish(command);
          }
        else if(bridge_mode_ == MCU_MODE)
          target_angle_msg.angle[i] = target_angle;
      }

    if(bridge_mode_ == MCU_MODE) servo_ctrl_pub_.publish(target_angle_msg);
  }

  bool jointsTorqueEnableCallback(dynamixel_controllers::TorqueEnable::Request &req, dynamixel_controllers::TorqueEnable::Response &res)
  {
    if(bridge_mode_ == DYNAMIXEL_HUB_MODE)
      {
        for(int i = 0; i < joint_num_; i ++)
          {
            std::stringstream joint_no;
            joint_no << i + 1;

            std::string srv_name = std::string("/j") + joint_no.str()  + std::string("_controller/torque_enable");

            ros::ServiceClient client = nh_.serviceClient<dynamixel_controllers::TorqueEnable>(srv_name);
            dynamixel_controllers::TorqueEnable srv;
            srv.request.torque_enable = req.torque_enable;
            if (client.call(srv))
              {
                if(req.torque_enable) ROS_INFO("joint%d: enable torque", i+1);
                else ROS_INFO("joint%d: disable torque", i+1);
                return true;
              }
            else
              {
                ROS_ERROR("Failed to call service %s", srv_name.c_str());
                return false;
              }
          }
      }
    else if(bridge_mode_ == MCU_MODE)
      {
        std_msgs::UInt8 enable_torque;
        if(req.torque_enable) enable_torque.data = 1;
        else enable_torque.data = 0;
        servo_config_cmd_pub_.publish(enable_torque);
        return true;
      }
  }

  bool overloadCheckActivateCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    overload_check_ = req.data;

    res.success = true;
    return true;
  }

  void bridgeFunc(const ros::TimerEvent & e)
  {
    if(servo_on_mask_ != servo_full_on_mask_) return;
    jointStatePublish();
  }

  void jointStatePublish()
  {
    sensor_msgs::JointState hydrus_joints_state;
    hydrus_joints_state.header.stamp = ros::Time::now();
    hydrus_joints_state.name.resize(joint_num_);
    hydrus_joints_state.position.resize(joint_num_);

    /* four link: the normal-approximation singular position(pi/2): only if all joints are within 5[deg] range. */
    bool normal_approximation = true;
    if(joint_num_ == 3)
      {
        for(int i = 0; i < joint_num_; i ++)
          {
            if(fabs(joint_module_[i].current_angle - M_PI/2) > 0.085 && fabs(joint_module_[i].current_angle + M_PI/2) > 0.085)
              normal_approximation = false;
          }
      }
    else normal_approximation = false;

    for(int i = 0; i < joint_num_; i ++)
      {
        hydrus_joints_state.name[i] = joint_module_[i].joint_name;
        if(normal_approximation)
          {
            if(fabs(joint_module_[i].current_angle - M_PI/2) < 0.085)
              hydrus_joints_state.position[i] = M_PI/2;
            if(fabs(joint_module_[i].current_angle + M_PI/2) < 0.085)
              hydrus_joints_state.position[i] = -M_PI/2;
          }
        else
          hydrus_joints_state.position[i] = joint_module_[i].current_angle;
      }

    joints_state_pub_.publish(hydrus_joints_state);

    /* need to publish dynamixel msg */
    if(bridge_mode_ == MCU_MODE)
      {
        dynamixel_msgs::MotorStateList dynamixel_msg;
        for(int i = 0; i < joint_num_; i ++)
          {
            dynamixel_msgs::MotorState motor_msg;
            motor_msg.timestamp = ros::Time::now().toSec();
            motor_msg.id = i;
            motor_msg.error = joint_module_[i].error;
            motor_msg.load = joint_module_[i].load;
            motor_msg.moving = joint_module_[i].moving;
            motor_msg.temperature = joint_module_[i].temp;
            dynamixel_msg.motor_states.push_back(motor_msg);
          }

        dynamixel_msg_pub_.publish(dynamixel_msg);
      }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hydrus_joitns");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  HydrusJoints *hydrus_joints = new HydrusJoints(n,np);
  ros::spin();
  delete hydrus_joints;

  return 0;
}

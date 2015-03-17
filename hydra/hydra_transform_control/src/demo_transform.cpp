#include <ros/ros.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <string>

#include <aerial_robot_msgs/RPYCtrlOffset.h>
#include <aerial_robot_msgs/AttGains.h>


typedef struct{
  int pos_roll_i_term;
  int pos_pitch_i_term;
  int pos_yaw_i_term;
  
  int att_roll_pitch_p_gain;
  int att_roll_d_gain;
  int att_pitch_d_gain;
  int att_yaw_d_gain;

  double pos_roll_p_gain;
  double pos_pitch_p_gain;
  double pos_yaw_p_gain;
  std::vector<double> joints_angle;
}node_info;



class DemoTransform
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  std::vector<node_info> nodes_info_;
  int node_sequence_;
  int node_num_;
  int joint_num_;

  ros::Subscriber joints_state_sub_; 
  ros::Subscriber transform_start_sub_;
  ros::Publisher joints_ctrl_pub_; //for joint => to dynamixel
  ros::Publisher pos_i_term_pub_; //for i control term => to jsk_quadcopter(flight control)
  ros::Publisher transform_convergent_pub_; //for i control term => to jsk_quadcopter(flight control)
  ros::Publisher att_gain_pub_;   //for attitude control gain => to kduino directly

  int transform_start_flag_;

  ros::Timer  ctrl_timer_;
  double ctrl_rate_;

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joints_state_msg)
  {
    if(transform_start_flag_)
      {
        //rate
        float rate_up = 0;
        float rate_down = 0;
        for(int i = 0; i < (int)joints_state_msg->position.size(); i++)
          {
            rate_up += fabs(joints_state_msg->position[i] - nodes_info_[node_sequence_].joints_angle[i]);
            rate_down += fabs(nodes_info_[node_sequence_+1].joints_angle[i] - nodes_info_[node_sequence_].joints_angle[i]);
          }
        float rate = rate_up / rate_down;
        //ROS_INFO("rate: %f", rate);

        //i term offset
        aerial_robot_msgs::RPYCtrlOffset i_term_offset;
        i_term_offset.roll_offset = nodes_info_[node_sequence_].pos_roll_i_term + (nodes_info_[node_sequence_+1].pos_roll_i_term - nodes_info_[node_sequence_].pos_roll_i_term)* rate;
        i_term_offset.pitch_offset = nodes_info_[node_sequence_].pos_pitch_i_term + (nodes_info_[node_sequence_+1].pos_pitch_i_term - nodes_info_[node_sequence_].pos_pitch_i_term)* rate;
        pos_i_term_pub_.publish(i_term_offset);

        //att gain
        aerial_robot_msgs::AttGains att_gains;
        att_gains.roll_d_gain = (uint8_t)nodes_info_[node_sequence_].att_roll_d_gain + (uint8_t)((nodes_info_[node_sequence_+1].att_roll_d_gain - nodes_info_[node_sequence_].att_roll_d_gain)* rate);
        att_gains.pitch_d_gain = (uint8_t)nodes_info_[node_sequence_].att_pitch_d_gain + (uint8_t)((nodes_info_[node_sequence_+1].att_pitch_d_gain - nodes_info_[node_sequence_].att_pitch_d_gain)* rate);
        att_gain_pub_.publish(att_gains);

        //node shift process
        bool node_shift_flag = true;
        for(int i = 0 ; i < joint_num_; i ++)
          {
            if(fabs(joints_state_msg->position[i] - nodes_info_[node_sequence_ + 1].joints_angle[i]) > 0.1)
              node_shift_flag = 0;
          }
        if(node_shift_flag) 
          {
            node_sequence_++;
            if(node_sequence_ == node_num_ - 1)
              {
                ROS_INFO("complete the transform");
                transform_start_flag_ = 0;
              }
            else
              {
                if(transform_start_flag_ == CONTINUOUS_MODE_)
                  {
                    sensor_msgs::JointState joints_ctrl_msg;
                    joints_ctrl_msg.position.resize(joint_num_);
                    for(int i = 0; i < 3; i ++)
                      joints_ctrl_msg.position[i] = nodes_info_[node_sequence_ + 1].joints_angle[i];
                    joints_ctrl_pub_.publish(joints_ctrl_msg);
                  }
                else if(transform_start_flag_ == DISCRETE_MODE_)
                  {
                    ROS_INFO("stop at %d step", node_sequence_);
                    transform_start_flag_ = 0;
                  }
              }
            std_msgs::Int8 trans_convergent_msg;
            trans_convergent_msg.data = node_sequence_;
            transform_convergent_pub_.publish(trans_convergent_msg);
          }
      }
      
  }

  void transformStartCallback(const std_msgs::Int8ConstPtr& msg)
  {
    transform_start_flag_ = msg->data;
      

    //initialize pose:joint1:M_PI/2, joint2:M_PI/2, joint3:M_PI_PI/2
    sensor_msgs::JointState joints_ctrl_msg;
    joints_ctrl_msg.position.resize(joint_num_);
    for(int i = 0; i < 3; i ++)
      joints_ctrl_msg.position[i] = nodes_info_[node_sequence_ + 1].joints_angle[i];
    joints_ctrl_pub_.publish(joints_ctrl_msg);

    //i term offset
    aerial_robot_msgs::RPYCtrlOffset i_term_offset;
    i_term_offset.roll_offset = nodes_info_[node_sequence_].pos_roll_i_term;
    i_term_offset.pitch_offset = nodes_info_[node_sequence_].pos_pitch_i_term;
    pos_i_term_pub_.publish(i_term_offset);

    //att gain
    aerial_robot_msgs::AttGains att_gains;
    att_gains.roll_d_gain = (uint8_t)nodes_info_[node_sequence_].att_roll_d_gain;
    att_gains.pitch_d_gain = (uint8_t)nodes_info_[node_sequence_].att_pitch_d_gain;
    att_gain_pub_.publish(att_gains);


  }

public:

  DemoTransform(ros::NodeHandle nh, ros::NodeHandle nhp)
    : nh_(nh),nhp_(nhp)
  {
    std::string ns = nhp_.getNamespace();
    ROS_INFO("%s", ns.c_str());
    nhp_.param("node_num", node_num_, 13);
    nhp_.param("joint_num", joint_num_, 3);
    
    node_sequence_ = 0;
    nodes_info_.resize(node_num_);
    for(int i = 0; i < node_num_; i++)
      {
        std::stringstream node_no;
        node_no << i;
        nhp_.param(std::string("node") + node_no.str() + std::string("_pos_roll_i_term"), nodes_info_[i].pos_roll_i_term, 0);
        nhp_.param(std::string("node") + node_no.str() + std::string("_pos_pitch_i_term"), nodes_info_[i].pos_pitch_i_term, 0);
        nhp_.param(std::string("node") + node_no.str() + std::string("_att_roll_pitch_p_gain"), nodes_info_[i].att_roll_pitch_p_gain, 0);
        nhp_.param(std::string("node") + node_no.str() + std::string("_att_roll_d_gain"), nodes_info_[i].att_roll_d_gain, 0);
        nhp_.param(std::string("node") + node_no.str() + std::string("_att_pitch_d_gain"), nodes_info_[i].att_pitch_d_gain, 0);
        
        nodes_info_[i].joints_angle.resize(joint_num_);
        ROS_INFO("node:%d, pos_roll_i_term:%d, pos_pitch_i_term:%d,att_roll_pitch_p_gain:%d,att_roll_d_gain:%d,att_pitch_d_gain:%d",
                 i, nodes_info_[i].pos_roll_i_term, nodes_info_[i].pos_pitch_i_term, nodes_info_[i].att_roll_pitch_p_gain,
                 nodes_info_[i].att_roll_d_gain,nodes_info_[i].att_pitch_d_gain);

        for(int j = 0 ; j < joint_num_; j++)
          {
            std::stringstream joint_no;
            joint_no << j;
            nhp_.param(std::string("node") + node_no.str() + std::string("_joint") + joint_no.str() + std::string("_angle"), nodes_info_[i].joints_angle[j], 0.0);
            ROS_INFO("joint%d_angle: %f",j,nodes_info_[i].joints_angle[j]);
          }
      }
    

    joints_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("hydra/joint_states", 1, &DemoTransform::jointStateCallback, this, ros::TransportHints().tcpNoDelay());

    joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("hydra/joints_ctrl", 1);
    transform_convergent_pub_ = nh_.advertise<std_msgs::Int8>("hydra/transform_convergent_flag", 1);
    pos_i_term_pub_ = nh_.advertise<aerial_robot_msgs::RPYCtrlOffset>("/flight_control/rpy_ctrl_offset", 1);
    att_gain_pub_ = nh_.advertise<aerial_robot_msgs::AttGains>("kduino/att_gains", 1);
    transform_start_sub_ = nh_.subscribe<std_msgs::Int8>("/teleop_command/transform_start",  1, &DemoTransform::transformStartCallback, this, ros::TransportHints().tcpNoDelay());
    transform_start_flag_ = 0;
    
    // nhp_.param("ctrl_rate", ctrl_rate_, 40.0);
    // ctrl_timer_ = nhp_.createTimer(ros::Duration(1.0 / ctrl_rate_), &DemoTransform::ctrlFunc, this);

  }

  ~DemoTransform()
  {
  }

  static const uint8_t CONTINUOUS_MODE_ = 1;
  static const uint8_t DISCRETE_MODE_ = 2;


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_transform");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  DemoTransform *demo_transform = new DemoTransform(n,np);
  ros::spin();
  delete demo_transform;

  return 0;
}


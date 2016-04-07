/*
  1. retime is bad => uniform the time stamp in the aerial robot system

 */

#ifndef MOCAP_DATA_H
#define MOCAP_DATA_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <kalman_filter/kf_pos_vel_acc.h>
#include <kalman_filter/digital_filter.h>
#include <tf/transform_listener.h>

#include <aerial_robot_base/States.h>
#include <aerial_robot_base/ImuData.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>


class MocapData
{
 public:

 MocapData(ros::NodeHandle nh, ros::NodeHandle nh_private, BasicEstimator* estimator)
   : nh_(nh, "mocap"), nhp_(nh_private, "mocap")
    {

      estimator_ = estimator;

      rosParamInit(nhp_);

      mocap_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/aerial_robot/pose", 1, &MocapData::poseCallback, this, ros::TransportHints().udp());


      if(cog_offset_)
        {
          cog_offset_sub_ = nh_.subscribe<std_msgs::Float32>(cog_rotate_sub_name_, 5, &MocapData::cogOffsetCallback, this, ros::TransportHints().tcpNoDelay()); 
        }

      //debug, can be delete
      pose_stamped_pub_ = nh_.advertise<aerial_robot_base::States>(pub_name_, 5); 


      //low pass filter
      lpf_pos_x_ =  IirFilter((float)rx_freq_,
                              (float)cutoff_pos_freq_,
                              (float)cutoff_vel_freq_);
      lpf_pos_y_ =  IirFilter((float)rx_freq_,
                              (float)cutoff_pos_freq_,
                              (float)cutoff_vel_freq_);
      lpf_pos_z_ =  IirFilter((float)rx_freq_,
                              (float)cutoff_pos_freq_,
                              (float)cutoff_vel_freq_);
      lpf_pos_psi_ =  IirFilter((float)rx_freq_,
                                (float)cutoff_pos_freq_,
                                (float)cutoff_vel_freq_);

      lpf_acc_x_ =  IirFilter((float)rx_freq_,
                              (float)cutoff_pos_freq_);
      lpf_acc_y_ =  IirFilter((float)rx_freq_,
                              (float)cutoff_pos_freq_);
      lpf_acc_z_ =  IirFilter((float)rx_freq_,
                              (float)cutoff_pos_freq_);

      estimator_->setOuterEstimatePoseFlag(BasicEstimator::X_AXIS | BasicEstimator::Y_AXIS | BasicEstimator::Z_AXIS | BasicEstimator::YAW_AXIS);
      estimator_->setOuterEstimateVelFlag(BasicEstimator::X_AXIS | BasicEstimator::Y_AXIS | BasicEstimator::Z_AXIS | BasicEstimator::YAW_AXIS);

    }
  ~MocapData()
    {
    }

  static const int TIME_SYNC_CALIB_COUNT = 10;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher  pose_stamped_pub_;
  ros::Subscriber mocap_sub_;
  ros::Subscriber cog_offset_sub_;

  BasicEstimator* estimator_;


  float cog_offset_angle_;
  bool cog_offset_;


  IirFilter lpf_pos_x_, lpf_pos_y_, lpf_pos_z_, lpf_pos_psi_;
  IirFilter lpf_acc_x_, lpf_acc_y_, lpf_acc_z_;
  double rx_freq_;
  double cutoff_pos_freq_;
  double cutoff_vel_freq_;
  std::string pub_name_;
  std::string cog_rotate_sub_name_;

  float pos_x_offset_, pos_y_offset_, pos_z_offset_;

  void poseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
  {

    static float prev_pos_x, prev_pos_y, prev_pos_z, prev_psi, prev_phy, prev_theta;
    static float prev_vel_x, prev_vel_y, prev_vel_z;
    static bool first_flag = true;
    static ros::Time previous_time;

    double raw_pos_x = 0, raw_pos_y = 0, raw_pos_z = 0, raw_theta = 0, raw_phy = 0, raw_psi = 0;
    double raw_vel_x = 0, raw_vel_y = 0, raw_vel_z = 0, raw_vel_theta = 0, raw_vel_phy = 0, raw_vel_psi = 0;
    double pos_x = 0, pos_y = 0, pos_z = 0, theta = 0, phy = 0, psi = 0;
    double vel_x = 0, vel_y = 0, vel_z = 0, vel_theta = 0, vel_phy = 0, vel_psi = 0;

    double raw_acc_x = 0, raw_acc_y = 0, raw_acc_z = 0;
    double acc_x = 0, acc_y = 0, acc_z = 0;

    static uint64_t time_offset=0;

    if(!first_flag)
      {
        raw_pos_x = msg->pose.position.x - pos_x_offset_;
        raw_vel_x = (msg->pose.position.x - prev_pos_x) / (msg->header.stamp.toSec() - previous_time.toSec());
        raw_acc_x = (raw_vel_x - prev_vel_x) / (msg->header.stamp.toSec() - previous_time.toSec());
        raw_pos_y = msg->pose.position.y - pos_y_offset_;
        raw_vel_y = (msg->pose.position.y - prev_pos_y) / (msg->header.stamp.toSec() - previous_time.toSec());
        raw_acc_y = (raw_vel_y - prev_vel_y) / (msg->header.stamp.toSec() - previous_time.toSec());
        raw_pos_z = msg->pose.position.z - pos_z_offset_;
        raw_vel_z = (msg->pose.position.z - prev_pos_z) / (msg->header.stamp.toSec() - previous_time.toSec());
        raw_acc_z = (raw_vel_z - prev_vel_z) / (msg->header.stamp.toSec() - previous_time.toSec());

        tf::Quaternion q(msg->pose.orientation.x,
                         msg->pose.orientation.y,
                         msg->pose.orientation.z,
                         msg->pose.orientation.w);
        tf::Matrix3x3(q).getRPY(raw_phy, raw_theta, raw_psi);
        raw_vel_phy = (raw_phy - prev_phy) / (msg->header.stamp.toSec() - previous_time.toSec());
        raw_vel_theta = (raw_theta - prev_theta) / (msg->header.stamp.toSec() - previous_time.toSec());
        raw_vel_psi = (raw_psi - prev_psi) / (msg->header.stamp.toSec() - previous_time.toSec());

        lpf_pos_x_.filterFunction(raw_pos_x, pos_x, raw_vel_x, vel_x);
        lpf_pos_y_.filterFunction(raw_pos_y, pos_y, raw_vel_y, vel_y);
        lpf_pos_z_.filterFunction(raw_pos_z, pos_z, raw_vel_z, vel_z);
        lpf_pos_psi_.filterFunction(raw_psi, psi, raw_vel_psi, vel_psi);

        lpf_acc_x_.filterFunction(raw_acc_x, acc_x);
        lpf_acc_y_.filterFunction(raw_acc_y, acc_y);
        lpf_acc_z_.filterFunction(raw_acc_z, acc_z);

        //correct way
        estimator_->setStatePosX(pos_x);
        estimator_->setStatePosY(pos_y);
        estimator_->setStatePosZ(pos_z);
        estimator_->setStatePsiCog(psi + cog_offset_angle_);
        estimator_->setStatePsiBoard(psi);
        //ROS_INFO("angle from map to cog is %f, %f, %f, %f", psi + cog_offset_angle_, pos_x, pos_y, pos_z);
        estimator_->setStateVelX(vel_x);
        estimator_->setStateVelY(vel_y);
        estimator_->setStateVelZ(vel_z);
        estimator_->setStateVelPsi(vel_psi);

        //publish for deubg, can delete
        aerial_robot_base::States ground_truth_pose;
        //ground_truth_pose.header.stamp.fromNSec(msg->header.stamp.toNSec()+ time_offset);
        ground_truth_pose.header.stamp = msg->header.stamp;

        aerial_robot_base::State x_state;
        x_state.id = "x";
        x_state.raw_pos = raw_pos_x;
        x_state.raw_vel = raw_vel_x;
        x_state.pos = pos_x;
        x_state.vel = vel_x;
        x_state.reserves.push_back(acc_x);
        x_state.reserves.push_back(raw_acc_x);

        aerial_robot_base::State y_state;
        y_state.id = "y";
        y_state.raw_pos = raw_pos_y;
        y_state.raw_vel = raw_vel_y;
        y_state.pos = pos_y;
        y_state.vel = vel_y;
        y_state.reserves.push_back(acc_y);
        y_state.reserves.push_back(raw_acc_y);

        aerial_robot_base::State z_state;
        z_state.id = "z";
        z_state.raw_pos = raw_pos_z;
        z_state.raw_vel = raw_vel_z;
        z_state.pos = pos_z;
        z_state.vel = vel_z;
        z_state.reserves.push_back(acc_z);
        z_state.reserves.push_back(raw_acc_z);


        aerial_robot_base::State yaw_state;
        yaw_state.id = "yaw";
        yaw_state.raw_pos = raw_psi;
        yaw_state.raw_vel = raw_vel_psi;
        yaw_state.pos = psi;
        yaw_state.vel = vel_psi;

        aerial_robot_base::State pitch_state;
        pitch_state.id = "pitch";
        pitch_state.raw_pos = raw_theta;
        pitch_state.raw_vel = raw_vel_theta;

        aerial_robot_base::State roll_state;
        roll_state.id = "roll";
        roll_state.raw_pos = raw_phy;
        roll_state.raw_vel = raw_vel_phy;

        ground_truth_pose.states.push_back(x_state);
        ground_truth_pose.states.push_back(y_state);
        ground_truth_pose.states.push_back(z_state);
        ground_truth_pose.states.push_back(yaw_state);
        ground_truth_pose.states.push_back(pitch_state);
        ground_truth_pose.states.push_back(roll_state);

        pose_stamped_pub_.publish(ground_truth_pose);

      }

    if(first_flag)
      {
        prev_pos_x = msg->pose.position.x;
        prev_pos_y = msg->pose.position.y;
        prev_pos_z = msg->pose.position.z;

        pos_x_offset_ = msg->pose.position.x;
        pos_y_offset_ = msg->pose.position.y;
        pos_z_offset_ = msg->pose.position.z;

        //time
        time_offset = ros::Time::now().toNSec() - msg->header.stamp.toNSec();

        first_flag = false;
      }

    prev_pos_x = msg->pose.position.x;
    prev_pos_y = msg->pose.position.y;
    prev_pos_z = msg->pose.position.z;
    prev_vel_x = raw_vel_x;
    prev_vel_y = raw_vel_y;
    prev_vel_z = raw_vel_z;

    prev_theta = raw_theta;
    prev_phy = raw_phy;
    prev_psi = raw_psi;
    previous_time = msg->header.stamp;


  }

  void cogOffsetCallback(std_msgs::Float32 offset_msg)
  {
    cog_offset_angle_ =  offset_msg.data;
  }


  void rosParamInit(  ros::NodeHandle nh)
  {
    std::string ns = nh.getNamespace();
    nhp_.param("pub_name", pub_name_, std::string("ground_truth/pose"));
    nhp_.param("cog_rotate_sub_name", cog_rotate_sub_name_, std::string("/cog_rotate"));

    nhp_.param("rx_freq", rx_freq_, 100.0);
    nhp_.param("cutoff_pos_freq", cutoff_pos_freq_, 20.0);
    nhp_.param("cutoff_vel_freq", cutoff_vel_freq_, 20.0);

    nh.param ("cog_offset", cog_offset_, false);

    printf("%s: COG offset is %s\n", ns.c_str(), cog_offset_ ? ("true") : ("false"));
    cog_offset_angle_ = 0;

  }


};

#endif













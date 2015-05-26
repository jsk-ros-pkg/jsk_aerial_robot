/*
  1. retime is bad => uniform the time stamp in the aerial robot system

 */

#ifndef MOCAP_DATA_H
#define MOCAP_DATA_H

//* ros
#include <ros/ros.h>
#include <aeiral_robot_base/state_estimation.h>
#include <aeiral_robot_base/digital_filter.h>
#include <tf/transform_listener.h>

#include <aeiral_robot_base/States.h>
#include <aeiral_robot_base/ImuData.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>


class MocapData
{
 public:

 MocapData(ros::NodeHandle nh, ros::NodeHandle nh_private, Estimator* estimator)
   : nh_(nh, "mocap"), nhp_(nh_private, "mocap")
    {
      rosParamInit(nhp_);

      if(!retime_flag_) time_sync_count_ = 0;
      mocap_sub_ = nodeHandle_.subscribe<geometry_msgs::Pose>("/aerial_robot/pose", 1, &MocapRetime::poseCallback, this, ros::TransportHints().tcpNoDelay());

      if(cog_offset_)
        {
          //cog_offset_sub_ = nh_.subscribe<std_msgs::Float32>("/hydra/cog_rotate", 5, &MocapData::cogOffsetCallback, this, ros::TransportHints().tcpNoDelay()); 
          cog_offset_sub_ = nh_.subscribe<std_msgs::Float32>(cog_rotate_sub_name_, 5, &MocapData::cogOffsetCallback, this, ros::TransportHints().tcpNoDelay()); 
        }

      //debug, can be delete
      pose_stamped_pub_ = nh_.advertise<aerial_robot_base::FourAxisState>(pub_name_, 5); 


      //time modification
      offset_ = 0;
      sec_offset_ = 0;
      n_sec_offset_ = 0;
      time_sync_count_ = TIME_SYNC_CALIB_COUNT;


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


 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher  pose_stamped_pub_
  ros::Subscriber mocap_sub_;
  ros::Subscriber cog_offset_sub_;


  //time modification
  bool retime_flag_;
  long offset_, sec_offset_, n_sec_offset_;
  int time_sync_count_;


  float cog_offset_angle_;
  bool cog_offset_;


  IirFilter lpf_pos_x_, lpf_pos_y_, lpf_pos_z_, lpf_pos_psi_;
  IirFilter lpf_acc_x_, lpf_acc_y_, lpf_acc_z_;
  double rx_freq_;
  double cutoff_pos_freq_;
  double cutoff_vel_freq_;
  std::string pub_name_;
  std::string cog_rotate_sub_name_;

  float pos_x_offset_, pos_x_offset_, pos_z_offset_;

  void poseCallback(const geometry_msgs::PoseConstPtr & msg)
  {

    static float prev_pos_x, prev_pos_y, prev_pos_z, prev_psi;
    static float prev_vel_x, prev_vel_y, prev_vel_z;
    static bool first_flag = true;
    static ros::Time previous_time;

    double raw_pos_x = 0, raw_pos_y = 0, raw_pos_z = 0, raw_theta = 0, raw_phy = 0, raw_psi = 0;
    double raw_vel_x = 0, raw_vel_y = 0, raw_vel_z = 0, raw_vel_theta = 0, raw_vel_phy = 0, raw_vel_psi = 0;
    double pos_x = 0, pos_y = 0, pos_z = 0, theta = 0, phy = 0, psi = 0;
    double vel_x = 0, vel_y = 0, vel_z = 0, vel_theta = 0, vel_phy = 0, vel_psi = 0;

    double raw_acc_x = 0, raw_acc_y = 0, raw_acc_z = 0;
    double acc_x = 0, acc_y = 0, acc_z = 0;


    //time refined
    if(time_sync_count_ > 0)
      {
        time_sync_count_ --;
        if(time_sync_count_ < TIME_SYNC_CALIB_COUNT)
          {
            long offset_tmp = ros::Time::now().toNSec() - estimator_->getSystemTimeStamp().toNSec();
            sec_offset_  = sec_offset_ +  (offset_tmp / 1000000000);
            n_sec_offset_ = n_sec_offset_ + (offset_tmp % 1000000000);
            //ROS_INFO("mocap time is %ld, imu time is %ld, sec_offset is %ld, n_sec_offset is %ld", ros::Time::now().toNSec(), estimator_->getSystemTimeStamp().toNSec(), offset_tmp, n_sec_offset_);
          }
        if(time_sync_count_ == 0)
          {
            sec_offset_ /= TIME_SYNC_CALIB_COUNT;
            n_sec_offset_ /= TIME_SYNC_CALIB_COUNT;
            offset_ = sec_offset_ * 1000000000 + n_sec_offset_;

            ROS_INFO("offset is %ld.%ld", sec_offset_, n_sec_offset_);

          }
      }


    if(time_sync_count_ == 0)
      {
        if(!first_flag)
          {
            raw_pos_x = msg->position.x - pos_x_offset_;
            raw_vel_x = (msg->position.x - prev_pos_x) / (ros::Time::now().toSec() - previous_time.toSec());
            raw_acc_x = (raw_vel_x - prev_vel_x) / (ros::Time::now().toSec() - previous_time.toSec());
            raw_pos_y = msg->position.y - posY_offset;
            raw_vel_y = (msg->position.y - prev_pos_y) / (ros::Time::now().toSec() - previous_time.toSec());
            raw_acc_y = (raw_vel_y - prev_vel_y) / (ros::Time::now().toSec() - previous_time.toSec());
            raw_pos_z = msg->position.z - posZ_offset;
            raw_vel_z = (msg->position.z - prev_pos_z) / (ros::Time::now().toSec() - previous_time.toSec());
            raw_acc_z = (raw_vel_z - prev_vel_z) / (ros::Time::now().toSec() - previous_time.toSec());

            tf::Quaternion q(msg->orientation.x,
                             msg->orientation.y,
                             msg->orientation.z,
                             msg->orientation.w);
            tf::Matrix3x3(q).getRPY(raw_phy, raw_theta, raw_psi);
            raw_vel_psi = (raw_psi - prev_psi) / (ros::Time::now().toSec() - previous_time.toSec());

            filterX_.filterFunction(raw_pos_x, pos_x, raw_vel_x, vel_x);
            filterY_.filterFunction(raw_pos_y, pos_y, raw_vel_y, vel_y);
            filterZ_.filterFunction(raw_pos_z, pos_z, raw_vel_z, vel_z);
            filterPsi_.filterFunction(raw_psi, psi, raw_vel_psi, vel_psi);

            filterAccX_.filterFunction(raw_acc_x, acc_x);
            filterAccY_.filterFunction(raw_acc_y, acc_y);
            filterAccZ_.filterFunction(raw_acc_z, acc_z);

            //correct way
            estimator_->setStatePosX(pos_x);
            estimator_->setStatePosY(pos_y);
            estimator_->setStatePosZ(pos_z);
            estimator_->setStatePsiCog(psi + cog_offset_angle_);
            estimator_->setStatePsiBoard(psi);
            ROS_INFO("angle from map to cog is %f", psi + cog_rotate_offset_);
            estimator_->setStateVelX(vel_x);
            estimator_->setStateVelY(vel_y);
            estimator_->setStateVelZ(vel_z);
            estimator_->setStateVelPsi(vel_psi);

            //publish for deubg, can delete
            aeiral_robot_base::States ground_truth_pose;
            ground_truth_pose.header.stamp.fromNSec(ros::Time::now().toNSec()-offset);

            aerial_robot_base::State x_state;
            x_state.id = "x"
            x_state.raw_pos = raw_pos_x;
            x_state.raw_vel = raw_vel_x;
            x_state.pos = pos_x;
            x_state.vel = vel_x;
            x_state.reserves.push_back(acc_x);
            x_state.reserves.push_back(raw_acc_x);

            aerial_robot_base::State y_state;
            y_state.id = "y"
            y_state.raw_pos = raw_pos_y;
            y_state.raw_vel = raw_vel_y;
            y_state.pos = pos_y;
            y_state.vel = vel_y;
            y_state.reserves.push_back(acc_y);
            y_state.reserves.push_back(raw_acc_y);

            aerial_robot_base::State z_state;
            z_state.id = "z"
            z_state.raw_pos = raw_pos_z;
            z_state.raw_vel = raw_vel_z;
            z_state.pos = pos_z;
            z_state.vel = vel_z;
            z_state.reserves.push_back(acc_z);
            z_state.reserves.push_back(raw_acc_z);


            aerial_robot_base::State yaw_state;
            yaw_state.id = "yaw"
            yaw_state.raw_pos = raw_psi;
            yaw_state.raw_vel = raw_vel_psi;
            yaw_state.pos = psi;
            yaw_state.vel = vel_psi;

            aerial_robot_base::State pitch_state;
            pitch_state.id = "pitch"
            pitch_state.raw_pos = raw_pitch;

            aerial_robot_base::State roll_state;
            roll_state.id = "roll"
            roll_state.raw_pos = raw_roll;
            
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
            prev_pos_x = msg->position.x;
            prev_pos_y = msg->position.y;
            prev_pos_z = msg->position.z;

            posX_offset = msg->position.x;
            posY_offset = msg->position.y;
            posZ_offset = msg->position.z;
            first_flag = false;
          }

        prev_pos_x = msg->position.x;
        prev_pos_y = msg->position.y;
        prev_pos_z = msg->position.z;
        prev_vel_x = raw_vel_x;
        prev_vel_y = raw_vel_y;
        prev_vel_z = raw_vel_z;

        prev_psi = raw_psi;
        previous_time = ros::Time::now();


      }
  }

  void cog_offsetCallback(std_msgs::Float32 offset_msg)
  {
    cog_offset_angle_ =  offset_msg.data;
  }


  void rosParamInit(  ros::NodeHandle nh)
  {
    std::string ns = nh.getNamespace();
    nhp_.param("retime_flag", retime_flag_, true);
    nhp_.param("pub_name", pub_name_, std::string("ground_truth/pose"));
    nhp_.param("cog_rotate_sub_name", cog_rotate_sub_name_, std::string("cog_rotate"));

    nhp_.param("rx_freq", rx_freq_, 100.0);
    nhp_.param("cutoff_pos_freq", cutoff_pos_freq_, 50.0);
    nhp_.param("cutoff_vel_freq", cutoff_vel_freq_, 50.0);

    nh.param ("cog_offset", cog_offset_, false);

    printf("%s: COG offset is %s\n", ns.c_str(), cog_offset_ ? ("true") : ("false"));
    cog_offset_angle_ = 0;

  }


};

#endif













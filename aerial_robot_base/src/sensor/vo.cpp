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

/* ros */
#include <ros/ros.h>

/* base class */
#include <aerial_robot_base/sensor_base_plugin.h>

/* kalman filters */
#include <kalman_filter/kf_pos_vel_acc_plugin.h>

/* ros msg */
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <spinal/ServoControlCmd.h>
#include <std_msgs/Empty.h>

namespace
{
  ros::Time init_servo_st;
}

namespace sensor_plugin
{
  class VisualOdometry :public sensor_plugin::SensorBase
  {
  public:

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
    {
      SensorBase::initialize(nh, nhp, estimator, sensor_name);
      rosParamInit();

      /* ros publisher: aerial_robot_base::State */
      vo_state_pub_ = nh_.advertise<aerial_robot_msgs::States>("data",10);

      /* ros subscriber: vo */
      string vo_sub_topic_name;
      nhp_.param("vo_sub_topic_name", vo_sub_topic_name, string("vo") );
      vo_sub_ = nh_.subscribe(vo_sub_topic_name, 1, &VisualOdometry::voCallback, this);


      /* servo control timer */
      if(variable_sensor_tf_flag_)
        {
          init_servo_st = ros::Time::now();
          /* ros publisher: servo motor */
          vo_servo_pub_ = nh_.advertise<spinal::ServoControlCmd>("/vo_servo_target_pwm",10); //angle -> pwm
          vo_servo_debug_sub_ = nh_.subscribe("/vo_servo_debug", 1, &VisualOdometry::servoDebugCallback, this);

          servo_control_timer_ = nhp_.createTimer(ros::Duration(servo_control_rate_), &VisualOdometry::servoControl,this); // 10 Hz
        }
    }

    ~VisualOdometry(){}
    VisualOdometry():init_time_(true), servo_auto_change_flag_(false), servo_control_rate_(0.1)
    {
      world_offset_tf_.setIdentity();
      baselink_tf_.setIdentity();

      vo_state_.states.resize(3);
      vo_state_.states[0].id = "x";
      vo_state_.states[0].state.resize(2);
      vo_state_.states[1].id = "y";
      vo_state_.states[1].state.resize(2);
      vo_state_.states[2].id = "z";
      vo_state_.states[2].state.resize(2);
    }

  private:
    /* ros */
    ros::Subscriber vo_sub_;
    ros::Publisher vo_state_pub_;
    ros::Publisher vo_servo_pub_;
    ros::Subscriber vo_servo_debug_sub_;
    ros::Timer  servo_control_timer_;

    /* ros param */
    double vo_noise_sigma_;
    bool valid_yaw_;
    bool vio_flag_;
    bool debug_verbose_;

    /* servo */
    bool servo_auto_change_flag_;
    double servo_height_thresh_;
    double servo_angle_;
    double servo_init_angle_, servo_downwards_angle_;
    double servo_vel_;
    double servo_control_rate_;
    int servo_min_pwm_, servo_max_pwm_;
    double servo_min_angle_, servo_max_angle_;
    tf::TransformBroadcaster br_;

    bool init_time_;
    tf::Transform world_offset_tf_;
    tf::Transform baselink_tf_;
    tf::StampedTransform servo_tf_; //for servo
    aerial_robot_msgs::States vo_state_;

    void voCallback(const nav_msgs::Odometry::ConstPtr & vo_msg)
    {
      /* only do egmotion estimate mode */
      if(!getFuserActivate(BasicEstimator::EGOMOTION_ESTIMATE))
        {
          ROS_WARN_THROTTLE(1,"Visual Odometry: no egmotion estimate mode");
          return;
        }

      tf::Transform raw_sensor_tf;
      tf::poseMsgToTF(vo_msg->pose.pose, raw_sensor_tf);

      if(init_time_)
        {
          init_time_ = false;

          if(!updateBaseLink2SensorTransform())
            {
              init_time_ = true;
              return;
            }

          servo_tf_.stamp_ = vo_msg->header.stamp; //reset the time stamp, very important for rosbag play

          /* set the init offset from world to the baselink of UAV from egomotion estimation (e.g. yaw) */
          world_offset_tf_.setRotation(tf::createQuaternionFromYaw(estimator_->getState(State::YAW_BASE, BasicEstimator::EGOMOTION_ESTIMATE)[0]));
          /* set the init offset from world to the baselink of UAV if we know the ground truth */
          if(estimator_->getStateStatus(State::YAW_BASE, BasicEstimator::GROUND_TRUTH))
            {
              world_offset_tf_.setOrigin(estimator_->getPos(Frame::BASELINK, BasicEstimator::GROUND_TRUTH));
              world_offset_tf_.setRotation(tf::createQuaternionFromYaw(estimator_->getState(State::YAW_BASE, BasicEstimator::GROUND_TRUTH)[0]));

              double y, p, r; world_offset_tf_.getBasis().getRPY(r, p, y);
            }

          /* also consider the offset tf from baselink to sensor */
          world_offset_tf_ *= sensor_tf_;

          ROS_INFO("VO: start kalman filter");
          tf::Vector3 init_pos = (world_offset_tf_ * raw_sensor_tf * sensor_tf_.inverse()).getOrigin();

          for(auto& fuser : estimator_->getFuser(BasicEstimator::EGOMOTION_ESTIMATE))
            {
              string plugin_name = fuser.first;
              boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
              int id = kf->getId();

              if(plugin_name == "kalman_filter/kf_pos_vel_acc_bias" ||
                 plugin_name == "kalman_filter/kf_pos_vel_acc")
                {
                  if(id < (1 << State::ROLL_COG))
                    {
                      if(time_sync_) kf->setTimeSync(true);
                      kf->setInitState(init_pos[id >> (State::X_BASE + 1)], 0);
                      kf->setMeasureFlag();
                    }
                  //if(id & (1 << State::X_BASE)) kf->setDebugVerbose(true);
                }

              if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
                {
                  if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                    {
                      if(time_sync_) kf->setTimeSync(true);
                      VectorXd init_state(6);
                      init_state << init_pos[0], 0, init_pos[1], 0, 0, 0;
                      kf->setInitState(init_state);
                      kf->setMeasureFlag();
                    }
                }
            }

          estimator_->setStateStatus(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);
          estimator_->setStateStatus(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);
          estimator_->setStateStatus(State::Z_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);
          estimator_->setStateStatus(State::YAW_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);
          return;
        }

      /* transformaton from baselink to vo sensor, if we use the servo motor */
      updateBaseLink2SensorTransform();
      baselink_tf_ = world_offset_tf_ * raw_sensor_tf * sensor_tf_.inverse();

      tf::Vector3 raw_pos;
      tf::pointMsgToTF(vo_msg->pose.pose.position, raw_pos);
      tf::Quaternion raw_q;
      tf::quaternionMsgToTF(vo_msg->pose.pose.orientation, raw_q);

      if(debug_verbose_)
        {
          double y, p, r;  tf::Matrix3x3(raw_q).getRPY(r, p, y);
          ROS_INFO("vo raw pos: [%f, %f, %f], raw rot: [%f, %f, %f]",
                   raw_pos.x(), raw_pos.y(), raw_pos.z(), r, p, y);
          tf::Vector3 mocap_pos = estimator_->getPos(Frame::BASELINK, BasicEstimator::GROUND_TRUTH);
          ROS_INFO("mocap pos: [%f, %f, %f], vo pos: [%f, %f, %f]",
                   mocap_pos.x(), mocap_pos.y(), mocap_pos.z(),
                   baselink_tf_.getOrigin().x(), baselink_tf_.getOrigin().y(),
                   baselink_tf_.getOrigin().z());
          baselink_tf_.getBasis().getRPY(r, p, y);
          ROS_INFO("mocap yaw: %f, vo rot: [%f, %f, %f]", estimator_->getState(State::YAW_BASE, BasicEstimator::GROUND_TRUTH)[0], r, p, y);
        }
      estimateProcess(vo_msg->header.stamp);

      /* publish */
      vo_state_.header.stamp = vo_msg->header.stamp;

      for(int axis = 0; axis < 3; axis++)
        vo_state_.states[axis].state[0].x = baselink_tf_.getOrigin()[axis]; //raw
      vo_state_pub_.publish(vo_state_);

      /* update */
      updateHealthStamp(vo_msg->header.stamp.toSec());
    }

    void estimateProcess(ros::Time stamp)
    {
      /* YAW */
      /* TODO: the weighting filter with IMU */
      tfScalar r,p,y;
      baselink_tf_.getBasis().getRPY(r,p,y);
      estimator_->setState(State::YAW_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 0, y);

      ros::Time stamp_temp = stamp;

      /* XYZ */
      for(auto& fuser : estimator_->getFuser(BasicEstimator::EGOMOTION_ESTIMATE))
        {
          string plugin_name = fuser.first;
          boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
          int id = kf->getId();

          if(!kf->getFilteringFlag()) continue;

          /* x_w, y_w, z_w */
          if(id < (1 << State::ROLL_COG))
            {
              if(plugin_name == "kalman_filter/kf_pos_vel_acc" ||
                 plugin_name == "kalman_filter/kf_pos_vel_acc_bias")
                {
                  /* set noise sigma */
                  VectorXd measure_sigma(1);
                  measure_sigma << vo_noise_sigma_;
                  kf->setMeasureSigma(measure_sigma);

                  /* correction */
                  int index = id >> (State::X_BASE + 1);
                  VectorXd meas(1); meas <<  baselink_tf_.getOrigin()[index];
                  vector<double> params = {kf_plugin::POS};
                  if(time_sync_ && delay_ < 0) stamp.fromSec(stamp_temp.toSec() + delay_);

                  kf->correction(meas, stamp.toSec(), params);
                  VectorXd state = kf->getEstimateState();
                  estimator_->setState(index + 3, BasicEstimator::EGOMOTION_ESTIMATE, 0, state(0));
                  estimator_->setState(index + 3, BasicEstimator::EGOMOTION_ESTIMATE, 1, state(1));

                  vo_state_.states[index].state[1].x = state(0); //estimated pos
                  vo_state_.states[index].state[1].y = state(1); //estimated vel
                }
            }
          if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
            {
              if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                {
                  /* set noise sigma */
                  VectorXd measure_sigma(2);
                  measure_sigma << vo_noise_sigma_, vo_noise_sigma_;
                  kf->setMeasureSigma(measure_sigma);

                  /* correction */
                  VectorXd meas(2); meas << baselink_tf_.getOrigin()[0], baselink_tf_.getOrigin()[1];
                  vector<double> params = {kf_plugin::POS};
                  if(time_sync_ && delay_ < 0) stamp.fromSec(stamp_temp.toSec() + delay_);

                  kf->correction(meas, stamp.toSec(), params);
                  VectorXd state = kf->getEstimateState();

                  estimator_->setState(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 0, state(0));
                  estimator_->setState(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 1, state(1));
                  estimator_->setState(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 0, state(2));
                  estimator_->setState(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 1, state(3));
                  vo_state_.states[0].state[1].x = state(0);
                  vo_state_.states[0].state[1].y = state(1);
                  vo_state_.states[0].state[1].z = state(4);

                  vo_state_.states[1].state[1].x = state(2);
                  vo_state_.states[1].state[1].y = state(3);
                  vo_state_.states[1].state[1].z = state(5);
                }
            }
        }
    }

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();

      nhp_.param("vio_flag", vio_flag_, true );
      if(param_verbose_) cout << ns << ": vio flag is " <<  vio_flag_ << endl;

      nhp_.param("valid_yaw", valid_yaw_, true );
      if(param_verbose_) cout << ns << ": valid yaw is " << valid_yaw_ << endl;

      nhp_.param("vo_noise_sigma", vo_noise_sigma_, 0.01 );
      if(param_verbose_) cout << ns << ": vo noise sigma is " <<  vo_noise_sigma_ << endl;

      nhp_.param("debug_verbose", debug_verbose_, false );
      if(param_verbose_) cout << ns << ": debug verbose is " <<  debug_verbose_ << endl;

      nhp_.param("servo_auto_change_flag", servo_auto_change_flag_, false );
      if(param_verbose_) cout << ns << ": servo auto change flag is " <<  servo_auto_change_flag_ << endl;

      nhp_.param("servo_height_thresh", servo_height_thresh_, 0.7);
      if(param_verbose_) cout << ns << ": servo height thresh is " << servo_height_thresh_ << endl;

      nhp_.param("servo_vel", servo_vel_, 0.02); // rad
      if(param_verbose_) cout << ns << ": servo vel is " << servo_vel_ << endl;

      nhp_.param("servo_init_angle", servo_init_angle_, 0.0); // rad
      if(param_verbose_) cout << ns << ": servo init angle is " << servo_init_angle_ << endl;
      servo_angle_ = servo_init_angle_;

      nhp_.param("servo_downwards_angle", servo_downwards_angle_, 0.0); // rad
      if(param_verbose_) cout << ns << ": servo downwards angle is " << servo_downwards_angle_ << endl;

      nhp_.param("servo_min_pwm", servo_min_pwm_, 1000); // pwm [ms]
      if(param_verbose_) cout << ns << ": servo min pwm is " << servo_min_pwm_ << endl;
      nhp_.param("servo_max_pwm", servo_max_pwm_, 2000); // pwm [ms]
      if(param_verbose_) cout << ns << ": servo max pwm is " << servo_max_pwm_ << endl;

      nhp_.param("servo_min_angle", servo_min_angle_, -M_PI/2); // angle [rad]
      if(param_verbose_) cout << ns << ": servo min angle is " << servo_min_angle_ << endl;
      nhp_.param("servo_max_angle", servo_max_angle_, M_PI/2); // angle [rad]
      if(param_verbose_) cout << ns << ": servo max angle is " << servo_max_angle_ << endl;

      nhp_.param("servo_ref_frame", servo_tf_.frame_id_, std::string("servo"));
      if(param_verbose_) cout << ns << ": servo ref frame is " << servo_tf_.frame_id_ << endl;
      nhp_.param("servo_child_frame", servo_tf_.child_frame_id_, std::string("servo_horn"));
      if(param_verbose_) cout << ns << ": servo child frame is " << servo_tf_.child_frame_id_ << endl;
      servo_tf_.setIdentity();
    }

    void servoControl(const ros::TimerEvent & e)
    {
      assert(variable_sensor_tf_flag_);

      bool send_pub_ = false;

      /* after takeoff */
      if(servo_auto_change_flag_ && estimator_->getState(State::Z_BASE, BasicEstimator::EGOMOTION_ESTIMATE)[0] > servo_height_thresh_ && servo_angle_ != servo_downwards_angle_)
        {
          if(fabs(servo_angle_ - servo_downwards_angle_) > servo_vel_ * servo_control_rate_)
            {
              int sign = fabs(servo_angle_ - servo_downwards_angle_) / (servo_angle_ - servo_downwards_angle_);
              servo_angle_ -=  sign / servo_vel_ * servo_control_rate_;
            }
          else servo_angle_ = servo_downwards_angle_;

          send_pub_ = true;
        }

      /* before ladning */
      if(estimator_->getState(State::Z_BASE, BasicEstimator::EGOMOTION_ESTIMATE)[0] < servo_height_thresh_ - 0.1 &&  servo_angle_ != servo_init_angle_)
        {
          if(fabs(servo_angle_ - servo_init_angle_) > servo_vel_ * servo_control_rate_)
            {
              int sign = fabs(servo_angle_ - servo_init_angle_) / (servo_angle_ - servo_init_angle_);
              servo_angle_ -=  sign / servo_vel_ * servo_control_rate_;
            }
          else servo_angle_ = servo_init_angle_;

          send_pub_ = true;
        }

      int servo_target_pwm = (servo_angle_ - servo_min_angle_) / (servo_max_angle_ - servo_min_angle_) * (servo_max_pwm_ - servo_min_pwm_) + servo_min_pwm_;

      /* init */
      if ((ros::Time::now() - init_servo_st).toSec() < 1.0 ) // 1 [sec]
        send_pub_ = true;

      spinal::ServoControlCmd servo_target_pwm_msg;
      servo_target_pwm_msg.index.push_back(0);
      servo_target_pwm_msg.angles.push_back(servo_target_pwm);
      if(send_pub_) vo_servo_pub_.publish(servo_target_pwm_msg);

      /* tf publisher */
      servo_tf_.stamp_ += (e.current_real - e.last_real);
      servo_tf_.setRotation(tf::createQuaternionFromYaw(servo_angle_)); // z axis
      br_.sendTransform(servo_tf_);
    }

    void servoDebugCallback(const std_msgs::Empty::ConstPtr & vo_msg)
    {
      servo_auto_change_flag_ = true;
    }
  };

};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::VisualOdometry, sensor_plugin::SensorBase);




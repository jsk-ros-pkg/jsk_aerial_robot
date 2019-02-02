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

/* base class */
#include <aerial_robot_base/sensor/vo.h>

namespace
{
  ros::Time init_servo_st;
  double max_du = 0;

  tf::Transform prev_sensor_tf;
}

namespace sensor_plugin
{
  VisualOdometry::VisualOdometry():
    sensor_plugin::SensorBase(string("vo")),
    servo_auto_change_flag_(false),
    full_vel_mode_(false),
    z_vel_mode_(false)
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



  void VisualOdometry::initialize(ros::NodeHandle nh, ros::NodeHandle nhp, StateEstimator* estimator, string sensor_name)
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
        string topic_name;
        nhp_.param("vo_servo_topic_name", topic_name, string("/vo_servo_target_pwm"));
        vo_servo_pub_ = nh_.advertise<sensor_msgs::JointState>(topic_name, 1);

        vo_servo_debug_sub_ = nh_.subscribe("/vo_servo_debug", 1, &VisualOdometry::servoDebugCallback, this);

        servo_control_timer_ = nhp_.createTimer(ros::Duration(servo_control_rate_), &VisualOdometry::servoControl,this); // 10 Hz
      }
  }

  void VisualOdometry::voCallback(const nav_msgs::Odometry::ConstPtr & vo_msg)
  {
    /* only do egmotion estimate mode */
    if(!getFuserActivate(StateEstimator::EGOMOTION_ESTIMATE))
      {
        ROS_WARN_THROTTLE(1,"Visual Odometry: no egmotion estimate mode");
        return;
      }

    /* update the sensor tf w.r.t baselink */
    if(!updateBaseLink2SensorTransform()) return;

    /* servo init condition */
    if(variable_sensor_tf_flag_ && ros::Time::now().toSec() - init_servo_st.toSec() < 1.0)
      return;

    /* check whether is force att control mode */
    if(estimator_->getForceAttControlFlag() && getStatus() != Status::INVALID)
      {
        estimator_->setStateStatus(State::YAW_BASE, StateEstimator::EGOMOTION_ESTIMATE, false);
        setStatus(Status::INVALID);
      }

    tf::Transform raw_sensor_tf;
    tf::poseMsgToTF(vo_msg->pose.pose, raw_sensor_tf); // motion update
    curr_timestamp_ = vo_msg->header.stamp.toSec() + delay_; //temporal update

    if(getStatus() == Status::INACTIVE)
      {
        /* for z */
        if(estimator_->getAltHandler() != nullptr && estimator_->getAltHandler()->getStatus() != Status::ACTIVE)
          {
            ROS_WARN_THROTTLE(1, "vo: the altimeter is not initialized, wait");
            z_vel_mode_ = true;
            return;
          }

        /* to get the correction rotation and omega of baselink with the consideration of time delay, along with the yaw problem */
        if(estimator_->getImuHandler()->getStatus() != Status::ACTIVE)
          {
            ROS_WARN_THROTTLE(1, "vo: the imu is not initialized, wait");
            return;
          }

        setStatus(Status::INIT);

        std::cout << "VO: start kalman filter";
        /* chose pos / vel estimation mode, according to the view of the camera */
        /* TODO: should consider the illustration or feature dense of the image view */
        if(fabs((sensor_tf_.getBasis() * tf::Vector3(0,0,1)).x()) > 0.1 ||
           fabs((sensor_tf_.getBasis() * tf::Vector3(0,0,1)).y()) > 0.1)
          {
            full_vel_mode_ = true;
            std::cout << ", full vel mode for state estimation ";

            if(estimator_->getGpsHandler() != nullptr && estimator_->getGpsHandler()->getStatus() == Status::ACTIVE)
              {
                std::cout << ", no temporal delay in outdoor mode, ";
                delay_ = 0;
                curr_timestamp_ =  vo_msg->header.stamp.toSec();
              }
          }
        else
          {
            full_vel_mode_ = false;
            std::cout << ", odometry pos mode for state estimation ";
          }

        /* step1: set the init offset from world to the baselink of UAV from egomotion estimation (e.g. yaw) */
        /** ^{w}H_{b} **/
        world_offset_tf_.setRotation(tf::createQuaternionFromYaw(estimator_->getState(State::YAW_BASE, StateEstimator::EGOMOTION_ESTIMATE)[0]));

        tf::Vector3 world_offset_pos = estimator_->getPos(Frame::BASELINK, StateEstimator::EGOMOTION_ESTIMATE);
        if(estimator_->getStateStatus(State::X_BASE, StateEstimator::EGOMOTION_ESTIMATE))
          world_offset_tf_.getOrigin().setX(world_offset_pos.x());
        if(estimator_->getStateStatus(State::Y_BASE, StateEstimator::EGOMOTION_ESTIMATE))
          world_offset_tf_.getOrigin().setY(world_offset_pos.y());
        if(estimator_->getStateStatus(State::Z_BASE, StateEstimator::EGOMOTION_ESTIMATE))
          world_offset_tf_.getOrigin().setZ(world_offset_pos.z());

        /* set the init offset from world to the baselink of UAV if we know the ground truth */
        if(estimator_->getStateStatus(State::YAW_BASE, StateEstimator::GROUND_TRUTH))
          {
            world_offset_tf_.setOrigin(estimator_->getPos(Frame::BASELINK, StateEstimator::GROUND_TRUTH));
            world_offset_tf_.setRotation(tf::createQuaternionFromYaw(estimator_->getState(State::YAW_BASE, StateEstimator::GROUND_TRUTH)[0]));

            double y, p, r; world_offset_tf_.getBasis().getRPY(r, p, y);
          }

        /* step2: also consider the offset tf from baselink to sensor */
        /** ^{w}H_{b} * ^{b}H_{vo} * ^{vo}H_{w_vo} = ^{w}H_{w_vo} **/
        world_offset_tf_ *= (sensor_tf_ * raw_sensor_tf.inverse());

        //double y, p, r; raw_sensor_tf.getBasis().getRPY(r, p, y);
        tf::Vector3 init_pos = (world_offset_tf_ * raw_sensor_tf * sensor_tf_.inverse()).getOrigin();

        for(auto& fuser : estimator_->getFuser(StateEstimator::EGOMOTION_ESTIMATE))
          {
            string plugin_name = fuser.first;
            boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
            int id = kf->getId();

            if(plugin_name == "kalman_filter/kf_pos_vel_acc")
              {
                if(id < (1 << State::ROLL_COG))
                  {
                    if(estimator_->getStateStatus(State::X_BASE + (id >> (State::X_BASE + 1)), StateEstimator::EGOMOTION_ESTIMATE))
                      continue;

                    if(!full_vel_mode_)
                      {
                        if(id & (1 << State::Z_BASE))
                          {
                            if(!z_vel_mode_)
                              {
                                kf->setInitState(init_pos[2], 0);
                                std::cout << ", init state z with pos mode";
                              }
                            else
                              std::cout << ", init state z with vel mode";
                          }
                        else
                          {
                            kf->setInitState(init_pos[id >> (State::X_BASE + 1)], 0);
                            std::cout << ", init state " << ((id >> (State::X_BASE + 1) == 0)?std::string("x"):std::string("y")) << "  with pos mode";
                          }
                      }
                    kf->setMeasureFlag();
                  }
              }

            if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
              {
                if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                  {
                    if(estimator_->getStateStatus(State::X_BASE, StateEstimator::EGOMOTION_ESTIMATE) && estimator_->getStateStatus(State::Y_BASE, StateEstimator::EGOMOTION_ESTIMATE))
                      continue;

                    if(!full_vel_mode_)
                      {
                        VectorXd init_state(6);
                        init_state << init_pos[0], 0, init_pos[1], 0, 0, 0;
                        kf->setInitState(init_state);
                        std::cout << ", init state x/y with pos mode";
                      }
                    else
                      std::cout << ", init state x/y with vel mode";

                    kf->setMeasureFlag();
                  }
              }
          }
        std::cout << std::endl;

        estimator_->setStateStatus(State::X_BASE, StateEstimator::EGOMOTION_ESTIMATE, true);
        estimator_->setStateStatus(State::Y_BASE, StateEstimator::EGOMOTION_ESTIMATE, true);
        estimator_->setStateStatus(State::Z_BASE, StateEstimator::EGOMOTION_ESTIMATE, true);

        prev_sensor_tf = raw_sensor_tf;
        prev_timestamp_ = curr_timestamp_;
        setStatus(Status::ACTIVE);
        return;
      }

    /* transformaton from baselink to vo sensor, if we use the servo motor */

    baselink_tf_ = world_offset_tf_ * raw_sensor_tf * sensor_tf_.inverse();

    tf::Vector3 raw_pos;
    tf::pointMsgToTF(vo_msg->pose.pose.position, raw_pos);
    tf::Quaternion raw_q;
    tf::quaternionMsgToTF(vo_msg->pose.pose.orientation, raw_q);

    // velocity:
    tf::Transform delta_tf = prev_sensor_tf.inverse() * raw_sensor_tf;
    tf::Vector3 raw_local_vel = delta_tf.getOrigin() / (curr_timestamp_ - prev_timestamp_);

    tf::Matrix3x3 r;
    tf::Vector3 omega;
    int mode = estimator_->getStateStatus(State::YAW_BASE, StateEstimator::GROUND_TRUTH)?StateEstimator::GROUND_TRUTH:StateEstimator::EGOMOTION_ESTIMATE;
    estimator_->findRotOmege((curr_timestamp_ + prev_timestamp_) / 2, mode, r, omega);

    raw_global_vel_ = r * ( sensor_tf_.getBasis() * raw_local_vel - omega.cross(sensor_tf_.getOrigin()));

    if(debug_verbose_)
      {
        double y, p, r;  tf::Matrix3x3(raw_q).getRPY(r, p, y);
        ROS_INFO("vo raw pos: [%f, %f, %f], raw rot: [%f, %f, %f]",
                 raw_pos.x(), raw_pos.y(), raw_pos.z(), r, p, y);
        tf::Vector3 mocap_pos = estimator_->getPos(Frame::BASELINK, StateEstimator::GROUND_TRUTH);
        ROS_INFO("mocap pos: [%f, %f, %f], vo pos: [%f, %f, %f]",
                 mocap_pos.x(), mocap_pos.y(), mocap_pos.z(),
                 baselink_tf_.getOrigin().x(), baselink_tf_.getOrigin().y(),
                 baselink_tf_.getOrigin().z());
        baselink_tf_.getBasis().getRPY(r, p, y);
        ROS_INFO("mocap yaw: %f, vo rot: [%f, %f, %f]", estimator_->getState(State::YAW_BASE, StateEstimator::GROUND_TRUTH)[0], r, p, y);
      }

    double start_time = ros::Time::now().toSec();
    estimateProcess();
    /*
    double time_du = ros::Time::now().toSec() - start_time;
    if(max_du < time_du) max_du = time_du;
    ROS_INFO("max du: %f, time du: %f", max_du, time_du);
    */

    /* publish */
    vo_state_.header.stamp.fromSec(curr_timestamp_);
    for(int axis = 0; axis < 3; axis++)
      vo_state_.states[axis].state[0].x = baselink_tf_.getOrigin()[axis]; //raw
    vo_state_.states[0].state[0].y = raw_global_vel_.x();
    vo_state_.states[1].state[0].y = raw_global_vel_.y();
    vo_state_.states[2].state[0].y = raw_global_vel_.z();

    vo_state_pub_.publish(vo_state_);

    /* update */
    prev_sensor_tf = raw_sensor_tf;
    prev_timestamp_ =  curr_timestamp_; // vo_msg->header.stamp;

    updateHealthStamp();
  }

  void VisualOdometry::estimateProcess()
  {
    if(getStatus() == Status::INVALID) return;

    if(full_vel_mode_)
      {
        double height = estimator_->getState(State::Z_BASE, StateEstimator::EGOMOTION_ESTIMATE)[0];
        if(height < downwards_vo_min_height_ || height > downwards_vo_max_height_)
          {
            //ROS_WARN_THROTTLE(1, "VO, the height %f is not valid for vo to do downards vo", height);
            return;
          }
      }
    else
      {
        /* YAW */
        if(!estimator_->getStateStatus(State::YAW_BASE, StateEstimator::EGOMOTION_ESTIMATE))
          {
            estimator_->setStateStatus(State::YAW_BASE, StateEstimator::EGOMOTION_ESTIMATE, true);
            ROS_WARN("VO, set yaw estimate status true");
          }
        tfScalar r,p,y;
        baselink_tf_.getBasis().getRPY(r,p,y);
        estimator_->setState(State::YAW_BASE, StateEstimator::EGOMOTION_ESTIMATE, 0, y);
      }

    /* XYZ */
    for(auto& fuser : estimator_->getFuser(StateEstimator::EGOMOTION_ESTIMATE))
      {
        string plugin_name = fuser.first;
        boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;

        if(!kf->getFilteringFlag()) continue;

        int id = kf->getId();
        double timestamp = curr_timestamp_;
        double outlier_thresh = full_vel_mode_?(vel_outlier_thresh_ / (vel_noise_sigma_) / (vel_noise_sigma_)):0;
        /* x_w, y_w, z_w */
        if(id < (1 << State::ROLL_COG))
          {
            if(plugin_name == "kalman_filter/kf_pos_vel_acc")
              {
                /* correction */
                VectorXd measure_sigma(1);
                if((id & (1 << State::X_BASE)) || (id & (1 << State::Y_BASE)))
                  {
                    if(full_vel_mode_) measure_sigma << vel_noise_sigma_;
                    else measure_sigma << level_pos_noise_sigma_;
                  }
                else
                  {
                    if(z_vel_mode_ || full_vel_mode_)
                      measure_sigma << vel_noise_sigma_;
                    else
                      measure_sigma << z_pos_noise_sigma_;
                  }

                int index = id >> (State::X_BASE + 1);
                VectorXd meas(1);
                vector<double> params;
                if((id & (1 << State::X_BASE)) || (id & (1 << State::Y_BASE)))
                  {
                    if(full_vel_mode_)
                      {
                        meas << raw_global_vel_[index];
                        params = {kf_plugin::VEL};
                        timestamp = (curr_timestamp_ + prev_timestamp_) / 2; //velocity timestamp
                      }
                    else
                      {
                        meas << baselink_tf_.getOrigin()[index];
                        params = {kf_plugin::POS};
                      }
                  }
                else
                  {
                    if(z_vel_mode_ || full_vel_mode_)
                      {
                        meas << raw_global_vel_[index];
                        params = {kf_plugin::VEL};
                        timestamp = (curr_timestamp_ + prev_timestamp_) / 2; //velocity timestamp
                      }
                    else
                      {
                        meas << baselink_tf_.getOrigin()[index];
                        params = {kf_plugin::POS};
                      }

                    if(z_no_delay_) timestamp -= delay_;
                  }

                kf->correction(meas, measure_sigma,
                               time_sync_?(timestamp):-1, params, outlier_thresh);
                // VectorXd state = kf->getEstimateState();
                // estimator_->setState(index + 3, StateEstimator::EGOMOTION_ESTIMATE, 0, state(0));
                // estimator_->setState(index + 3, StateEstimator::EGOMOTION_ESTIMATE, 1, state(1));
              }
          }
        if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
          {
            if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
              {
                /* correction */
                VectorXd measure_sigma(2);
                measure_sigma << level_pos_noise_sigma_, level_pos_noise_sigma_;

                VectorXd meas(2);
                vector<double> params;
                if(full_vel_mode_)
                  {
                    meas << raw_global_vel_[0], raw_global_vel_[1];
                    params = {kf_plugin::VEL};
                  }
                else
                  {
                    meas << baselink_tf_.getOrigin()[0], baselink_tf_.getOrigin()[1];
                    params = {kf_plugin::POS};
                  }

                kf->correction(meas, measure_sigma, time_sync_?(timestamp):-1, params);
                // VectorXd state = kf->getEstimateState();

                // estimator_->setState(State::X_BASE, StateEstimator::EGOMOTION_ESTIMATE, 0, state(0));
                // estimator_->setState(State::X_BASE, StateEstimator::EGOMOTION_ESTIMATE, 1, state(1));
                // estimator_->setState(State::Y_BASE, StateEstimator::EGOMOTION_ESTIMATE, 0, state(2));
                // estimator_->setState(State::Y_BASE, StateEstimator::EGOMOTION_ESTIMATE, 1, state(3));
              }
          }
      }
  }

  void VisualOdometry::rosParamInit()
  {
    std::string ns = nhp_.getNamespace();

    if(nhp_.hasParam("full_vel_mode"))
      {
        nhp_.getParam("full_vel_mode", full_vel_mode_);
        if(param_verbose_) cout << ns << ": full vel mode is " <<  full_vel_mode_ << endl;
      }
    if(nhp_.hasParam("z_vel_mode"))
      {
        nhp_.getParam("z_vel_mode", z_vel_mode_);
        if(param_verbose_) cout << ns << ": z vel mode is " <<  z_vel_mode_ << endl;
      }

    nhp_.param("z_no_delay", z_no_delay_, true );
    if(param_verbose_) cout << ns << ": z no delay is " <<  z_no_delay_ << endl;

    nhp_.param("level_pos_noise_sigma", level_pos_noise_sigma_, 0.01 );
    if(param_verbose_) cout << ns << ": level_pos noise sigma is " <<  level_pos_noise_sigma_ << endl;
    nhp_.param("z_pos_noise_sigma", z_pos_noise_sigma_, 0.01 );
    if(param_verbose_) cout << ns << ": z_pos noise sigma is " <<  z_pos_noise_sigma_ << endl;

    nhp_.param("vel_noise_sigma", vel_noise_sigma_, 0.05 );
    if(param_verbose_) cout << ns << ": vel noise sigma is " <<  vel_noise_sigma_ << endl;

    nhp_.param("vel_outlier_thresh", vel_outlier_thresh_, 1.0);
    if(param_verbose_) cout << ns << ": vel outlier thresh is " <<  vel_outlier_thresh_ << endl;

    nhp_.param("downwards_vo_min_height", downwards_vo_min_height_, 0.8);
    if(param_verbose_) cout << ns << ": downwards vo min height is " << downwards_vo_min_height_ << endl;

    nhp_.param("downwards_vo_max_height", downwards_vo_max_height_, 10.0);
    if(param_verbose_) cout << ns << ": downwards vo max height is " << downwards_vo_max_height_ << endl;

    nhp_.param("debug_verbose", debug_verbose_, false );
    if(param_verbose_) cout << ns << ": debug verbose is " <<  debug_verbose_ << endl;

    nhp_.param("joint", joint_name_, std::string("servo"));
    if(param_verbose_) cout << ns << ": servo joint name is " << joint_name_ << endl;

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

    nhp_.param("servo_min_angle", servo_min_angle_, -M_PI/2); // angle [rad]
    if(param_verbose_) cout << ns << ": servo min angle is " << servo_min_angle_ << endl;
    nhp_.param("servo_max_angle", servo_max_angle_, M_PI/2); // angle [rad]
    if(param_verbose_) cout << ns << ": servo max angle is " << servo_max_angle_ << endl;

    nhp_.param("servo_control_rate", servo_control_rate_, 0.1);
    if(param_verbose_) cout << ns << ": servo control rate is " << servo_control_rate_ << endl;
  }

  void VisualOdometry::servoControl(const ros::TimerEvent & e)
  {
    assert(variable_sensor_tf_flag_);

    bool send_pub_ = false;

    /* after takeoff */
    if(servo_auto_change_flag_ && estimator_->getState(State::Z_BASE, StateEstimator::EGOMOTION_ESTIMATE)[0] > servo_height_thresh_ && servo_angle_ != servo_downwards_angle_)
      {
        if(fabs(servo_angle_ - servo_downwards_angle_) > servo_vel_ * servo_control_rate_)
          {
            int sign = fabs(servo_angle_ - servo_downwards_angle_) / (servo_angle_ - servo_downwards_angle_);
            servo_angle_ -=  sign * servo_vel_ * servo_control_rate_;
          }
        else servo_angle_ = servo_downwards_angle_;

        send_pub_ = true;
      }

    /* before landing */
    if(estimator_->getState(State::Z_BASE, StateEstimator::EGOMOTION_ESTIMATE)[0] < servo_height_thresh_ - 0.1 &&  servo_angle_ != servo_init_angle_)
      {
        if(fabs(servo_angle_ - servo_init_angle_) > servo_vel_ * servo_control_rate_)
          {
            int sign = fabs(servo_angle_ - servo_init_angle_) / (servo_angle_ - servo_init_angle_);
            servo_angle_ -=  sign * servo_vel_ * servo_control_rate_;
          }
        else servo_angle_ = servo_init_angle_;

        send_pub_ = true;
      }

    /* init */
    if (ros::Time::now().toSec() - init_servo_st.toSec() < 1.0) // 1 [sec]
      send_pub_ = true;

    if(send_pub_)
      {
        sensor_msgs::JointState msg;
        msg.name.push_back(joint_name_);
        msg.position.push_back(servo_angle_);
        vo_servo_pub_.publish(msg);
      }
  }
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::VisualOdometry, sensor_plugin::SensorBase);




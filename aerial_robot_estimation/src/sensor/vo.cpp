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
#include <aerial_robot_estimation/sensor/vo.h>

namespace
{
  double init_servo_st = 0;
  double max_du = 0;

  tf::Transform prev_sensor_tf;
  tf::Vector3 baselink_omega;
  tf::Matrix3x3 baselink_r;
}

namespace sensor_plugin
{
  VisualOdometry::VisualOdometry():
    sensor_plugin::SensorBase(),
    servo_auto_change_flag_(false)
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

  void VisualOdometry::initialize(ros::NodeHandle nh,
                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                  string sensor_name, int index)
  {
    SensorBase::initialize(nh, robot_model, estimator, sensor_name, index);
    rosParamInit();

    /* ros subscriber: vo */
    string topic_name;
    getParam<std::string>("vo_sub_topic_name", topic_name, string("vo"));

    uint32_t queuse_size = 1; // if the timestamp is not synchronized, we can only use the latest sensor value.
    if(time_sync_) queuse_size = 10;
    vo_sub_ = nh_.subscribe(topic_name, queuse_size, &VisualOdometry::voCallback, this);


    /* servo control timer */
    if(variable_sensor_tf_flag_)
      {
        /* ros publisher: servo motor */
        getParam<std::string>("vo_servo_topic_name", topic_name, string("vo_servo_target_pwm"));
        vo_servo_pub_ = nh_.advertise<sensor_msgs::JointState>(topic_name, 1);

        getParam<std::string>("vo_servo_debug_topic_name", topic_name, string("vo_servo_debug"));
        vo_servo_debug_sub_ = nh_.subscribe(topic_name, 1, &VisualOdometry::servoDebugCallback, this);

        servo_control_timer_ = indexed_nhp_.createTimer(ros::Duration(servo_control_rate_), &VisualOdometry::servoControl,this); // 10 Hz
      }

    prev_timestamp_ = 0;
  }

  void VisualOdometry::voCallback(const nav_msgs::Odometry::ConstPtr & vo_msg)
  {
    /* only do egmotion estimate mode */
    if(!getFuserActivate(aerial_robot_estimation::EGOMOTION_ESTIMATE))
      {
        ROS_WARN_THROTTLE(1,"Visual Odometry: no egmotion estimate mode");
        return;
      }

    /* update the sensor tf w.r.t baselink */
    if(!updateBaseLink2SensorTransform()) return;

    /* servo init condition */
    if(variable_sensor_tf_flag_ && ros::Time::now().toSec() - init_servo_st < 1.0 && init_servo_st > 0)
      return;

    /* check whether is force att control mode */
    if(estimator_->getForceAttControlFlag() && getStatus() == Status::ACTIVE)
      {
        if(estimator_->getStateStatus(State::YAW_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))
          estimator_->setStateStatus(State::YAW_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, false);
        setStatus(Status::INVALID);
      }

    /* check the sensor value whether valid */
    if(std::isnan(vo_msg->pose.pose.position.x) ||
       std::isnan(vo_msg->pose.pose.position.y) ||
       std::isnan(vo_msg->pose.pose.position.z) ||
       std::isnan(vo_msg->twist.twist.linear.x) ||
       std::isnan(vo_msg->twist.twist.linear.y) ||
       std::isnan(vo_msg->twist.twist.linear.z))
      {
        ROS_ERROR("VIO sensor [%s] publishes NaN value!", vo_sub_.getTopic().c_str());
        reset();
        return;
      }

    tf::Transform raw_sensor_tf;
    tf::poseMsgToTF(vo_msg->pose.pose, raw_sensor_tf); // motion update

    /* temporal update */
    curr_timestamp_ = vo_msg->header.stamp.toSec() + delay_;
    reference_timestamp_ = curr_timestamp_;

    /* throttle message */
    if(throttle_rate_ > 0)
      {
        if (curr_timestamp_ - prev_timestamp_ < 1 / throttle_rate_)
          {
            return;
          }
      }

    if(getStatus() == Status::INACTIVE)
      {
        /* for z */
        bool alt_initialized = false;
        for(const auto& handler: estimator_->getAltHandlers())
          {
            if(handler->getStatus() == Status::ACTIVE)
              {
                alt_initialized = true;
                break;
              }
          }

        if(!alt_initialized && estimator_->getAltHandlers().size() > 0)
          {
            ROS_WARN_THROTTLE(1, "vo: no altimeter is initialized, wait");
            z_vel_mode_ = true;
            return;
          }

        /* to get the correction rotation and omega of baselink with the consideration of time delay, along with the yaw problem */
        bool imu_initialized = false;
        for(const auto& handler: estimator_->getImuHandlers())
          {
            if(handler->getStatus() == Status::ACTIVE)
              {
                imu_initialized = true;
                break;
              }
          }

        if(!imu_initialized)
          {
            ROS_WARN_THROTTLE(1, "vo: no imu is initialized, wait");
            return;
          }

        auto sensor_view_rot = estimator_->getOrientation(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE) * sensor_tf_.getBasis();
        if(vio_mode_)
          {
            /* get the true rotation (i.e. attitude) from the sensor in vio mode */
            tf::Quaternion q;
            tf::quaternionMsgToTF(vo_msg->pose.pose.orientation, q);
            sensor_view_rot.setRotation(q);
          }

        /* can not start fusion from this sensor if the sensor is downward and the height is too low */
        double downward_rate = (sensor_view_rot * tf::Vector3(1,0,0)).z();
        if(downward_rate < -0.8 &&
           estimator_->getState(State::Z_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE)[0] < downwards_vo_min_height_)
          {
            return;
          }

        setStatus(Status::INIT);

        ROS_INFO_STREAM(indexed_nhp_.getNamespace()  << ": start kalman filter");
        /* chose pos / vel estimation mode, according to the view of the camera */
        /* TODO: should consider the illustration or feature dense of the image view */

        if(downward_rate < -0.8)
          {
            fusion_mode_ = ONLY_VEL_MODE;
            std::cout << ", only vel mode from downward view state estimation";

            bool gps_initialized = false;
            for(const auto& handler: estimator_->getGpsHandlers())
              {
                if(handler->getStatus() == Status::ACTIVE)
                  {
                    gps_initialized = true;
                    break;
                  }
              }

            if(gps_initialized)
              {
                getParam<double>("outdoor_vel_noise_sigma", vel_noise_sigma_, vel_noise_sigma_);
                std::cout << ", the vel noise sigma is: " << vel_noise_sigma_ << ", ";

                /* heuristic option */
                if(outdoor_no_vel_time_sync_)
                  {
                    /* TODO: very heuristic, but effective. we observe the outdoor env is tricky for stereo cam stamp identification (e.g. zed mini) */
                    time_sync_ = false;
                    delay_ = 0;
                    std::cout << ", no temporal delay in outdoor mode, ";
                  }
              }
          }

        /** step1: ^{w}H_{b'}, b': level frame of b **/
        tf::Transform w_bdash_f(tf::createQuaternionFromYaw(estimator_->getState(State::YAW_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE)[0]));

        tf::Vector3 baselink_pos = estimator_->getPos(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE);
        if(estimator_->getStateStatus(State::X_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))
          w_bdash_f.getOrigin().setX(baselink_pos.x());
        if(estimator_->getStateStatus(State::Y_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))
          w_bdash_f.getOrigin().setY(baselink_pos.y());
        if(estimator_->getStateStatus(State::Z_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))
          w_bdash_f.getOrigin().setZ(baselink_pos.z());

        /* set the offset if we know the ground truth */
        if(estimator_->getStateStatus(State::YAW_BASE, aerial_robot_estimation::GROUND_TRUTH))
          {
            w_bdash_f.setOrigin(estimator_->getPos(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH));
            w_bdash_f.setRotation(tf::createQuaternionFromYaw(estimator_->getState(State::YAW_BASE, aerial_robot_estimation::GROUND_TRUTH)[0]));
          }

        /** step2: ^{vo}H_{b'} **/
        tf::Transform vo_bdash_f = raw_sensor_tf * sensor_tf_.inverse(); // ^{vo}H_{b}
        double r,p,y;
        vo_bdash_f.getBasis().getRPY(r,p,y);
        vo_bdash_f.setRotation(tf::createQuaternionFromYaw(y)); // ^{vo}H_{b'}

        /** step3: ^{w}H_{vo} = ^{w}H_{b'} * ^{b'}H_{vo} **/
        world_offset_tf_ = w_bdash_f * vo_bdash_f.inverse();

        /* publish the offset tf if necessary */
        geometry_msgs::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = vo_msg->header.stamp;
        static_transformStamped.header.frame_id = "world";
        static_transformStamped.child_frame_id = vo_msg->header.frame_id;
        tf::transformTFToMsg(world_offset_tf_, static_transformStamped.transform);
        static_broadcaster_.sendTransform(static_transformStamped);

        tf::Vector3 init_pos = w_bdash_f.getOrigin();


        for(auto& fuser : estimator_->getFuser(aerial_robot_estimation::EGOMOTION_ESTIMATE))
          {
            string plugin_name = fuser.first;
            boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
            int id = kf->getId();

            if(plugin_name == "kalman_filter/kf_pos_vel_acc")
              {
                if(id < (1 << State::ROLL_COG))
                  {
                    /* not need to initialize */
                    if(estimator_->getStateStatus(State::X_BASE + (id >> (State::X_BASE + 1)), aerial_robot_estimation::EGOMOTION_ESTIMATE))
                      continue;

                    if(fusion_mode_ != ONLY_VEL_MODE) //debug
                      {
                        if(id & (1 << State::Z_BASE))
                          {
                            if(!z_vel_mode_)
                              {
                                kf->setInitState(init_pos[2], 0);
                                std::cout << ", init state z with " << ((fusion_mode_== ONLY_POS_MODE)?"ony_pos":"pos_vel") << " mode";
                              }
                            else
                              std::cout << ", init state z with vel mode";
                          }
                        else
                          {
                            kf->setInitState(init_pos[id >> (State::X_BASE + 1)], 0);
                            std::cout << ", init state " << ((id >> (State::X_BASE + 1) == 0)?"x":"y") << "  with "
                                      << ((fusion_mode_== ONLY_POS_MODE)?"only_pos":"pos_vel") << " mode";
                          }
                      }
                    kf->setMeasureFlag();
                  }
              }

            if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
              {
                if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                  {
                    if(estimator_->getStateStatus(State::X_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE) && estimator_->getStateStatus(State::Y_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))
                      continue;

                    if(fusion_mode_ != ONLY_VEL_MODE)
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

        estimator_->setStateStatus(State::X_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);
        estimator_->setStateStatus(State::Y_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);
        estimator_->setStateStatus(State::Z_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);

        prev_sensor_tf = raw_sensor_tf;
        prev_timestamp_ = curr_timestamp_;
        setStatus(Status::ACTIVE);
        return;
      }

    /* RESET   */
    if(getStatus() == Status::RESET && ros::Time::now().toSec() - reset_stamp_ > reset_duration_)
    {
      prev_sensor_tf = raw_sensor_tf;
      setStatus(Status::ACTIVE);
    }

    /* transformaton from baselink to vo sensor, if we use the servo motor */

    baselink_tf_ = world_offset_tf_ * raw_sensor_tf * sensor_tf_.inverse();

    tf::Vector3 raw_pos;
    tf::pointMsgToTF(vo_msg->pose.pose.position, raw_pos);
    tf::Quaternion raw_q;
    tf::quaternionMsgToTF(vo_msg->pose.pose.orientation, raw_q);

    // velocity:
    tf::Vector3 raw_vel;
    tf::vector3MsgToTF(vo_msg->twist.twist.linear, raw_vel);
    /* get the latest orientation and omega */
    baselink_r = estimator_->getOrientation(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE);
    baselink_omega = estimator_->getAngularVel(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE);

    if (time_sync_)
      {
        // TODO: what is the following previous tricky code?
        // int mode = estimator_->getStateStatus(State::YAW_BASE, aerial_robot_estimation::GROUND_TRUTH)?aerial_robot_estimation::GROUND_TRUTH:aerial_robot_estimation::EGOMOTION_ESTIMATE;
        int mode = aerial_robot_estimation::EGOMOTION_ESTIMATE;

        if (raw_vel == tf::Vector3(0.0,0.0,0.0))
          {
            /* the odometry message does not contain velocity information, we have to calulcate by ourselves. */
            tf::Transform delta_tf = prev_sensor_tf.inverse() * raw_sensor_tf;
            raw_vel = delta_tf.getOrigin() / (curr_timestamp_ - prev_timestamp_);

            reference_timestamp_ = (curr_timestamp_ + prev_timestamp_) / 2;
            estimator_->findRotOmega(reference_timestamp_, mode, baselink_r, baselink_omega);
          }
        else
          {
            if(!estimator_->findRotOmega(reference_timestamp_, mode, baselink_r, baselink_omega, false))
              {
                //ROS_INFO("raw msg timestamp: %f", vo_msg->header.stamp.toSec());
                reference_timestamp_ = estimator_->getImuLatestTimeStamp(); //special process for realsense t265
              }
          }
      }

    raw_global_vel_ = world_offset_tf_.getBasis() * raw_vel;
    if (local_vel_mode_)
      {
        // if the velocity is described in local frame (i.e., the sensor frame),
        // we need to convert to global one
        raw_global_vel_ = baselink_r * sensor_tf_.getBasis() * raw_vel;
      }
    // consider the offset between baselink and sensor frames
    raw_global_vel_ -= baselink_r * baselink_omega.cross(sensor_tf_.getOrigin());


    if(debug_verbose_)
      {
        double y, p, r;  tf::Matrix3x3(raw_q).getRPY(r, p, y);
        ROS_INFO("vo raw pos: [%f, %f, %f], raw rot: [%f, %f, %f]",
                 raw_pos.x(), raw_pos.y(), raw_pos.z(), r, p, y);
        tf::Vector3 mocap_pos = estimator_->getPos(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH);
        ROS_INFO("mocap pos: [%f, %f, %f], vo pos: [%f, %f, %f]",
                 mocap_pos.x(), mocap_pos.y(), mocap_pos.z(),
                 baselink_tf_.getOrigin().x(), baselink_tf_.getOrigin().y(),
                 baselink_tf_.getOrigin().z());
        baselink_tf_.getBasis().getRPY(r, p, y);
        ROS_INFO("mocap yaw: %f, vo rot: [%f, %f, %f]", estimator_->getState(State::YAW_BASE, aerial_robot_estimation::GROUND_TRUTH)[0], r, p, y);
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

    state_pub_.publish(vo_state_);

    /* update */
    prev_sensor_tf = raw_sensor_tf;
    //ROS_INFO("vo time diff: %f", curr_timestamp_ - prev_timestamp_);
    prev_timestamp_ =  curr_timestamp_; // vo_msg->header.stamp;

    updateHealthStamp();
  }

  void VisualOdometry::estimateProcess()
  {
    if(getStatus() == Status::INVALID || getStatus() == Status::RESET) return;

    /* downward check */
    tf::Matrix3x3 sensor_view_rot;
    if(vio_mode_)
      sensor_view_rot = baselink_tf_.getBasis() * sensor_tf_.getBasis();
    else sensor_view_rot = baselink_r * sensor_tf_.getBasis();

    if((sensor_view_rot * tf::Vector3(1,0,0)).z() < -0.8)
      {
        double height = estimator_->getState(State::Z_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE)[0];
        if(height < downwards_vo_min_height_ || height > downwards_vo_max_height_)
          {

            //ROS_WARN_THROTTLE(1, "%s, the height %f is not valid for vo to do downards vo", indexed_nhp_.getNamespace().c_str(), height);
            return;
          }
      }
    else
      {
        /* YAW */
        if(!estimator_->getStateStatus(State::YAW_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))
          {
            estimator_->setStateStatus(State::YAW_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);
            ROS_WARN_STREAM(indexed_nhp_.getNamespace() <<": set yaw estimate status true");
          }
        tfScalar r,p,y;
        baselink_tf_.getBasis().getRPY(r,p,y);
        estimator_->setState(State::YAW_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, 0, y);
      }

    /* XYZ */
    for(auto& fuser : estimator_->getFuser(aerial_robot_estimation::EGOMOTION_ESTIMATE))
      {
        string plugin_name = fuser.first;
        boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;

        if(!kf->getFilteringFlag()) continue;

        int id = kf->getId();
        double timestamp = reference_timestamp_;
        double outlier_thresh = (fusion_mode_ == ONLY_VEL_MODE)?(vel_outlier_thresh_ / (vel_noise_sigma_) / (vel_noise_sigma_)):0;
        /* x_w, y_w, z_w */
        if(id < (1 << State::ROLL_COG))
          {
            if(plugin_name == "kalman_filter/kf_pos_vel_acc")
              {
                int index = id >> (State::X_BASE + 1);
                vector<double> params;

                /* correction */
                if (fusion_mode_ == ONLY_POS_MODE)
                  {
                    VectorXd meas(1);
                    VectorXd measure_sigma(1);
                    measure_sigma << level_pos_noise_sigma_;
                    meas << baselink_tf_.getOrigin()[index];
                    params = {kf_plugin::POS};

                    if(id & (1 << State::Z_BASE))
                      {
                        if(z_vel_mode_)
                          {
                            measure_sigma << vel_noise_sigma_;
                            meas << raw_global_vel_[index];
                            params = {kf_plugin::VEL};
                          }
                        else
                          measure_sigma << z_pos_noise_sigma_;

                        if(z_no_delay_) timestamp -= delay_;
                      }

                    kf->correction(meas, measure_sigma,
                                   time_sync_?(timestamp):-1,
                                   params, outlier_thresh);
                  }
                else if(fusion_mode_ == ONLY_VEL_MODE)
                  {
                    VectorXd measure_sigma(1);
                    measure_sigma << vel_noise_sigma_;

                    VectorXd meas(1);
                    meas << raw_global_vel_[index];
                    params = {kf_plugin::VEL};

                    kf->correction(meas, measure_sigma,
                                   time_sync_?(timestamp):-1,
                                   params, outlier_thresh);
                  }
                else if(fusion_mode_ == POS_VEL_MODE)
                  {
                    if(id & (1 << State::Z_BASE))
                      {
                        if(z_no_delay_) timestamp -= delay_;

                        if(z_vel_mode_)
                          {
                            VectorXd measure_sigma(1);
                            measure_sigma << vel_noise_sigma_;
                            VectorXd meas(1);
                            meas << raw_global_vel_[index];
                            params = {kf_plugin::VEL};
                            kf->correction(meas, measure_sigma,
                                           time_sync_?(timestamp):-1,
                                           params, outlier_thresh);
                          }
                        else
                          {
                            VectorXd measure_sigma(2);
                            measure_sigma << z_pos_noise_sigma_, vel_noise_sigma_;
                            VectorXd meas(2);
                            meas << baselink_tf_.getOrigin()[index], raw_global_vel_[index];
                            params = {kf_plugin::POS_VEL};
                            kf->correction(meas, measure_sigma,
                                           time_sync_?(timestamp):-1,
                                           params, outlier_thresh);
                          }
                      }
                    else
                      {
                        VectorXd measure_sigma(2);
                        measure_sigma << level_pos_noise_sigma_, vel_noise_sigma_;
                        VectorXd meas(2);
                        meas << baselink_tf_.getOrigin()[index], raw_global_vel_[index];
                        params = {kf_plugin::POS_VEL};
                        kf->correction(meas, measure_sigma,
                                       time_sync_?(timestamp):-1,
                                       params, outlier_thresh);
                      }
                }
              else
                {
                  ROS_WARN("wrong fusion model %d", fusion_mode_);
                }
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
                if(fusion_mode_ == ONLY_VEL_MODE)
                  {
                    meas << raw_global_vel_[0], raw_global_vel_[1];
                    params = {kf_plugin::VEL};
                  }
                else if(fusion_mode_ == ONLY_POS_MODE)
                  {
                    meas << baselink_tf_.getOrigin()[0], baselink_tf_.getOrigin()[1];
                    params = {kf_plugin::POS};
                  }
                else
                  {
                    ROS_WARN("POS_VEL_MODE for xy_roll_pitch_bias is not supported");
                  }

                kf->correction(meas, measure_sigma, time_sync_?(timestamp):-1, params);
              }
          }
      }
  }

  void VisualOdometry::rosParamInit()
  {
    getParam<int>("fusion_mode", fusion_mode_, (int)ONLY_POS_MODE);
    getParam<bool>("vio_mode", vio_mode_, false);
    getParam<bool>("local_vel_mode", local_vel_mode_, true);
    getParam<bool>("z_vel_mode", z_vel_mode_, false);
    getParam<bool>("z_no_delay", z_no_delay_, false);
    getParam<bool>("outdoor_no_vel_time_sync", outdoor_no_vel_time_sync_, false);
    getParam<double>("throttle_rate", throttle_rate_, 0.0);
    getParam<double>("level_pos_noise_sigma", level_pos_noise_sigma_, 0.01 );
    getParam<double>("z_pos_noise_sigma", z_pos_noise_sigma_, 0.01 );
    getParam<double>("vel_noise_sigma", vel_noise_sigma_, 0.05 );
    getParam<double>("vel_outlier_thresh", vel_outlier_thresh_, 1.0);
    getParam<double>("downwards_vo_min_height", downwards_vo_min_height_, 0.8);
    getParam<double>("downwards_vo_max_height", downwards_vo_max_height_, 10.0);

    getParam<std::string>("joint", joint_name_, std::string("servo"));
    getParam<bool>("servo_auto_change_flag", servo_auto_change_flag_, false );
    getParam<double>("servo_height_thresh", servo_height_thresh_, 0.7);
    getParam<double>("servo_vel", servo_vel_, 0.02); // rad
    getParam<double>("servo_init_angle", servo_init_angle_, 0.0); // rad
    getParam<double>("servo_downwards_angle", servo_downwards_angle_, 0.0); // rad
    getParam<double>("servo_min_angle", servo_min_angle_, -M_PI/2); // angle [rad]
    getParam<double>("servo_max_angle", servo_max_angle_, M_PI/2); // angle [rad]
    getParam<double>("servo_control_rate", servo_control_rate_, 0.1);

    servo_angle_ = servo_init_angle_;
  }

  void VisualOdometry::servoControl(const ros::TimerEvent & e)
  {
    assert(variable_sensor_tf_flag_);

    bool send_pub_ = false;

    /* after takeoff */
    if(servo_auto_change_flag_ && estimator_->getState(State::Z_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE)[0] > servo_height_thresh_ && servo_angle_ != servo_downwards_angle_)
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
    if(estimator_->getState(State::Z_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE)[0] < servo_height_thresh_ - 0.1 &&  servo_angle_ != servo_init_angle_)
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
    if(init_servo_st == 0) init_servo_st = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - init_servo_st < 1.0) // 1 [sec]
      {
        send_pub_ = true;
      }

    if(send_pub_)
      {
        sensor_msgs::JointState msg;
        msg.name.push_back(joint_name_);
        msg.position.push_back(servo_angle_);
        vo_servo_pub_.publish(msg);
      }
  }

  bool VisualOdometry::reset()
  {
    /* call reset rosserive */
    std::string srv_name;
    getParam<std::string>("reset_srv_name", srv_name, string("reset"));

    ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>(srv_name);

    if(!client.exists ())
      {
        ROS_WARN("rosservice %s does not exist", srv_name.c_str());
        setStatus(Status::INVALID);
        return false;
      }

    /* waiting for the completement of reset is allowed because of the multi-thread spin */
    std_srvs::Empty srv;
    if (client.call(srv))
      {
        fusion_mode_ = ONLY_VEL_MODE;
        setStatus(Status::RESET);
        reset_stamp_ = ros::Time::now().toSec();
        ROS_INFO("Reset VIO sensor %s", sensor_name_.c_str());
        return true;
      }
    else
      {
        ROS_ERROR("Failed to call service %s", srv_name.c_str());
        setStatus(Status::INVALID);
        return false;
      }
  }

};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::VisualOdometry, sensor_plugin::SensorBase);




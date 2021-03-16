#include <hydrus/torsion_estimator.h>

TorsionEstimator::TorsionEstimator (ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  tfListener_(tfBuffer_)
{
  nh_private_.param("debug", debug_, false);
  nh_private_.param("simulation", is_simulation_, false);

  // parameters for system
  nh_private_.param("links", link_num_, 6);
  neuron_imu_data_.resize(link_num_);
  link_transforms_.resize(link_num_);
  R_ci_cache_.resize(link_num_-1);
  prev_torsions_.resize(link_num_-1);
  torsion_vel_in_prev1_.resize(link_num_-1);
  torsion_vel_in_prev2_.resize(link_num_-1);
  torsion_vel_out_prev1_.resize(link_num_-1);
  torsion_vel_out_prev2_.resize(link_num_-1);
  torsion_in_prev1_.resize(link_num_-1);
  torsion_in_prev2_.resize(link_num_-1);
  torsion_out_prev1_.resize(link_num_-1);
  torsion_out_prev2_.resize(link_num_-1);

  nh_private_.param<std::string>("robot_name", robot_name_, "hydrus");
  nh_private_.param("use_mocap", is_use_mocap_, true);
  nh_private_.param("torsion_vel_cutoff_freq", torsion_vel_cutoff_freq_, 10.0);
  nh_private_.param("torsion_vel_q", torsion_vel_q_, 1.0);
  nh_private_.param("torsion_cutoff_freq", torsion_cutoff_freq_, 10.0);
  nh_private_.param("torsion_q", torsion_q_, 1.0);

  // ros subscribers
  if (!is_use_mocap_) {
    nh_private_.param<std::string>("neuron_imu_frame_name_prefix",neuron_imu_frame_name_prefix_, "neuron");
    nh_private_.param<std::string>("neuron_imu_frame_name_suffix",neuron_imu_frame_name_suffix_, "_imu");
  } else {
    nh_private_.param<std::string>("mocap_frame_name_prefix",neuron_imu_frame_name_prefix_, "mocap/link");
    nh_private_.param<std::string>("mocap_frame_name_suffix",neuron_imu_frame_name_suffix_, "/pose");
  }
  for (int i = 0; i < link_num_; ++i) {
    if (!is_use_mocap_) {
      neuron_imu_subs_.push_back(nh_.subscribe<sensor_msgs::Imu>(
          neuron_imu_frame_name_prefix_+std::to_string(i+1)+neuron_imu_frame_name_suffix_,
          1, boost::bind(&TorsionEstimator::neuronIMUCallback, this, _1, i)));
    } else {
      if (is_simulation_) {
        neuron_imu_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
            neuron_imu_frame_name_prefix_+std::to_string(i+1)+neuron_imu_frame_name_suffix_,
            1, boost::bind(&TorsionEstimator::simMocapCallback, this, _1, i)));
      } else {
        neuron_imu_subs_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(
            neuron_imu_frame_name_prefix_+std::to_string(i+1)+neuron_imu_frame_name_suffix_,
            1, boost::bind(&TorsionEstimator::mocapCallback, this, _1, i)));
      }
    }
    neuron_imu_data_[i].orientation.x = 0;
    neuron_imu_data_[i].orientation.y = 0;
    neuron_imu_data_[i].orientation.z = 0;
    neuron_imu_data_[i].orientation.w = 1;
  }

  // ros publisher
  estimate_state_pub_ = nh_private_.advertise<sensor_msgs::JointState>("torsion_estimate", 5);

  // ros timer
  nh_private_.param("kf_step_rate", kf_step_rate_, 30.0);
  kf_timer_ = nh.createTimer(ros::Duration(1/kf_step_rate_), boost::bind(&TorsionEstimator::kfStepCallback, this, _1));
}

void TorsionEstimator::kfStepCallback(const ros::TimerEvent& e)
{
  std::vector<double> y_measure(link_num_-1);

  for (int i = 0; i < link_num_-1; ++i) {
    try{
      geometry_msgs::TransformStamped link_transform;
      link_transform = tfBuffer_.lookupTransform(robot_name_+"/link"+std::to_string(i+2), robot_name_+"/link"+std::to_string(i+1), ros::Time(0));
      link_transforms_[i] = link_transform;
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s",ex.what());
      return;
    }
  }

  for (int i = 0; i < link_num_-1; ++i) {
    R_ci_cache_[i] = TorsionEstimator::quat_to_rot_mat(neuron_imu_data_[i].orientation);
  }

  for (int i = 0; i < link_num_-1; ++i) {
    tf2::Quaternion q_shape, q_org_observed, q_next_observed, q_torsion;
    tf2::convert(link_transforms_[i].transform.rotation, q_shape);
    tf2::convert(neuron_imu_data_[i].orientation, q_org_observed);
    tf2::convert(neuron_imu_data_[i+1].orientation, q_next_observed);

    q_torsion = q_org_observed.inverse() * q_next_observed * q_shape.inverse();

    double roll, pitch, yaw;
    tf2::getEulerYPR(q_torsion, yaw, pitch, roll);

    // Low Pass BiQuad filter
    // filter parameters
    double omega = 2.0 * 3.14159265 *  torsion_cutoff_freq_ / kf_step_rate_;
    double alpha = sin(omega) / (2.0 * torsion_q_);
    double a0 =  1.0 + alpha;
    double a1 = -2.0 * cos(omega);
    double a2 =  1.0 - alpha;
    double b0 = (1.0 - cos(omega)) / 2.0;
    double b1 =  1.0 - cos(omega);
    double b2 = (1.0 - cos(omega)) / 2.0;

    double torsion_in = -roll;
    double torsion = b0/a0 * torsion_in + b1/a0 * torsion_in_prev1_[i]  + b2/a0 * torsion_in_prev2_[i]
      - a1/a0 * torsion_out_prev1_[i] - a2/a0 * torsion_out_prev2_[i];

    torsion_in_prev2_[i] = torsion_in_prev1_[i];
    torsion_in_prev1_[i] = torsion_in;
    torsion_out_prev2_[i] = torsion_out_prev1_[i];
    torsion_out_prev1_[i] = torsion;

    y_measure[i] = torsion;
  }

  Eigen::VectorXd torsion_vel = Eigen::VectorXd::Zero(link_num_-1);
  // Low Pass BiQuad filter
  // filter parameters
  double omega = 2.0 * 3.14159265 *  torsion_vel_cutoff_freq_ / kf_step_rate_;
  double alpha = sin(omega) / (2.0 * torsion_vel_q_);

  double a0 =  1.0 + alpha;
  double a1 = -2.0 * cos(omega);
  double a2 =  1.0 - alpha;
  double b0 = (1.0 - cos(omega)) / 2.0;
  double b1 =  1.0 - cos(omega);
  double b2 = (1.0 - cos(omega)) / 2.0;

  for (int i = 0; i < link_num_-1; ++i) {
    Eigen::VectorXd torsion_vel_in = Eigen::VectorXd::Zero(link_num_-1);
    torsion_vel_in(i) = (y_measure[i]-prev_torsions_[i])*kf_step_rate_;

    torsion_vel(i) = b0/a0 * torsion_vel_in(i) + b1/a0 * torsion_vel_in_prev1_[i]  + b2/a0 * torsion_vel_in_prev2_[i]
                      - a1/a0 * torsion_vel_out_prev1_[i] - a2/a0 * torsion_vel_out_prev2_[i];

    torsion_vel_in_prev2_[i] = torsion_vel_in_prev1_[i];
    torsion_vel_in_prev1_[i] = torsion_vel_in(i);

    torsion_vel_out_prev2_[i] = torsion_vel_out_prev1_[i];
    torsion_vel_out_prev1_[i] = torsion_vel(i);

    prev_torsions_[i] = y_measure[i];
  }

  // publish estimate result
  sensor_msgs::JointState torsion_estimate;
  for(int i=0; i<link_num_-1; i++) {
    torsion_estimate.name.push_back("torsion"+std::to_string(i+1));
    torsion_estimate.position.push_back( y_measure[i] );
    torsion_estimate.velocity.push_back( torsion_vel(i) );
    torsion_estimate.effort.push_back(0);
  }
  estimate_state_pub_.publish(torsion_estimate);
}

void TorsionEstimator::neuronIMUCallback(const sensor_msgs::ImuConstPtr& msg, const int link_id) {
  neuron_imu_data_[link_id].angular_velocity = msg->angular_velocity;
  neuron_imu_data_[link_id].linear_acceleration = msg->linear_acceleration;
  neuron_imu_data_[link_id].orientation = msg->orientation;
}

void TorsionEstimator::simMocapCallback(const nav_msgs::OdometryConstPtr& msg, const int link_id) {
  mocapFilter(link_id, msg->pose.pose, msg->header);
}

void TorsionEstimator::mocapCallback(const geometry_msgs::PoseStampedConstPtr& msg, const int link_id) {
  mocapFilter(link_id, msg->pose, msg->header);
}

void TorsionEstimator::mocapFilter(const int link_id, const geometry_msgs::Pose& pose, const std_msgs::Header& header) {
  double r, p, y, r_p, p_p, y_p;
  TorsionEstimator::geometry_quat_to_rpy(r,p,y,pose.orientation);
  TorsionEstimator::geometry_quat_to_rpy(r_p,p_p,y_p,neuron_imu_data_[link_id].orientation);

  neuron_imu_data_[link_id].angular_velocity.x = (r-r_p)/(header.stamp.nsec - neuron_imu_data_[link_id].header.stamp.nsec)*1e9;
  neuron_imu_data_[link_id].angular_velocity.y = (p-p_p)/(header.stamp.nsec - neuron_imu_data_[link_id].header.stamp.nsec)*1e9;
  neuron_imu_data_[link_id].angular_velocity.z = (y-y_p)/(header.stamp.nsec - neuron_imu_data_[link_id].header.stamp.nsec)*1e9;

  neuron_imu_data_[link_id].header = header;
  neuron_imu_data_[link_id].orientation = pose.orientation;
}

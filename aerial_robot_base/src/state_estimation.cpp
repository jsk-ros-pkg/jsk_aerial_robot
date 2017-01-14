#include "aerial_robot_base/state_estimation.h"

RigidEstimator::RigidEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  BasicEstimator(nh, nh_private)
{
  rosParamInit();
}

RigidEstimator::~RigidEstimator() {}

void RigidEstimator::tfPublish()
{
  statesBroadcast();
}


void RigidEstimator::statesBroadcast()
{
  aerial_robot_base::States full_states;
  nav_msgs::Odometry states;

  //full_states.header.stamp = getSystemTimeStamp();
  full_states.header.stamp = ros::Time::now();
  states.header.stamp = ros::Time::now();
  states.header.frame_id = std::string("/nav"); //TOOD: use rosparam
  states.child_frame_id = std::string("/bae_link"); //TOOD: use rosparam

  aerial_robot_base::State x_state;
  x_state.id = "x";
  x_state.pos = getGTState(X_W, 0);
  x_state.vel = getGTState(X_W, 1);
  x_state.raw_pos = getEEState(X_W, 0);
  x_state.raw_vel = getEEState(X_W, 1);
  x_state.kf_pos = getEXState(X_W, 0);
  x_state.kf_vel = getEXState(X_W, 1);
  x_state.reserves.push_back(getEEState(X_B, 0));
  x_state.reserves.push_back(getEEState(X_B, 1));
  x_state.reserves.push_back(getEXState(X_B, 0));
  x_state.reserves.push_back(getEXState(X_B, 1));
  states.pose.pose.position.x
    = (state_mode_ == GROUND_TRUTH)?getGTState(X_W, 0):getEEState(X_W, 0);
  states.twist.twist.linear.x
    = (state_mode_ == GROUND_TRUTH)?getGTState(X_W, 1):getEEState(X_W, 1);

  aerial_robot_base::State y_state;
  y_state.id = "y";
  y_state.pos = getGTState(Y_W, 0);
  y_state.vel = getGTState(Y_W, 1);
  y_state.raw_pos = getEEState(Y_W, 0);
  y_state.raw_vel = getEEState(Y_W, 1);
  y_state.kf_pos = getEXState(Y_W, 0);
  y_state.kf_vel = getEXState(Y_W, 1);
  y_state.reserves.push_back(getEEState(Y_B, 0));
  y_state.reserves.push_back(getEEState(Y_B, 1));
  y_state.reserves.push_back(getEXState(Y_B, 0));
  y_state.reserves.push_back(getEXState(Y_B, 1));
  states.pose.pose.position.y
    = (state_mode_ == GROUND_TRUTH)?getGTState(Y_W, 0):getEEState(Y_W, 0);
  states.twist.twist.linear.y
    = (state_mode_ == GROUND_TRUTH)?getGTState(Y_W, 1):getEEState(Y_W, 1);

  aerial_robot_base::State z_state;
  z_state.id = "z";
  z_state.pos = getGTState(Z_W, 0);
  z_state.vel = getGTState(Z_W, 1);
  z_state.raw_pos = getEEState(Z_W, 0);
  z_state.raw_vel = getEEState(Z_W, 1);
  z_state.kf_pos = getEXState(Z_W, 0);
  z_state.kf_vel = getEXState(Z_W, 1);
  z_state.reserves.push_back(getEEState(Z_W, 2));
  states.pose.pose.position.z
    = (state_mode_ == GROUND_TRUTH)?getGTState(Z_W, 0):getEEState(Z_W, 0);
  states.twist.twist.linear.z
    = (state_mode_ == GROUND_TRUTH)?getGTState(Z_W, 1):getEEState(Z_W, 1);

  tf::Quaternion q;
  if(state_mode_ == GROUND_TRUTH)
    {
      q.setRPY(getGTState(ROLL_W, 0), getGTState(PITCH_W, 0), getGTState(YAW_W_B, 0));
      states.twist.twist.angular.x = getGTState(ROLL_W, 1);
      states.twist.twist.angular.y = getGTState(PITCH_W, 1);
      states.twist.twist.angular.z = getGTState(YAW_W_B, 1);
    }
  if(state_mode_ == EGOMOTION_ESTIMATE)
    {
      q.setRPY(getEEState(ROLL_W, 0), getEEState(PITCH_W, 0), getEEState(YAW_W_B, 0));
      states.twist.twist.angular.x = getEEState(ROLL_W, 1);
      states.twist.twist.angular.y = getEEState(PITCH_W, 1);
      states.twist.twist.angular.z = getEEState(YAW_W_B, 1);
    }
  states.pose.pose.orientation.x = q.x();
  states.pose.pose.orientation.y = q.y();
  states.pose.pose.orientation.z = q.z();
  states.pose.pose.orientation.w = q.w();

  aerial_robot_base::State yaw_state;
  yaw_state.id = "yaw";
  yaw_state.pos = getGTState(YAW_W_B, 0);
  yaw_state.vel = getGTState(YAW_W_B, 1);
  yaw_state.raw_pos = getEEState(YAW_W_B, 0);
  yaw_state.raw_vel = getEEState(YAW_W_B, 1);
  yaw_state.kf_pos = getEXState(YAW_W_B, 0);
  yaw_state.kf_vel = getEXState(YAW_W_B, 1);
  yaw_state.reserves.push_back(getGTState(YAW_W_COG, 0));
  yaw_state.reserves.push_back(getGTState(YAW_W_COG, 1));
  yaw_state.reserves.push_back(getEEState(YAW_W_COG, 0));
  yaw_state.reserves.push_back(getEEState(YAW_W_COG, 1));
  aerial_robot_base::State pitch_state;
  pitch_state.id = "pitch";
  pitch_state.pos = getGTState(PITCH_W, 0);
  pitch_state.raw_pos = getEEState(PITCH_W, 0);
  aerial_robot_base::State roll_state;
  roll_state.id = "roll";
  roll_state.pos = getGTState(ROLL_W, 0);
  roll_state.raw_pos = getEEState(ROLL_W, 0);

  full_states.states.push_back(x_state);
  full_states.states.push_back(y_state);
  full_states.states.push_back(z_state);
  full_states.states.push_back(roll_state);
  full_states.states.push_back(pitch_state);
  full_states.states.push_back(yaw_state);

  full_states_pub_.publish(full_states);

  state_pub_.publish(states);
}

bool RigidEstimator::pattern_match(std::string &pl, std::string &pl_candidate)
{
  int cmp = fnmatch(pl.c_str(), pl_candidate.c_str(), FNM_CASEFOLD);
  if (cmp == 0)
    return true;
  else if (cmp != FNM_NOMATCH) {
    // never see that, i think that it is fatal error.
    ROS_FATAL("Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
              pl.c_str(), pl_candidate.c_str(), cmp);
    ros::shutdown();
  }
  return false;
}


void RigidEstimator::rosParamInit()
{
  std::string ns = nhp_.getNamespace();
  nhp_.param ("state_mode", state_mode_, 1); //EGOMOTION_ESTIMATE: 1
  if(param_verbose_) cout << ns <<": state_mode is " << state_mode_ << endl;

  /* kalman filter egomotion plugin list */
  ros::V_string egomotion_list{};
  nhp_.getParam("egomotion_list", egomotion_list);

  sensor_fusion_loader_ptr_ = boost::shared_ptr< pluginlib::ClassLoader<kf_base_plugin::KalmanFilter> >(new pluginlib::ClassLoader<kf_base_plugin::KalmanFilter>("kalman_filter", "kf_base_plugin::KalmanFilter"));

  int index = 0;
  for (auto &egomotion_plugin_name : egomotion_list)
    {
      for (auto &name : sensor_fusion_loader_ptr_->getDeclaredClasses())
        {
          if(!pattern_match(egomotion_plugin_name, name)) continue;

          fuser_egomotion_plugin_name_.push_back(egomotion_plugin_name);

          std::stringstream fuser_no;
          fuser_no << index + 1;

          int fuser_egomotion_id;
          string fuser_egomotion_name;

          if (!nhp_.getParam ("fuser_egomotion_id" + fuser_no.str(), fuser_egomotion_id))
            ROS_ERROR("%d, no param in fuser egomotion id", index);
          if(param_verbose_) cout << "fuser_egomotion_id" << index + 1 << " is " << fuser_egomotion_id << endl;
          fuser_egomotion_id_.push_back(fuser_egomotion_id);

          if (!nhp_.getParam ("fuser_egomotion_name" + fuser_no.str(), fuser_egomotion_name))
            ROS_ERROR("%d, no param in fuser egomotion name", index);
          if(param_verbose_) cout << "fuser_egomotion_name" << index + 1 << " is " << fuser_egomotion_name << endl;

          fuser_egomotion_.push_back(sensor_fusion_loader_ptr_->createInstance(name));
          fuser_egomotion_[index]->initialize(nh_, fuser_egomotion_name, fuser_egomotion_id_[index]);

          index ++;
          break;
        }
    }
  assert((fuser_egomotion_plugin_name_.size() == index) && (fuser_egomotion_id_.size() == index));

  /* kalman filter egomotion plugin list */
  ros::V_string experiment_list{};
  nhp_.getParam("experiment_list", experiment_list);

  index = 0;
  for (auto &experiment_plugin_name : experiment_list)
    {
      for (auto &name : sensor_fusion_loader_ptr_->getDeclaredClasses())
        {
          if(!pattern_match(experiment_plugin_name, name)) continue;
          fuser_experiment_plugin_name_.push_back(experiment_plugin_name);

          std::stringstream fuser_no;
          fuser_no << index + 1;

          int fuser_experiment_id;
          string fuser_experiment_name;

          if (!nhp_.getParam ("fuser_experiment_id" + fuser_no.str(), fuser_experiment_id))
            ROS_ERROR("%d, no param in fuser experiment id", index);
          if(param_verbose_) cout << "fuser_experiment_id" << index + 1 << " is " << fuser_experiment_id << endl;
          fuser_experiment_id_.push_back(fuser_experiment_id);

          if (!nhp_.getParam ("fuser_experiment_name" + fuser_no.str(), fuser_experiment_name))
            ROS_ERROR("%d, no param in fuser experiment name", index);
          if(param_verbose_) cout << "fuser_experiment_name" << index + 1 << " is " << fuser_experiment_name << endl;

          fuser_experiment_.push_back(sensor_fusion_loader_ptr_->createInstance(name));
          fuser_experiment_[index]->initialize(nh_, fuser_experiment_name, fuser_experiment_id_[index]);

          index ++;
          break;
        }
    }
  assert((fuser_experiment_plugin_name_.size() == index) && (fuser_experiment_id_.size() == index));

  sensor_plugin_ptr_ =  boost::shared_ptr< pluginlib::ClassLoader<sensor_plugin::SensorBase> >( new pluginlib::ClassLoader<sensor_plugin::SensorBase>("aerial_robot_base", "sensor_plugin::SensorBase"));

  ros::V_string sensor_list{};
  nhp_.getParam("sensor_list", sensor_list);

  index = 0;
  for (auto &sensor_plugin_name : sensor_list)
    {
      for (auto &name : sensor_plugin_ptr_->getDeclaredClasses())
        {
          if(!pattern_match(sensor_plugin_name, name)) continue;

          std::stringstream sensor_no;
          sensor_no << index + 1;

          sensors_.push_back(sensor_plugin_ptr_->createInstance(name));
          index ++;
          break;
        }
    }

  assert((sensors_.size() == index) && (sensor_list.size() == index));

  /* initilaize in the same time */
  for(size_t i = 0; i < sensors_.size(); i++)
    sensors_[i]->initialize(nh_, ros::NodeHandle(""), this, sensors_, sensor_list, (int)i);

}

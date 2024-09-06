#include <delta/navigation/delta_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

void RollingNavigator::rollingPlanner()
{
  tf::Vector3 cog_euler;
  {
    tf::Quaternion cog2baselink_rot;
    tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
    tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();
    double r, p, y;
    cog_rot.getRPY(r, p, y);
    cog_euler = tf::Vector3(r, p, y);
  }

  Eigen::Matrix3d curr_target_baselink_rot;
  Eigen::Vector3d b1 = Eigen::Vector3d(1.0, 0.0, 0.0), b2 = Eigen::Vector3d(0.0, 1.0, 0.0);
  matrixTFToEigen(tf::Matrix3x3(curr_target_baselink_quat_), curr_target_baselink_rot);

  switch(current_ground_navigation_mode_)
    {
    case aerial_robot_navigation::FLYING_STATE:
      {
        break;
      }

    case aerial_robot_navigation::STANDING_STATE:
      {
        /* evaluate current point in trajectory */
        agi::Vector<> baselink_roll_trajectory = agi::Vector<>::Zero(3);
        if(ground_trajectory_mode_)
          {
            poly_.eval(ros::Time::now().toSec(), baselink_roll_trajectory);

            if(ros::Time::now().toSec() > ground_trajectory_start_time_ + ground_trajectory_duration_)
              {
                ROS_INFO_STREAM("[navigation] finished trajectory tracking");
                ground_trajectory_mode_ = false;
              }
          }

        /* set trajectory or final state */
        Eigen::Matrix3d rot_mat;
        if(ground_trajectory_mode_)
          {
            rot_mat = Eigen::AngleAxisd(baselink_roll_trajectory(0), b1);
            setTargetOmegaX(baselink_roll_trajectory(1));
            setTargetAngAccX(baselink_roll_trajectory(2));
          }
        else
          {
            rot_mat = Eigen::AngleAxisd(M_PI / 2.0, b1);
            setTargetOmegaX(0.0);
            setTargetAngAccX(0.0);
          }

        /* state transition to rolling state based on the baselink rotation */
        double baselink_roll = estimator_->getEuler(Frame::BASELINK, estimate_mode_).x();
        if(fabs(fabs(baselink_roll) - M_PI / 2.0) < standing_baselink_roll_converged_thresh_)
          {
            ROS_INFO_STREAM("[navigation] baselink roll " << baselink_roll << " is smaller than threshold " << standing_baselink_roll_converged_thresh_);
            setGroundNavigationMode(aerial_robot_navigation::ROLLING_STATE);
          }

        /* set desire coordinate */
        KDL::Rotation rot_mat_kdl = eigenToKdl(rot_mat);
        double qx, qy, qz, qw;
        rot_mat_kdl.GetQuaternion(qx, qy, qz, qw);
        setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));

        break;
      }

    case aerial_robot_navigation::ROLLING_STATE:
      {
        /* follow trajectory by the end of during trajectory duration */
        agi::Vector<> baselink_roll_trajectory = agi::Vector<>::Zero(3);
        if(ground_trajectory_mode_)
          {
            poly_.eval(ros::Time::now().toSec(), baselink_roll_trajectory);

            if(ros::Time::now().toSec() > ground_trajectory_start_time_ + ground_trajectory_duration_)
              {
                ROS_INFO_STREAM("[navigation] finished trajectory tracking");
                ground_trajectory_mode_ = false;
              }
          }

        /* calculate rolling pitch angle */
        Eigen::Matrix3d additional_rot_mat;
        if(!getPitchAngVelUpdating())
          {
            setTargetOmegaY(0);
            additional_rot_mat.setIdentity();
          }
        else
          {
            double target_pitch_ang_vel = getTargetPitchAngVel();
            setTargetOmegaY(target_pitch_ang_vel);

            if((getNaviState() != aerial_robot_navigation::TAKEOFF_STATE) && (getNaviState() != aerial_robot_navigation::HOVER_STATE))
              {
                additional_rot_mat = Eigen::AngleAxisd(loop_du_ * target_pitch_ang_vel, b2);
              }
            else
              {
                if(fabs(cog_euler.y()) < rolling_pitch_update_thresh_)
                  {
                    additional_rot_mat = Eigen::AngleAxisd(loop_du_ * target_pitch_ang_vel, b2);
                  }
                else
                  {
                    additional_rot_mat.setIdentity();
                    ROS_WARN_STREAM_THROTTLE(0.5, "[navigation] do not update target pitch because the pitch " << cog_euler.y() << " is  larger than thresh " << rolling_pitch_update_thresh_);
                  }
              }
          }
        curr_target_baselink_rot = additional_rot_mat * curr_target_baselink_rot;

        /* set target controller target and desire orientation*/
        Eigen::Matrix3d rot_mat;
        if(ground_trajectory_mode_)
          {
            /* set pid term for roll from trajectory */
            setTargetOmegaX(baselink_roll_trajectory(1));
            setTargetAngAccX(baselink_roll_trajectory(2));
            rot_mat = Eigen::AngleAxisd(baselink_roll_trajectory(0), b1);
          }
        else
          {
            /* set pid term as 0 for roll */
            setTargetOmegaX(0.0);
            setTargetAngAccX(0.0);
            rot_mat = curr_target_baselink_rot;
          }

        /* return when not need to set target baselink rotation */
        if(!ground_trajectory_mode_ && !getPitchAngVelUpdating())
          return;

        /* set desire coordinate */
        KDL::Rotation rot_mat_kdl = eigenToKdl(rot_mat);
        double qx, qy, qz, qw;
        rot_mat_kdl.GetQuaternion(qx, qy, qz, qw);
        setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));

        break;
      }

    case aerial_robot_navigation::DOWN_STATE:
      {
        tf::Matrix3x3 down_start_baselink_rot = tf::Matrix3x3(down_start_baselink_quat_);
        Eigen::Matrix3d rot_mat;
        matrixTFToEigen(down_start_baselink_rot, rot_mat);

        double down_angle = std::clamp(down_mode_roll_anglvel_ * (ros::Time::now().toSec() - down_start_time_), 0.0, M_PI / 2.0);
        rot_mat = Eigen::AngleAxisd(-down_angle, b1) * rot_mat;

        KDL::Rotation rot_mat_kdl = eigenToKdl(rot_mat);
        double qx, qy, qz, qw;
        rot_mat_kdl.GetQuaternion(qx, qy, qz, qw);
        setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        break;
      }

    default:
      break;
    }
}

void RollingNavigator::setGroundNavigationMode(int state)
{
  if(state != current_ground_navigation_mode_)
    {
      ROS_WARN_STREAM("[navigation] switch to " << indexToGroundNavigationModeString(state));
    }
  else
    {
      return;
    }

  if(state == aerial_robot_navigation::FLYING_STATE)
    {
      setTargetXyFromCurrentState();

      ros::NodeHandle navi_nh(nh_, "navigation");
      double target_z;
      getParam<double>(navi_nh, "takeoff_height", target_z, 1.0);
      setTargetPosZ(target_z);

      setTargetYawFromCurrentState();

      controllers_reset_flag_ = true;
    }

  if(state == aerial_robot_navigation::STANDING_STATE)
    {
      ground_trajectory_start_time_ = ros::Time::now().toSec();
      poly_.reset();
      poly_.scale(ground_trajectory_start_time_, ground_trajectory_duration_);
      poly_.addConstraint(ground_trajectory_start_time_, agi::Vector<3>(0.0, 0.0, 0.0));
      poly_.addConstraint(ground_trajectory_start_time_ + ground_trajectory_duration_, agi::Vector<3>(M_PI / 2.0, 0.0, 0.0));
      poly_.solve();
      ground_trajectory_mode_ = true;
    }

  if(state == aerial_robot_navigation::ROLLING_STATE)
    {
    }

  if(state == aerial_robot_navigation::DOWN_STATE)
    {
      down_start_time_ = ros::Time::now().toSec();
      down_start_baselink_quat_ = getCurrentTargetBaselinkQuat();
      ROS_INFO_STREAM("[navigation] down start at: " << down_start_time_);
    }

  current_ground_navigation_mode_ = state;
}

void RollingNavigator::locomotionJoyCallback(const sensor_msgs::JoyConstPtr & joy_msg)
{
  sensor_msgs::Joy joy_cmd = (*joy_msg);

  switch(motion_mode_)
    {
    case aerial_robot_navigation::LOCOMOTION_MODE:
      {
        /* change ground navigation state */
        /* L1 + cross_up : standing state */
        if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == 1.0 && current_ground_navigation_mode_ != aerial_robot_navigation::STANDING_STATE)
          {
            ROS_INFO_STREAM("[joy] change to " << indexToGroundNavigationModeString(aerial_robot_navigation::STANDING_STATE));
            setGroundNavigationMode(aerial_robot_navigation::STANDING_STATE);
          }

        /* L1 + cross_down : rolling state */
        if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == -1.0 && current_ground_navigation_mode_ != aerial_robot_navigation::ROLLING_STATE)
          {
            ROS_INFO_STREAM("[joy] change to " << indexToGroundNavigationModeString(aerial_robot_navigation::ROLLING_STATE));
            setGroundNavigationMode(aerial_robot_navigation::ROLLING_STATE);
          }

        /* L1 + cross_left : flying state */
        if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1.0 && current_ground_navigation_mode_ != aerial_robot_navigation::FLYING_STATE)
          {
            ROS_INFO_STREAM("[joy] change to " << indexToGroundNavigationModeString(aerial_robot_navigation::FLYING_STATE));
            setGroundNavigationMode(aerial_robot_navigation::FLYING_STATE);
          }

        /* L1 + cross_right : down state */
        if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1.0 && current_ground_navigation_mode_ != aerial_robot_navigation::DOWN_STATE)
          {
            ROS_INFO_STREAM("[joy] change to " << indexToGroundNavigationModeString(aerial_robot_navigation::DOWN_STATE));
            setGroundNavigationMode(aerial_robot_navigation::DOWN_STATE);
          }

        /* set target angular velocity around yaw based on R-stick horizontal */
        if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && fabs(joy_cmd.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS]) > joy_stick_deadzone_)
          {
            target_yaw_ang_vel_ = rolling_max_yaw_ang_vel_ * joy_cmd.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS];

            setTargetOmegaZ(target_yaw_ang_vel_);

            yaw_ang_vel_updating_ = true;
          }
        else
          {
            if(yaw_ang_vel_updating_)
              {
                target_yaw_ang_vel_ = 0.0;

                tf::Quaternion cog2baselink_rot;
                tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
                tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();
                double r, p, y;
                cog_rot.getRPY(r, p, y);

                setTargetYaw(y);
                setTargetOmegaZ(0.0);

                yaw_ang_vel_updating_ = false;
              }
          }

        /* set target angular velocity around pitch based on L-stick vertical */
        if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && fabs(joy_cmd.axes[PS4_AXIS_STICK_LEFT_UPWARDS]) > joy_stick_deadzone_)
          {
            target_pitch_ang_vel_ = rolling_max_pitch_ang_vel_ * joy_cmd.axes[PS4_AXIS_STICK_LEFT_UPWARDS];
            pitch_ang_vel_updating_ = true;
          }
        else
          {
            target_pitch_ang_vel_ = 0;
            pitch_ang_vel_updating_ = false;
          }
        break;
      }
    }
}

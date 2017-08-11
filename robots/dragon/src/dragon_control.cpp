#include <dragon/dragon_control.h>

using namespace std;

DragonController::DragonController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag):
  TransformController(nh, nh_private, callback_flag), servo_torque_(false), landing_process_(false)
{
  nh_private_.param("real_machine", real_machine_, true);
  nh_private_.param("joints_torque_control_srv_name", joints_torque_control_srv_name_, std::string("/joints_controller/torque_enable"));

  /* height threshold to disable the joint servo when landing */
  nh_private_.param("height_thresh", height_thresh_, 0.1);

  string topic_name;
  nh_private_.param("uav_odom_topic_name", topic_name, string("/uav/baselink/odom"));
  uav_odom_sub_ = nh_.subscribe(topic_name, 1,  &DragonController::uavOdomCallback, this);
  flight_command_sub_ = nh_.subscribe("/flight_config_cmd", 1, &DragonController::flightCommandCallback, this);

}

void DragonController::control()
{
  ros::Rate loop_rate(control_rate_);
  static int i = 0;
  static int cnt = 0;

  if(!realtime_control_flag_) return;
  while(ros::ok())
    {
      if(landing_process_ )
        {
          if(uav_pos_.z() < height_thresh_ && real_machine_)
            {

              ros::ServiceClient client = nh_.serviceClient<dynamixel_controllers::TorqueEnable>(joints_torque_control_srv_name_);
              dynamixel_controllers::TorqueEnable srv;

              srv.request.torque_enable = false;

              if (client.call(srv))
                {
                  ROS_INFO("dragon control: disable the joint torque");
                  servo_torque_ = false;
                }
              else
                {
                  ROS_ERROR("Failed to call service %s", joints_torque_control_srv_name_.c_str());
                }

              landing_process_ = false;
            }
        }

      if(debug_verbose_) ROS_ERROR("start lqi");
      lqi();
      if(debug_verbose_) ROS_ERROR("finish lqi");

      loop_rate.sleep();
    }
}


void DragonController::uavOdomCallback(const nav_msgs::OdometryConstPtr& odom)
{
  tf::pointMsgToTF(odom->pose.pose.position, uav_pos_);
}

void DragonController::flightCommandCallback(const std_msgs::UInt8ConstPtr& flight_command)
{
  if(!real_machine_) return;

  ros::ServiceClient client = nh_.serviceClient<dynamixel_controllers::TorqueEnable>(joints_torque_control_srv_name_);
  dynamixel_controllers::TorqueEnable srv;

  switch (flight_command->data)
    {
    case Navigator::ARM_ON_CMD:
      {
        if(servo_torque_) return;

        srv.request.torque_enable = true;

        if (client.call(srv))
          {
            ROS_INFO("dragon control: enable the joint torque");
            servo_torque_ = true;
          }
        else
          {
            ROS_ERROR("Failed to call service %s", joints_torque_control_srv_name_.c_str());
          }

        break;
      }
    case Navigator::ARM_OFF_CMD:
      {
        landing_process_ = true;
        break;
      }
    case Navigator::FORCE_LANDING_CMD:
      {
        if(!servo_torque_) return;
        srv.request.torque_enable = false;

        if (client.call(srv))
          {
            ROS_INFO("dragon control: disable the joint torque");
            servo_torque_ = false;
          }
        else
          {
            ROS_ERROR("Failed to call service %s", joints_torque_control_srv_name_.c_str());
          }

        break;
      }
    default :
      break;
    }
}


void DragonController::jointStateCallback(const sensor_msgs::JointStateConstPtr& state)
{
  TransformController::jointStateCallback(state);
}

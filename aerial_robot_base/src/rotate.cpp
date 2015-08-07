#include <ros/ros.h>
#include <aerial_robot_base/States.h>
#include <std_msgs/Float32.h>

class Rotate
{
 public:

 Rotate(ros::NodeHandle nh, ros::NodeHandle nh_private)
   : nh_(nh), nhp_(nh_private)
    {
      std::string ns = nh.getNamespace();
      nhp_.param("pub_name", pub_name_, std::string("mocap/ground_truth/pose"));
      nhp_.param("cog_rotate_sub_name", cog_rotate_sub_name_, std::string("/cog_rotate"));
      cog_offset_angle_ = 0;


      data_sub_ = nh_.subscribe<aerial_robot_base::States>(pub_name_, 1, &Rotate::rotateCallback, this, ros::TransportHints().tcpNoDelay());

      cog_offset_sub_ = nh_.subscribe<std_msgs::Float32>(cog_rotate_sub_name_, 5, &Rotate::cogOffsetCallback, this, ros::TransportHints().tcpNoDelay()); 

      rotated_pub_ = nh_.advertise<aerial_robot_base::States>("rotated_data", 5); 

    }
  ~Rotate()
    {
    }


 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher  rotated_pub_;
  ros::Subscriber data_sub_;
  ros::Subscriber cog_offset_sub_;

  float cog_offset_angle_;
  std::string pub_name_;
  std::string cog_rotate_sub_name_;

  float pos_x_offset_, pos_y_offset_, pos_z_offset_;

  void rotateCallback(const aerial_robot_base::StatesConstPtr & msg)
  {
    float psi = msg->states[3].raw_pos;
    float pitch = msg->states[4].raw_pos;
    float roll = msg->states[5].raw_pos;
    //ROS_INFO("pitch:%f, roll:%f", msg->states[4].pos,  msg->states[5].pos);

    float rotated_pitch = cos(cog_offset_angle_) * pitch - sin(cog_offset_angle_) * roll;
    float rotated_roll = sin(cog_offset_angle_) * pitch + cos(cog_offset_angle_) * roll;

    //publish for deubg, can delete
    aerial_robot_base::States ground_truth_pose;
    ground_truth_pose.header.stamp= msg->header.stamp;


    aerial_robot_base::State pitch_state;
    pitch_state.id = "pitch";
    pitch_state.raw_pos = rotated_pitch;

    aerial_robot_base::State roll_state;
    roll_state.id = "roll";
    roll_state.raw_pos = rotated_roll;
            
    ground_truth_pose.states.push_back(pitch_state);
    ground_truth_pose.states.push_back(roll_state);

    rotated_pub_.publish(ground_truth_pose);
  }

  void cogOffsetCallback(std_msgs::Float32 offset_msg)
  {
    cog_offset_angle_ =  offset_msg.data;
  }




};


int main (int argc, char **argv)
{
  ros::init (argc, argv, "aeria_robot_base");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Rotate*  rotateNode = new Rotate(nh, nh_private);
  ros::spin ();

  return 0;
}

#include "ros/ros.h"
#include "hydra/HydraParam.h"
#include "jsk_quadcopter_common/RcData.h"

void kduinoRcDataCallback(const jsk_quadcopter_common::RcDataPtr &msg)
{
  ROS_INFO("RC Data: %d, %d, %d, %d", msg->roll, msg->pitch, msg->yaw, msg->throttle);
}

void hydraParamCallback(const hydra::HydraParamPtr &msg)
{
  ROS_INFO("Hydra Param: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", msg->q_matrix[0], msg->q_matrix[1],msg->q_matrix[2],msg->q_matrix[3],msg->q_matrix[4],msg->q_matrix[5],msg->q_matrix[6],msg->q_matrix[7],msg->q_matrix[8],msg->q_matrix[9],msg->q_matrix[10],msg->q_matrix[11],msg->q_matrix[12],msg->q_matrix[13],msg->q_matrix[14],msg->q_matrix[15], msg->i_principal[0], msg->i_principal[1],msg->i_principal[2],msg->rotate_angle[0], msg->rotate_angle[1]);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hydra_test");
  ros::NodeHandle n;
  ros::Rate loop_rate(40); //40 Hz
  ros::Subscriber  test_sub1_ = n.subscribe("kduino/rc_res", 1, kduinoRcDataCallback);
  ros::Subscriber  test_sub2_ = n.subscribe("kduino/hy_res", 1, hydraParamCallback);
  ros::Publisher  test_pub_ = n.advertise<hydra::HydraParam>("kduino/hydra_param",1);
  
  float start_time = ros::Time::now().toSec();

  while (ros::ok())
    {
      float diff_time = ros::Time::now().toSec() - start_time;

      hydra::HydraParam hydra_param;
      for(int i = 0; i < 16; i++)
        {
          hydra_param.q_matrix[i] = 100*sin(diff_time * i);
        }
      hydra_param.i_principal[0] = 200*sin(diff_time);
      hydra_param.i_principal[1] = 400*sin(diff_time);
      hydra_param.i_principal[2] = 600*sin(diff_time);
      hydra_param.rotate_angle[0] = 1000*sin(diff_time);
      hydra_param.rotate_angle[1] = 1000*cos(diff_time);

      test_pub_.publish(hydra_param);
      ros::spinOnce();
      loop_rate.sleep();

    }
  

  return 0;
}

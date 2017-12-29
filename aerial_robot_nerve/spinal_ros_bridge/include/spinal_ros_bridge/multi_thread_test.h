#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <ros/advertise_service_options.h>

/*
   analysis:
   one while function is for one thread, so need threads = number of services + 1
*/

class MultiThreadTest
{
public:
  MultiThreadTest(ros::NodeHandle nh):
    spinner_(2), trigger_(false)
  {
    nh2_.setCallbackQueue(&local_queue_);
    trigger_sub_ = nh_.subscribe("/enable_trigger", 1, &MultiThreadTest::triggerCallback, this);
    echo_sub_ = nh_.subscribe("/echo_sub", 1, &MultiThreadTest::echoCallback, this);
    echo_pub_ = nh_.advertise<std_msgs::String>("/echo_pub", 1);
    timer_ = nh_.createTimer(ros::Duration(0.1), &MultiThreadTest::mainFunc,this);

    /* local callback */
    test_service_ = nh_.advertiseService("/serive_test", &MultiThreadTest::serviceTestCallback, this);
    test_service2_ = nh_.advertiseService("/serive_test2", &MultiThreadTest::serviceTestCallback2, this);


    ros::AdvertiseServiceOptions opt;
    opt.init<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(std::string("hoge"), boost::bind(&MultiThreadTest::serviceTestCallback, this, _1, _2));

    ROS_ERROR("opt.datatype: %s, opt.req_datatype: %s, opt.res_datatype: %s",opt.datatype.c_str(),  opt.req_datatype.c_str(), opt.res_datatype.c_str());
    
    //spinner_.spin(&local_queue_);
    spinner_.spin();

    /* global callback */
    //ros::spin ();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh2_;

  ros::Subscriber echo_sub_;
  ros::Publisher echo_pub_;
  ros::Subscriber trigger_sub_;
  ros::ServiceServer test_service_;
  ros::ServiceServer test_service2_;
  ros::Timer timer_;

  ros::MultiThreadedSpinner spinner_;
  ros::CallbackQueue local_queue_;

  bool trigger_;

  void echoCallback(const std_msgs::Empty & msg )
  {
    std_msgs::String pub_msg;
    pub_msg.data = std::string("aotti");
    echo_pub_.publish(pub_msg);
    ROS_INFO("aotti");
  }

  void triggerCallback(const std_msgs::Empty & msg)
  {
    trigger_ = true;
  }


  void mainFunc(const ros::TimerEvent & e)
  {
    ROS_INFO("timer");
  }

  bool serviceTestCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {

    while(!trigger_){}

    res.message = std::string("neneti~");
    res.success = true;

    trigger_ = false;

    return true;
  }

  bool serviceTestCallback2(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {

    while(!trigger_){}

    res.message = std::string("aotti~");
    res.success = true;

    trigger_ = false;

    return true;
  }

};


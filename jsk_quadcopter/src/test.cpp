#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <jsk_quadcopter/ImuDebug.h>
#include <jsk_quadcopter/QuadcopterStatus.h>
#include <jsk_quadcopter/SlamDebug.h>
#include <jsk_quadcopter/MirrorModuleDebug.h>
#include <tf/transform_broadcaster.h>

int i = 0;

class test1
{
public:
  void call1(const ros::TimerEvent & e)
  {
    ROS_INFO("call1");
    i++;
    vi = 0;
    viPtr = &vi;

    std_msgs::Float64 msg;
    msg.data = -1;
    call_pub_.publish(msg);

  }

  void call()
  {
    ROS_INFO("call1");
    vi ++;
  }

  void start_callback(const std_msgs::EmptyConstPtr & msg, int i)
  {
    ROS_INFO("receive the start key %d",i);
  }
  
  const static int p = 111;

  int vi;
  int* viPtr;



  ros::Publisher call_pub_;

  virtual ~test1()
  {
    delete viPtr;
    printf("end test1\n");
  }

};

class test2
{
public:
  test2(ros::NodeHandle nh): nh_(nh, "test2")
  {
    test2_pub_ = nh_.advertise<jsk_quadcopter::ImuDebug>("imu_debug", 1);
  };
  virtual ~test2(){};

  void call2(const ros::TimerEvent & e)
  {
    jsk_quadcopter::ImuDebug imuDebug_;
    imuDebug_.header.stamp = ros::Time::now();
    imuDebug_.theta = 1.125;
    imuDebug_.phy = 2.75;
    imuDebug_.psi = 3.25;
    imuDebug_.height = 4.5;
    //ROS_INFO("                       call2: %d", i);
    test2_pub_.publish(imuDebug_);
  }

  void start2_callback(const std_msgs::EmptyConstPtr & msg)
  {
    ROS_INFO("receive the start key2");
  }
private :
  ros::NodeHandle nh_;
  ros::Publisher test2_pub_;
};


class test : public test1 //, public test2 
{
public:

  test2* tes2;

  test(ros::NodeHandle nh, std::string str) : test1(), nh_(nh), str3(str)
  { 
    ros::Subscriber start_sub_ = nh_.subscribe<std_msgs::Empty>("keyboard_command/start", 1, boost::bind(&test::stop_callback, this, _1));
    tes2 = new test2(nh_); 
    br_ = new tf::TransformBroadcaster();

    vi3 = & vi;

    ROS_INFO("the str is %s", str3.c_str());


  };

  test(int input){ fun = input; };

  ~test()
  {
    printf("end test3\n");
    delete tes2;
    delete br_;
  };



  void call3(const int & i)
  {
    tf::Transform transform_;

    tf::Quaternion tmp_;
    tmp_.setRPY(0.0 , 0.0 , 3*M_PI/4);
    transform_.setOrigin(tf::Vector3(0.28, 0.0, 0.0));
    transform_.setRotation(tmp_);
    ros::Time tm_ = ros::Time::now(); 
    br_->sendTransform(tf::StampedTransform(transform_, tm_, "wiimote", "IR"));

    call();
    
#if 1
    //ROS_INFO("vi3 :%d", *vi3);
    ROS_INFO("%d  %d", sizeof(vi3),  sizeof(*vi3));
    switch (i)
      {
      case p:
	ROS_INFO("                       call3: %d", i);
	ROS_INFO("                       %d", sizeof(funArray));
	break;
      default:
	break;
      }
#endif

    //ROS_INFO("wi is %d", wi);

  }

  void stop_callback(const std_msgs::EmptyConstPtr & msg)
  {
    ROS_INFO("receive the stop key");
  }
 
private:
  ros::NodeHandle nh_;
  tf::TransformBroadcaster* br_;

  std::string str3;
  int fun;
  int funArray[p];
  static int staFun ;

  int* vi3;
  int wi;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "test");


  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_("~");

  
  // test::staFun = 1;
  // ROS_INFO("%d", test::staFun);
  std::string str_("ok");

  test* test_test = new test(nh_, str_);
  test_test->call_pub_ = nh_.advertise<std_msgs::Float64>("test",1);

  // test  test_test[3];
  // test_test[0] = test(1);
  // test_test[1] = test(2);
  // test_test[2] = test(3);

  ros::Publisher  marker_pub = nh_.advertise<std_msgs::Empty>("ctrl_arrow", 1);
  ros::Publisher  value_pub = nh_private_.advertise<std_msgs::Empty>("ctrl_value", 1);

  //ros::Subscriber stop_sub_ = nh_.subscribe<std_msgs::Empty>("keyboard_command/stop", 1, boost::bind(&test::stop_callback, test_test, _1));


  ros::Subscriber start_sub_ = nh_.subscribe<std_msgs::Empty>("keyboard_command/start", 1,boost::bind(&test::start_callback, test_test, _1, 1));

  //ros::Subscriber start_sub_ = nh_.subscribe<std_msgs::Empty>("keyboard_command/start", 1, &test2::start2_callback, test_test->tes2);



  //+*+*+*+* 多重継承の基底クラスのメンバ関数を用いてcreateTimerを作ることはできない
  //ros::Timer timer_1 = nh_private_.createTimer (ros::Duration(0.1), boost::bind(&test::call1, test_test, _1));
  
  //ros::Timer timer_2 = nh_private_.createTimer (ros::Duration(0.01), boost::bind(&test::call2, test_test, _1));

  ros::Timer timer_2 = nh_private_.createTimer (ros::Duration(0.01), boost::bind(&test2::call2, test_test->tes2, _1));



#if 1
  ros::Rate rate(1.0);
  int cnt = 0;
  
  boost::function <void ()> call3_new = boost::bind(&test::call3, test_test, test_test->p);
  while (nh_.ok()){

    call3_new();
    //test_test->call();

    ros::spinOnce();
    rate.sleep();
  }

#else

  ros::spin ();

#endif
  printf("end\n");

  delete test_test;

  return 0;


};

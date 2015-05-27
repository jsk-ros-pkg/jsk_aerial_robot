/* 
0. flightNavCallback => implementation
1. the optical flow => indefference
2. rocket start => depracated
3. free fall
4. flight mode vs navi command => development


 */

#include "aerial_robot_base/flight_navigation.h"

Navigator::Navigator(ros::NodeHandle nh, ros::NodeHandle nh_private, 
                     BasicEstimator* estimator, FlightCtrlInput* flight_ctrl_input,
                     int ctrl_loop_rate)
 : nh_(nh, "navigator"),
   nhp_(nh_private, "navigator"),
   ctrl_loop_rate_(ctrl_loop_rate)
{
  Navigator::rosParamInit(nhp_);

  navi_sub_ = nh_.subscribe<aerial_robot_base::FlightNav>("full_states", 1, &Navigator::naviCallback, this, ros::TransportHints().tcpNoDelay());

  estimator_ = estimator;
  flight_ctrl_input_ = flight_ctrl_input;

  br_ =  new tf::TransformBroadcaster();

  final_target_pos_x_ = 0;
  final_target_vel_x_ = 0;
  final_target_pos_y_ = 0;
  final_target_vel_y_ = 0;
  final_target_pos_z_ = 0;
  final_target_vel_z_ = 0;
  final_target_theta_ = 0;
  final_target_vel_theta_ = 0;
  final_target_phy_ = 0;
  final_target_vel_phy_ = 0;
  final_target_psi_ = 0;
  final_target_vel_psi_ = 0;

  //current target value
  current_target_pos_x_ = 0;
  current_target_vel_x_ = 0;
  current_target_pos_y_ = 0;
  current_target_vel_y_ = 0;
  current_target_pos_z_ = 0;
  current_target_vel_z_ = 0;
  current_target_theta_ = 0;
  current_target_vel_theta_ = 0;
  current_target_phy_ = 0;
  current_target_vel_phy_ = 0;
  current_target_psi_ = 0;
  current_target_vel_psi_ = 0;

  stopNavigation();
  stopFlight();
  setNaviCommand( IDLE_COMMAND );


  //base navigation mode init
  flight_mode_ = NO_CONTROL_MODE;

  //free fall mode init
  free_fall_flag_ = false;
  motor_stop_flag_ =false;
  //throwing mode init
  throwing_mode_ = NO_CONTROL_MODE;


}

Navigator::~Navigator()
{
  printf(" deleted navigator!\n");
  delete br_;
}

void Navigator::naviCallback(const aerial_robot_base::FlightNavConstPtr & msg)
{
  //for x & y
  if(msg->command_mode == aerial_robot_base::FlightNav::VEL_FLIGHT_MODE_COMMAND)
    {
      setTargetVelX(msg->target_vel_x);
      setTargetVelY(msg->target_vel_y);
    }
  else if(msg->command_mode == aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND)
    {
      setTargetPosX(msg->target_pos_x);
      setTargetPosY(msg->target_pos_y);
    }

  //for z
  if(msg->pos_z_navi_mode == aerial_robot_base::FlightNav::VEL_FLIGHT_MODE_COMMAND)
    {
      addTargetPosZ(msg->target_pos_diff_z);
    }
  else if(msg->pos_z_navi_mode == aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND)
    {
      setTargetPosZ(msg->target_pos_z);
    }

  //for psi
  //not good, be carefukk
  if(msg->command_mode != aerial_robot_base::FlightNav::NO_NAVIGATION)
    {
      setTargetPsi(msg->target_psi);
    }
}


//1 implemented in  a more higher rate loop 
//2 implement callback function
//3 now: a 40Hz function ( z_offset is not important for flight control)
void Navigator::throwingModeNavi()
{
  static int ready_to_standby_cnt = 0;
  static int ready_to_arm_cnt = 0;
  static int ready_to_set_alt_cnt = 0;
  static int ready_to_alt_hold_cnt = 0;
  static int xy_vel_state = GOOD_XY_TRACKING;

  float pos_z = estimator_->getStatePosZ();
  float vel_z = estimator_->getStateVelZ();
  float acc_z = estimator_->getStateAccZb() - 9.8;

  float vel_x = estimator_->getStateVelXc();
  //float vel_y = estimator_->getStateVelYc();
  //float vel_x = estimator_->getStateVelX();

  float acc_x = estimator_->getStateAccXb();
  float acc_y = estimator_->getStateAccYb();

  static float prev_acc_x, prev_acc_z, prev_vel_z;

  if(throwing_mode_ == NO_CONTROL_MODE)
    {
      //+*+*+* for alt control
      if(getNaviCommand() == IDLE_COMMAND)
        {
          if(pos_z > throwing_mode_standby_alt_thre_) // altitude thre + getNaviCommand() == IDLE_COMMAND
            ready_to_standby_cnt ++;
          else 
            ready_to_standby_cnt = 0;
          if(ready_to_standby_cnt > throwing_mode_shift_step_cnt_thre_ * 2)
            {
              throwing_mode_ = THROWING_START_STANDBY;
              ROS_INFO("shift to THROWING_START_STANDBY");
            }
        }
    }

  else if(throwing_mode_ == THROWING_START_STANDBY)
    {
      //+*+* for alt control
      if(acc_x > throwing_mode_throw_acc_x_thre_ &&
         acc_z > throwing_mode_throw_acc_z_thre_) // altitude vel/pos/acc + x acc/vel + timer(e.g. 0.1s)
        {
          ROS_INFO("both acc x/z is big enough, accX:%f, accZ:%f", acc_x, acc_z);
          if(ready_to_arm_cnt == 0)
            {
              ready_to_arm_cnt ++;
              prev_acc_x = acc_x;
              prev_acc_z = acc_z;
            }
          else
            {
              // if(acc_x > prev_acc_x && acc_z > prev_acc_z)
              if(acc_x > prev_acc_x )
                {
                  ready_to_arm_cnt ++;
                  ROS_INFO("acc x/z is bigger than previous ones");
                }
              else
                ready_to_arm_cnt = 0;
            }
        }
      else
        ready_to_arm_cnt = 0;

      if(ready_to_arm_cnt > throwing_mode_shift_step_cnt_thre_)
        {
          ready_to_arm_cnt = 0;
          throwing_mode_ = THROWING_START_ARMINGON;
          setNaviCommand(START_COMMAND);
          startNavigation();
          ROS_ERROR("shift to arming mode");

        }
    }

  else if(throwing_mode_ == THROWING_START_ARMINGON)
    {
      ROS_INFO("arming mode");

      //+*+* for alt control
      if(acc_z < throwing_mode_set_alt_acc_z_thre_)  // -5
        {
          if(ready_to_set_alt_cnt < throwing_mode_shift_step_cnt_thre_)
            {
              if(ready_to_set_alt_cnt == 0)
                {
                  ready_to_set_alt_cnt ++;
                  prev_acc_z = acc_z;
                }
              else
                {
                  if(acc_z < prev_acc_z)
                    ready_to_set_alt_cnt ++;
                  else 
                    ready_to_set_alt_cnt = 0;
                }
            }
        }
      else
        ready_to_set_alt_cnt = 0;

      if(vel_z < throwing_mode_alt_hold_vel_z_thre_)  //0.5
        {
          if(ready_to_set_alt_cnt > throwing_mode_shift_step_cnt_thre_)
            {
              if(ready_to_alt_hold_cnt == 0)
                {
                  ready_to_alt_hold_cnt += throwing_mode_shift_step_cnt_thre_;
                  prev_vel_z = vel_z;
                }
            }
        }
      else
        ready_to_alt_hold_cnt = 0;


      //+*+* for optflow
      //TODO : remove the optflow
      if(estimator_->getStateVelXOpt() == 0) 
        {
          xy_vel_state = ZERO_XY_TRACKING;
          bool stop_flag = false;
          estimator_->setKFMeaureFlag(BasicEstimator::X_AXIS, stop_flag);
          estimator_->setKFMeaureFlag(BasicEstimator::Y_AXIS, stop_flag);
          ROS_WARN("ZERO TRACKING");
        }

      if(ready_to_set_alt_cnt == throwing_mode_shift_step_cnt_thre_)
        {
          final_target_pos_z_ = pos_z + 0.5 * vel_z * vel_z / 9.8 + 0.2;
          ready_to_set_alt_cnt++;
          ROS_ERROR("set alt, %f", final_target_pos_z_);
        }
      if(ready_to_alt_hold_cnt == throwing_mode_shift_step_cnt_thre_)
        {
          ready_to_set_alt_cnt = 0;
          ready_to_alt_hold_cnt = 0;
          throwing_mode_ = THROWING_START_ALT_HOLD;
          startFlight(); //not very good , temporarily
          setNaviCommand(HOVER_COMMAND); //special
          ROS_ERROR("shift to alt_hold mode");
        }

    }
  else if(throwing_mode_ == THROWING_START_ALT_HOLD)
    {
      //+*+* for alt control
      setNaviCommand(HOVER_COMMAND); //special, escape from start res from quadcopter
      ROS_INFO("alt hold mode");
      //very simple way
      if(vel_x < 0) //todo: altitude pos + x vel + etc
        {
          ROS_INFO("takeoff mode");
          throwing_mode_ = FLIGHT_MODE; //to avoid loop
          final_target_pos_x_ = estimator_->getStatePosXc();
          final_target_pos_y_ = estimator_->getStatePosYc();

          //final_target_pos_x_ = estimator_->getStatePosX();
          //final_target_pos_y_ = estimator_->getStatePosY();
        }

      //*+*+ for optical flow => bad
      if(xy_vel_state == ZERO_XY_TRACKING && 
         acc_z > 0 && estimator_->getStateVelXOpt() > 0.25)
        {
          xy_vel_state = RECOVER_XY_TRACKING;
          bool start_flag = true;
          estimator_->setKFMeaureFlag(BasicEstimator::X_AXIS, start_flag);
          estimator_->setKFMeaureFlag(BasicEstimator::Y_AXIS, start_flag);
          ROS_WARN("RECOVER TRACKING");
        }
    }

  else
    {
      //do nothing
    }
}


void Navigator::tfPublish()
{
  //TODO mutex
  tf::Transform map_to_target;
  tf::Quaternion tmp;
  
  map_to_target.setOrigin(tf::Vector3(current_target_pos_x_, current_target_pos_y_, current_target_pos_z_));
  tmp.setRPY(0.0, 0.0, current_target_psi_); 
  map_to_target.setRotation(tmp);
  ros::Time tm = ros::Time::now();
  br_->sendTransform(tf::StampedTransform(map_to_target, tm, map_frame_, target_frame_));

}

void Navigator::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nh.getNamespace();

  //*** teleop navigation
  if (!nh.getParam ("use_free_fall", use_free_fall_))
    use_free_fall_ = false;
  printf("%s: use_free_fall is %s\n", ns.c_str(), use_free_fall_ ? ("true") : ("false"));
  if (!nh.getParam ("free_fall_thre", free_fall_thre_))
    free_fall_thre_ = 0;
  printf("%s: free_fall_thre_ is %.3f\n", ns.c_str(), free_fall_thre_);

  if (!nh.getParam ("map_frame", map_frame_))
    map_frame_ = "unknown";
  printf("%s: map_frame_ is %s\n", ns.c_str(), map_frame_.c_str());

  if (!nh.getParam ("target_frame", target_frame_))
    target_frame_ = "unknown";
  printf("%s: target_frame_ is %s\n", ns.c_str(), target_frame_.c_str());

  if (!nh.getParam ("xy_control_mode", xy_control_mode_))
    xy_control_mode_ = 0;
  printf("%s: xy_control_mode_ is %d\n", ns.c_str(), xy_control_mode_);

  //hidden variable
  if (!nh.getParam ("xy_vel_mode_pos_ctrl_takeoff", xy_vel_mode_pos_ctrl_takeoff_))
    xy_vel_mode_pos_ctrl_takeoff_ = true;
  printf("%s: xy_vel_mode_pos_ctrl_takeoff is %s\n", ns.c_str(), xy_vel_mode_pos_ctrl_takeoff_ ? ("true") : ("false"));

  if (!nh.getParam ("use_throwing_mode", use_throwing_mode_))
    use_throwing_mode_ = false;
  printf("%s: use_throwing_mode is %s\n", ns.c_str(), use_throwing_mode_ ? ("true") : ("false"));

  if (!nh.getParam ("throwing_mode_standby_alt_thre", throwing_mode_standby_alt_thre_))
    throwing_mode_standby_alt_thre_ = 0;
  printf("%s: throwing_mode_standby_alt_thre_ is %f\n", ns.c_str(), throwing_mode_standby_alt_thre_);

  if (!nh.getParam ("throwing_mode_throw_acc_x_thre", throwing_mode_throw_acc_x_thre_))
    throwing_mode_throw_acc_x_thre_ = 0;
  printf("%s: throwing_mode_throw_acc_x_thre_ is %f\n", ns.c_str(), throwing_mode_throw_acc_x_thre_);

  if (!nh.getParam ("throwing_mode_throw_acc_z_thre", throwing_mode_throw_acc_z_thre_))
    throwing_mode_throw_acc_z_thre_ = 0;
  printf("%s: throwing_mode_throw_acc_z_thre_ is %f\n", ns.c_str(), throwing_mode_throw_acc_z_thre_);

  if (!nh.getParam ("throwing_mode_set_alt_acc_z_thre", throwing_mode_set_alt_acc_z_thre_))
    throwing_mode_set_alt_acc_z_thre_ = 0;
  printf("%s: throwing_mode_set_alt_acc_z_thre_ is %f\n", ns.c_str(), throwing_mode_set_alt_acc_z_thre_);

  if (!nh.getParam ("throwing_mode_alt_hold_vel_z_thre", throwing_mode_alt_hold_vel_z_thre_))
    throwing_mode_alt_hold_vel_z_thre_ = 0;
  printf("%s: throwing_mode_alt_hold_vel_z_thre_ is %f\n", ns.c_str(), throwing_mode_alt_hold_vel_z_thre_);

  if (!nh.getParam ("throwing_mode_shift_step_cnt_thre", throwing_mode_shift_step_cnt_thre_))
    throwing_mode_shift_step_cnt_thre_ = 0;
  printf("%s: throwing_mode_shift_step_cnt_thre_ is %d\n", ns.c_str(), throwing_mode_shift_step_cnt_thre_);

}




TeleopNavigator::TeleopNavigator(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 BasicEstimator* estimator, 
                                 FlightCtrlInput* flight_ctrl_input,
                                 int ctrl_loop_rate)
  :Navigator(nh, nh_private, estimator, flight_ctrl_input, ctrl_loop_rate)
{
  TeleopNavigator::rosParamInit(nhp_);


  //joystick init
  vel_control_flag_ = false;
  pos_control_flag_ = false;
  alt_control_flag_ = false;
  yaw_control_flag_ = false;


  arming_ack_sub_ = nh_.subscribe<std_msgs::Int8>("kduino/arming_ack", 1, &TeleopNavigator::armingAckCallback, this, ros::TransportHints().tcpNoDelay());
  takeoff_sub_ = nh_.subscribe<std_msgs::Empty>("teleop_command/takeoff", 1, &TeleopNavigator::takeoffCallback, this, ros::TransportHints().tcpNoDelay());
  halt_sub_ = nh_.subscribe<std_msgs::Empty>("teleop_command/halt", 1, &TeleopNavigator::haltCallback, this, ros::TransportHints().tcpNoDelay());
  land_sub_ = nh_.subscribe<std_msgs::Empty>("teleop_command/land", 1, &TeleopNavigator::landCallback, this, ros::TransportHints().tcpNoDelay());
  start_sub_ = nh_.subscribe<std_msgs::Empty>("teleop_command/start", 1,&TeleopNavigator::startCallback, this, ros::TransportHints().tcpNoDelay());
  roll_sub_ = nh_.subscribe<std_msgs::Int8>("teleop_command/roll", 1, &TeleopNavigator::rollCallback, this, ros::TransportHints().tcpNoDelay());
  pitch_sub_ = nh_.subscribe<std_msgs::Int8>("teleop_command/pitch", 1, &TeleopNavigator::pitchCallback, this, ros::TransportHints().tcpNoDelay());
  yaw_sub_ = nh_.subscribe<std_msgs::Int8>("teleop_command/yaw", 1, &TeleopNavigator::yawCallback, this, ros::TransportHints().tcpNoDelay());
  throttle_sub_ = nh_.subscribe<std_msgs::Int8>("teleop_command/throttle", 1, &TeleopNavigator::throttleCallback, this, ros::TransportHints().tcpNoDelay());
  ctrl_mode_sub_ = nh_.subscribe<std_msgs::Int8>("teleop_command/ctrl_mode", 1, &TeleopNavigator::xyControlModeCallback, this, ros::TransportHints().tcpNoDelay());

  joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy_stick_command", 1, &TeleopNavigator::joyStickControl, this, ros::TransportHints().tcpNoDelay());
  rc_cmd_pub_ = nh_.advertise<aerial_robot_msgs::RcData>("kduino/rc_cmd", 10); 
  msp_cmd_pub_ = nh_.advertise<std_msgs::UInt16>("kduino/msp_cmd", 10);

}

TeleopNavigator::~TeleopNavigator()
{
  printf(" deleted teleop navigator input!\n");
}

void TeleopNavigator::armingAckCallback(const std_msgs::Int8ConstPtr& ack_msg)
{
  if(ack_msg->data == 0)
    {//  arming off
      ROS_INFO("STOP RES From AERIAL ROBOT");
      stopNavigation(); 
      setNaviCommand(IDLE_COMMAND);
    }
 
 if(ack_msg->data == 1)
    {//  arming on
      ROS_INFO("START RES From AERIAL ROBOT");
      startNavigation(); 
      setNaviCommand(IDLE_COMMAND);
    }
}


void TeleopNavigator::takeoffCallback(const std_msgs::EmptyConstPtr & msg){
  if(getStartAble())  
    {
      if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
      setNaviCommand(TAKEOFF_COMMAND);
      ROS_INFO("Takeoff command");
    }
  else  stopFlight();
}

void TeleopNavigator::startCallback(const std_msgs::EmptyConstPtr & msg)
  {//すべて軸に対して、初期化
    setNaviCommand(START_COMMAND);
    final_target_pos_x_ = estimator_->getStatePosX();
    final_target_pos_y_ = estimator_->getStatePosY();
    final_target_pos_z_ = takeoff_height_;  // 0.55m

    final_target_psi_ = estimator_->getStatePsiBoard();
    ROS_INFO("Start command");
  }

void TeleopNavigator::landCallback(const std_msgs::EmptyConstPtr & msg)
  {
    if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
    setNaviCommand(LAND_COMMAND);
    //更新
    final_target_pos_x_ = estimator_->getStatePosX();
    final_target_pos_y_ = estimator_->getStatePosY();
    final_target_pos_z_ = 0; 
    final_target_psi_  = estimator_->getStatePsiBoard();
    ROS_INFO("Land command");
  }

void TeleopNavigator::haltCallback(const std_msgs::EmptyConstPtr & msg)
  {
    if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
    setNaviCommand(STOP_COMMAND);
    flight_mode_ = RESET_MODE;

    setTargetPosX(estimator_->getStatePosX());
    setTargetPosY(estimator_->getStatePosY());
    setTargetPsi(estimator_->getStatePsiBoard());


    ROS_INFO("Halt command");
  }

void TeleopNavigator::rollCallback(const std_msgs::Int8ConstPtr & msg)
  {// + : right ; - : left
    if(getStartAble() && getFlightAble())
      {
	setNaviCommand(HOVER_COMMAND);
	if(msg->data == 1){
	  ROS_INFO("RIGHT");
	  if(navi_frame_ == MAP_FRAME)
	    final_target_pos_y_ -= even_move_distance_; //0.2m on now state
	  else if(navi_frame_ == BODY_FRAME)
	    {
	    if(fabs(cos(estimator_->getStatePsiBoard())) > 
	       fabs(sin(estimator_->getStatePsiBoard())))
	      {
		final_target_pos_y_-= left_right_distance_ *
		  (fabs(cos(estimator_->getStatePsiBoard()))/cos(estimator_->getStatePsiBoard()));
		final_target_pos_x_ = estimator_->getStatePosX();
	      }
	    else if(fabs(cos(estimator_->getStatePsiBoard())) <
		    fabs(sin(estimator_->getStatePsiBoard())))
	      {
		final_target_pos_x_ += left_right_distance_ *
		  (fabs(sin(estimator_->getStatePsiBoard()))/sin(estimator_->getStatePsiBoard()));
		final_target_pos_y_= estimator_->getStatePosY();
	      
	      }
	    }
	}else{
	  ROS_INFO("LEFT");
	  if(navi_frame_ == MAP_FRAME)
	    final_target_pos_y_+= even_move_distance_; //0.2m on now state
	  else if(navi_frame_ == BODY_FRAME)
	    {
	      if(fabs(cos(estimator_->getStatePsiBoard())) >
		 fabs(sin(estimator_->getStatePsiBoard())))
		{
		  final_target_pos_y_+= left_right_distance_ * 
		    (fabs(cos(estimator_->getStatePsiBoard())) / cos(estimator_->getStatePsiBoard()));
		  final_target_pos_x_= estimator_->getStatePosX();
		}
	      else if(fabs(cos(estimator_->getStatePsiBoard())) <
		      fabs(sin(estimator_->getStatePsiBoard())))
		{
		  final_target_pos_x_-= left_right_distance_ * 
		    (fabs(sin(estimator_->getStatePsiBoard())) / sin(estimator_->getStatePsiBoard()));
		  final_target_pos_y_= estimator_->getStatePosY();
		}
	    }
	}
      }
  }

void TeleopNavigator::pitchCallback(const std_msgs::Int8ConstPtr & msg)
{// + : backward ; - : forward
  if(getStartAble() && getFlightAble())  
    {
      setNaviCommand(HOVER_COMMAND);
      if(msg->data == 1)
	{
	  ROS_INFO("FORWARD");
	  if(navi_frame_ == MAP_FRAME) final_target_pos_x_+= even_move_distance_; //0.2m
	  else if(navi_frame_ == BODY_FRAME)
	    {
	      if(fabs(cos(estimator_->getStatePsiBoard())) >
		 fabs(sin(estimator_->getStatePsiBoard()))){
		final_target_pos_x_ += forward_backward_distance_ * 
		  (fabs(cos(estimator_->getStatePsiBoard())) / cos(estimator_->getStatePsiBoard()));
		//final_target_pos_y_ = estimator_->getStatePosy_();
	      }
	      else if(fabs(cos(estimator_->getStatePsiBoard())) <
		      fabs(sin(estimator_->getStatePsiBoard()))){
		//final_target_pos_x_= estimator_->getStatePosX();
		final_target_pos_y_ += forward_backward_distance_ * 
		  (fabs(sin(estimator_->getStatePsiBoard())) / sin(estimator_->getStatePsiBoard()));
	      }
	    }
	}
      else
	{
	  ROS_INFO("BACKFORWARD");
	  if(navi_frame_ == MAP_FRAME) final_target_pos_x_-= even_move_distance_; //0.2m
	  else if(navi_frame_ == BODY_FRAME)
	    {
              //bad
	      final_target_pos_x_= estimator_->getStatePosX();
	      final_target_pos_y_ = estimator_->getStatePosY();
	    }
	}
    }
}


void TeleopNavigator::yawCallback(const std_msgs::Int8ConstPtr & msg)
{
  if(getStartAble() && getFlightAble())  
    { 
      setNaviCommand(HOVER_COMMAND);

      if(navi_frame_ == BODY_FRAME)
	{
	  final_target_pos_x_ = estimator_->getStatePosX();
	  final_target_pos_y_ = estimator_->getStatePosY();
	}


      if(msg->data == -1){
        final_target_psi_ = estimator_->getStatePsiBoard() - M_PI/2.0; 
	ROS_INFO("CW");
      }else if(msg->data == 1){
        final_target_psi_ = estimator_->getStatePsiBoard() + M_PI/2.0; 
	ROS_INFO("CCW");
      }

      if(msg->data == -2){
	//final_target_psi_ = estimator_->getStatePsi() - 0.1; //5 degree 
        final_target_psi_ -= 0.1; //bad method
	ROS_INFO("delta:CW");
      }else if(msg->data == 2){
	//final_target_psi_ = estimator_->getStatePsi() + 0.1;  //5 degree
        final_target_psi_ += 0.1; //bad method
	ROS_INFO("delta:CCW");
      }
      else
        {
          //nothing
        }
      //yaw_finalTargetの補正
      if(final_target_psi_ > M_PI) final_target_psi_ -= 2 * M_PI;
      else if(final_target_psi_ < -M_PI) final_target_psi_ += 2 *M_PI;
    }

}

void TeleopNavigator::throttleCallback(const std_msgs::Int8ConstPtr & msg)
{
  if(getStartAble() && getFlightAble())  
    { 
      setNaviCommand(HOVER_COMMAND);

      if(msg->data == 1){
	final_target_pos_z_ += up_down_distance_; 
	ROS_INFO("UP");
      }else{
	final_target_pos_z_ -= up_down_distance_; 
	ROS_INFO("DOWN");
      }
    }
  //ROS_INFO("Thrust command");
}

void TeleopNavigator::xyControlModeCallback(const std_msgs::Int8ConstPtr & msg)
{
    if(getStartAble())
    {
      if(msg->data == 0)
        {
          xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
          ROS_INFO("x/y position control mode");
        }
      if(msg->data == 1)
        {
          xy_control_mode_ = VEL_LOCAL_BASED_CONTROL_MODE;
          ROS_INFO("x/y velocity control mode");
        }
    }
}


void TeleopNavigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE || xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE)
    {
      //start 
      if(joy_msg->buttons[3] == 1 && getNaviCommand() != START_COMMAND)
        {
          setNaviCommand(START_COMMAND);
          final_target_pos_x_= estimator_->getStatePosX();
          final_target_pos_y_= estimator_->getStatePosY();
          final_target_pos_z_ = takeoff_height_;  // 0.55m
          final_target_psi_  = estimator_->getStatePsiBoard();
          ROS_INFO("Start command");
          return;
        }
      //halt
      if(joy_msg->buttons[0] == 1)
        {
          setNaviCommand(STOP_COMMAND);
          flight_mode_= RESET_MODE;

          setTargetPosX(estimator_->getStatePosX());
          setTargetPosY(estimator_->getStatePosY());
          setTargetPsi(estimator_->getStatePsiBoard());


          if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
          ROS_INFO("Halt command");
          return;
        }
      //takeoff
      if(joy_msg->buttons[7] == 1 && joy_msg->buttons[13] == 1)
        {

          if(getStartAble())  
            {
              if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
              setNaviCommand(TAKEOFF_COMMAND);
              ROS_INFO("Takeoff command");
            }
          else  stopFlight();

          return;
        }
      //landing
      if(joy_msg->buttons[5] == 1 && joy_msg->buttons[15] == 1)
        {
          if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;

          setNaviCommand(LAND_COMMAND);
          //更新
          final_target_pos_x_= estimator_->getStatePosX();
          final_target_pos_y_= estimator_->getStatePosY();
          final_target_pos_z_= 0; 
          final_target_psi_  = estimator_->getStatePsiBoard();
          ROS_INFO("Land command");

          return;
        }

      //change to vel control mode
      if(joy_msg->buttons[12] == 1 && !vel_control_flag_)
        {
          ROS_INFO("change to vel pos-based control");
          vel_control_flag_ = true;
          xy_control_mode_ = VEL_WORLD_BASED_CONTROL_MODE;
          final_target_vel_x_= 0; current_target_vel_x_= 0;
          final_target_vel_y_= 0; current_target_vel_y_= 0;
        }
      if(joy_msg->buttons[12] == 0 && vel_control_flag_)
        vel_control_flag_ = false;

      //change to pos control mode
      if(joy_msg->buttons[14] == 1 && !pos_control_flag_)
        {
          ROS_INFO("change to pos control");
          pos_control_flag_ = true;
          xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
          final_target_pos_x_= estimator_->getStatePosX();
          final_target_pos_y_= estimator_->getStatePosY();
        }
      if(joy_msg->buttons[14] == 0 && pos_control_flag_)
        pos_control_flag_ = false;

      //pitch && roll vel command for vel_mode
      if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE && getNaviCommand() == HOVER_COMMAND)
        {
          if(joy_msg->buttons[1] == 0)
            {//no push the left joysitck
              final_target_vel_x_= joy_msg->axes[1] * fabs(joy_msg->axes[1]) * target_vel_rate_;
              final_target_vel_y_= joy_msg->axes[0] * fabs(joy_msg->axes[0]) * target_vel_rate_;
            }
          if(joy_msg->buttons[1] == 1)
            {//push the left joysitck
              ROS_INFO("strong vel control");
              final_target_vel_x_
                = joy_msg->axes[1] * fabs(joy_msg->axes[1]) * target_vel_rate_ * cmd_vel_lev2_gain_;
              final_target_vel_y_
                = joy_msg->axes[0] * fabs(joy_msg->axes[0]) * target_vel_rate_ * cmd_vel_lev2_gain_;
            }
        }

      //throttle, TODO: not good
      if(fabs(joy_msg->axes[3]) > 0.2)
        {
          if(joy_msg->buttons[2] == 0 && getNaviCommand() == HOVER_COMMAND)
            {//push the right joysitck
              alt_control_flag_ = true;
              if(joy_msg->axes[3] >= 0) 
                final_target_pos_z_+= target_alt_interval_;
              else 
                final_target_pos_z_-= target_alt_interval_;
              ROS_INFO("Thrust command");
            }
        }
      else
        {
          if(alt_control_flag_)
            {
              alt_control_flag_= false;
              final_target_pos_z_= estimator_->getStatePosZ();
              ROS_INFO("Fixed Alt command, targetPosz_is %f",final_target_pos_z_);
            }
        }

#if 0 //no yaw 90 control
      //yaw
      if(!yaw_control_flag_)
        {
          if(joy_msg->buttons[10] == 1 && getNaviCommand() == HOVER_COMMAND)
            {//CCW
              ROS_INFO("CCW, change to pos control mode");
              yaw_control_flag_ = true;
              xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
              //final_target_psi_ += 3.1415/2.0; 
              final_target_psi_ = estimator_->getStatePsiBoard() + M_PI/2.0; 
              if(final_target_psi_ > M_PI) final_target_psi_ -= 2 * M_PI;
              else if(final_target_psi_ < -M_PI) final_target_psi_ += 2 *M_PI;

              final_target_pos_x_= estimator_->getStatePosX();
              final_target_pos_y_= estimator_->getStatePosY();
            }
          if(joy_msg->buttons[11] == 1 && getNaviCommand() == HOVER_COMMAND)
            {//CW
              ROS_INFO("CW,, change to pos control mode");
              yaw_control_flag_ = true;
              xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;

              final_target_psi_ = estimator_->getStatePsiBoard() - 3.1415/2.0; 
              if(final_target_psi_ > M_PI) final_target_psi_ -= 2 * M_PI;
              else if(final_target_psi_ < -M_PI) final_target_psi_ += 2 *M_PI;

              final_target_pos_x_= estimator_->getStatePosX();
              final_target_pos_y_= estimator_->getStatePosY();
            }
        }
      if(yaw_control_flag_ && joy_msg->buttons[10] == 0 && joy_msg->buttons[11] == 0)
        {
          yaw_control_flag_ = false;
          ROS_INFO("stop yaw control");
        }
#endif 
      if(joy_msg->buttons[2] == 1 && getNaviCommand() == HOVER_COMMAND)
        {
          if(fabs(joy_msg->axes[2]) > 0.05)
            {
              final_target_psi_ = estimator_->getStatePsiBoard() + joy_msg->axes[2] * target_yaw_rate_;
              ROS_INFO("yaw control");
            }
          else
            final_target_psi_ = estimator_->getStatePsiBoard();
        }
    }
  else if(xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE || xy_control_mode_ == POS_LOCAL_BASED_CONTROL_MODE)
    {
      //start 
      if(joy_msg->buttons[3] == 1 && getNaviCommand() != START_COMMAND)
        {
          setNaviCommand(START_COMMAND);
          final_target_pos_x_= estimator_->getStatePosX();
          final_target_pos_y_= estimator_->getStatePosY();
          final_target_pos_z_= takeoff_height_;  // 0.55m
          final_target_psi_  = estimator_->getStatePsiBoard();
          ROS_INFO("Start command");
          return;
        }
      //halt
      if(joy_msg->buttons[0] == 1)
        {
          setNaviCommand(STOP_COMMAND);
          flight_mode_= RESET_MODE;

          setTargetPosX(estimator_->getStatePosX());
          setTargetPosY(estimator_->getStatePosY());
          setTargetPsi(estimator_->getStatePsiBoard());

          ROS_INFO("Halt command");
          return;
        }
      //takeoff
      if(joy_msg->buttons[7] == 1 && joy_msg->buttons[13] == 1)
        {
          if(getStartAble())  
            {
              setNaviCommand(TAKEOFF_COMMAND);
              ROS_INFO("Takeoff command");
            }
          else  stopFlight();

          return;
        }
      //landing
      if(joy_msg->buttons[5] == 1 && joy_msg->buttons[15] == 1)
        {
          xy_control_mode_ = VEL_LOCAL_BASED_CONTROL_MODE;
          setNaviCommand(LAND_COMMAND);
          //更新
          final_target_pos_x_= estimator_->getStatePosX();
          final_target_pos_y_= estimator_->getStatePosY();
          final_target_pos_z_= 0; 
          final_target_psi_  = estimator_->getStatePsiBoard();
          ROS_INFO("Land command");

          return;
        }

      if(getStartAble() && getFlightAble() && getNaviCommand() != LAND_COMMAND)  
        {//start &  takeoff & !land
          setNaviCommand(HOVER_COMMAND);
          //pitch
          final_target_vel_x_= joy_msg->axes[1] * fabs(joy_msg->axes[1]) * target_vel_rate_;
          //roll
          final_target_vel_y_= joy_msg->axes[0] * fabs(joy_msg->axes[0]) * target_vel_rate_;
          //throttle
          if(fabs(joy_msg->axes[3]) > 0.2)
            {
              alt_control_flag_ = true;
              if(joy_msg->axes[3] >= 0) 
                final_target_pos_z_+= target_alt_interval_;
              else 
                final_target_pos_z_-= target_alt_interval_;
                  
              ROS_INFO("Thrust command");
            }
          else
            {
              if(alt_control_flag_)
                {
                  alt_control_flag_= false;
                  final_target_pos_z_= estimator_->getStatePosZ();
                  ROS_INFO("Fixed Alt command, targetPosz_is %f",final_target_pos_z_);
                }
            }
          if(final_target_pos_z_< 0.35) final_target_pos_z_= 0.35; // shuisei 
          if(final_target_pos_z_> 3) final_target_pos_z_= 3;
          //yaw 1
          final_target_psi_ = joy_msg->axes[2] * target_yaw_rate_;
          //yaw 2
          if(joy_msg->buttons[10] == 1)
            {
              ROS_INFO("CCW");
              final_target_psi_ = target_yaw_rate_;
            }
          if(joy_msg->buttons[11] == 1)
            {
              ROS_INFO("CW");
              final_target_psi_ = - target_yaw_rate_;
            }

        }
    }
}


void TeleopNavigator::targetValueCorrection()
{

  //no keystone correction
  current_target_pos_x_  = final_target_pos_x_;
  current_target_pos_y_= final_target_pos_y_;
  current_target_pos_z_  = final_target_pos_z_;
  current_target_vel_z_  = final_target_vel_z_;
  current_target_theta_ = final_target_theta_;
  current_target_vel_theta_ = final_target_vel_theta_;

  current_target_phy_ = final_target_phy_;
  current_target_vel_phy_ = final_target_vel_phy_;
  current_target_psi_ = final_target_psi_;
  current_target_vel_psi_ = final_target_vel_psi_;


#if 1 //keystone
  // for pitch and roll velocity control
  //pitch
  if(final_target_vel_x_ - current_target_vel_x_ > target_pitch_roll_interval_)
    current_target_vel_x_ += target_pitch_roll_interval_;
  else if (final_target_vel_x_ - current_target_vel_x_ < - target_pitch_roll_interval_)
    current_target_vel_x_ -= target_pitch_roll_interval_;
  else
    current_target_vel_x_ = final_target_vel_x_;
  //roll
  if(final_target_vel_y_ - current_target_vel_y_ > target_pitch_roll_interval_)
    current_target_vel_y_ += target_pitch_roll_interval_;
  else if (final_target_vel_y_ - current_target_vel_y_ < - target_pitch_roll_interval_)
    current_target_vel_y_ -= target_pitch_roll_interval_;
  else
    current_target_vel_y_ = final_target_vel_y_;
#else
  current_target_vel_x_ = final_target_vel_x_;
  current_target_vel_y_ = final_target_vel_y_;
#endif 

}


void TeleopNavigator::sendRcCmd()
{
  if(getNaviCommand() == START_COMMAND)
    { 
     ROS_INFO("START_COMMAND");
     std_msgs::UInt16 start_cmd;
     start_cmd.data = ARM_ON_CMD;
     msp_cmd_pub_.publish(start_cmd); 
    }
  else if(getNaviCommand() == STOP_COMMAND)
    { 
      std_msgs::UInt16 stop_cmd;
      stop_cmd.data = ARM_OFF_CMD;
      msp_cmd_pub_.publish(stop_cmd);
    }
  else if(getNaviCommand() == TAKEOFF_COMMAND ||
          getNaviCommand() == LAND_COMMAND ||
          getNaviCommand() == HOVER_COMMAND)
    {
      aerial_robot_msgs::RcData rc_data;
      rc_data.roll  =  flight_ctrl_input_->getRollValue();
      rc_data.pitch =  flight_ctrl_input_->getPitchValue();
      rc_data.yaw   =  flight_ctrl_input_->getYawValue();
      //rc_data.throttle = flight_ctrl_input_->getThrottleValue() - 1520; //MIDRC
      rc_data.throttle = flight_ctrl_input_->getThrottleValue() ;
      rc_cmd_pub_.publish(rc_data);

    }
  else
    {
      //ROS_ERROR("ERROR PISITION COMMAND, CAN NOT BE SEND TO Quadcopter");
    }
}

void TeleopNavigator::teleopNavigation()
{
  static int convergence_cnt = 0; 
  static int clock_cnt = 0; //mainly for velocity control takeoff

  //throwing mode
  if(use_throwing_mode_) throwingModeNavi();

  //keystron correction / target value process
  targetValueCorrection();

  if(getNaviCommand() == START_COMMAND)
    { //takeoff phase
      flight_mode_= NO_CONTROL_MODE;
    }
  else if(getNaviCommand() == TAKEOFF_COMMAND)
    {	//Take OFF Phase

      flight_mode_= TAKEOFF_MODE;

      //TODO convergenceFunction();
      if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE)
        {
          if (fabs(getTargetPosZ() - estimator_->getStatePosZ()) < POS_Z_THRE &&
              fabs(getTargetPosX() - estimator_->getStatePosX()) < POS_X_THRE &&
              fabs(getTargetPosY() - estimator_->getStatePosY()) < POS_Y_THRE)
            convergence_cnt++;
        }
      else if(xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE)
        {
          //TODO => check same as pos_world_based_control_mode
          if (fabs(getTargetPosZ() - estimator_->getStatePosZ()) < POS_Z_THRE)
              convergence_cnt++;


        }

      if (convergence_cnt > ctrl_loop_rate_) 
	{ //*** 安定収束した 20 ~ 40

          if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE)
            {
              convergence_cnt = 0;
              setNaviCommand(HOVER_COMMAND); 
              startFlight();
              ROS_WARN(" x/y pos stable hovering POS_WORLD_BASED_CONTROL_MODE");
            }
          else if(xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE)
            {
              if(xy_vel_mode_pos_ctrl_takeoff_)
                {
                  clock_cnt++;
                  ROS_WARN(" stable, clock_cnt: %d ", clock_cnt);
                  if(clock_cnt > (ctrl_loop_rate_ * TAKEOFF_COUNT))
                    {
                      clock_cnt = 0;
                      convergence_cnt = 0;
                      setNaviCommand(HOVER_COMMAND); 
                      startFlight();
                      ROS_WARN(" x/y pos stable hovering VEL_LOCAL_BASED_CONTROL_MODE");
                    }
                }
              else
                {
                  convergence_cnt = 0;
                  setNaviCommand(HOVER_COMMAND); 
                  startFlight();
                  ROS_WARN(" no x/y pos stable hovering ");
                }
            }
	}
    }
  else if(getNaviCommand() == LAND_COMMAND)
    {
      ROS_WARN(" land command");
      if (!getFlightAble()  && getStartAble()) 
	{
          ROS_ERROR(" land land mode mode");
	  setNaviCommand(STOP_COMMAND);
	  flight_mode_= RESET_MODE;

          setTargetPosX(estimator_->getStatePosX());
          setTargetPosY(estimator_->getStatePosY());
          setTargetPsi(estimator_->getStatePsiBoard());

          motor_stop_flag_ = false;
          resetFreeFallFlag();
          //bad
          estimator_->setRocketStartFlag();

          if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
	}

      else if (getFlightAble()  && getStartAble())
	{
	  flight_mode_= LAND_MODE; //--> for control

          if(use_free_fall_)
            {
              if(free_fall_thre_ > estimator_->getStatePosZ())
                {
                  ROS_WARN(" free fall mode");
                  free_fall_flag_ = true;
                }
              if(motor_stop_flag_)
                {
                  ROS_ERROR(" stop motor arming");
                  convergence_cnt = 0;
                  stopFlight();
                }
            }
          else
            { // do not use free fall
              if (fabs(getTargetPosZ() - estimator_->getStatePosZ()) < POS_Z_THRE)
                convergence_cnt++;
              if (convergence_cnt > ctrl_loop_rate_) 
                { 
                  ROS_WARN("  no free fall mode");
                  convergence_cnt = 0;
                  stopFlight();
                }
            }
	} 
      else if (!getStartAble())
	{
	  flight_mode_= NO_CONTROL_MODE;
	  setNaviCommand(IDLE_COMMAND);


          estimator_->setRocketStartFlag();
          resetFreeFallFlag();

	}
    }
  else if(getNaviCommand() == HOVER_COMMAND)
    {
      flight_mode_= FLIGHT_MODE;
    }
  else if(getNaviCommand() == IDLE_COMMAND)
    {
      flight_mode_= NO_CONTROL_MODE;
    }
  else if(getNaviCommand() == STOP_COMMAND)
    {
      flight_mode_= NO_CONTROL_MODE;
    }
  else
    {
      flight_mode_= NO_CONTROL_MODE;
      ROS_WARN("ERROR COMMAND, CAN NOT BE SEND TO AERIAL ROBOT");
    }
}


void TeleopNavigator::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nh.getNamespace();

  //*** teleop navigation
  if (!nh.getParam ("takeoff_height", takeoff_height_))
    takeoff_height_ = 0;
  printf("%s: takeoff_height_ is %.3f\n", ns.c_str(), takeoff_height_);

  if (!nh.getParam ("even_move_distance", even_move_distance_))
    even_move_distance_ = 0;
  printf("%s: even_move_distance_ is %.3f\n", ns.c_str(), even_move_distance_);

  if (!nh.getParam ("up_down_distance", up_down_distance_))
    up_down_distance_ = 0;
  printf("%s: up_down_distance_ is %.3f\n", ns.c_str(), up_down_distance_);

  if (!nh.getParam ("forward_backward_distance", forward_backward_distance_))
    forward_backward_distance_ = 0;
  printf("%s: forward_backward_distance_ is %.3f\n", ns.c_str(), forward_backward_distance_);

  if (!nh.getParam ("left_right_distance", left_right_distance_))
    left_right_distance_ = 0;
  printf("%s: left_right_distance_ is %.3f\n", ns.c_str(), left_right_distance_);

  if (!nh.getParam ("target_vel_rate", target_vel_rate_))
    target_vel_rate_ = 0;
  printf("%s: target_vel_rate_ is %.3f\n", ns.c_str(), target_vel_rate_);

  if (!nh.getParam ("target_pitch_roll_interval", target_pitch_roll_interval_))
    target_pitch_roll_interval_ = 0;
  printf("%s: target_pitch_roll_interval_ is %.3f\n", ns.c_str(), target_pitch_roll_interval_);

  if (!nh.getParam ("target_alt_interval", target_alt_interval_))
    target_alt_interval_ = 0;
  printf("%s: target_alt_interval_ is %.3f\n", ns.c_str(), target_alt_interval_);

  if (!nh.getParam ("target_yaw_rate", target_yaw_rate_))
    target_yaw_rate_ = 0;
  printf("%s: target_yaw_rate_ is %.3f\n", ns.c_str(), target_yaw_rate_);

  if (!nh.getParam ("navi_frame_int", navi_frame_int_))
    navi_frame_int_ = 0;
  printf("%s: navi_frame_int_ is %d\n", ns.c_str(), navi_frame_int_);
  navi_frame_ = navi_frame_int_;

  //hidden variable
  if (!nh.getParam ("cmd_vel_lev2_gain", cmd_vel_lev2_gain_))
    cmd_vel_lev2_gain_ = 1.0;
  printf("%s: cmd_vel_lev2_gain_ is %.3f\n", ns.c_str(), cmd_vel_lev2_gain_);
}

#include <jsk_quadcopter/flight_navigation.h>

Navigator::Navigator(ros::NodeHandle nh, ros::NodeHandle nh_private, int ctrl_loop_rate)
  : navigatorNodeHandle_(nh, "navigator"),
    navigatorNodeHandlePrivate_(nh_private, "navigator"),
    ctrlLoopRate(ctrl_loop_rate)
{
  tfB_ =  new tf::TransformBroadcaster();


  finalTargetPosX = 0;
  finalTargetVelX = 0;
  finalTargetPosY = 0;
  finalTargetVelY = 0;
  finalTargetPosZ = 0;
  finalTargetVelZ = 0;
  finalTargetTheta = 0;
  finalTargetVelTheta = 0;
  finalTargetPhy = 0;
  finalTargetVelPhy = 0;
  finalTargetPsi = 0;
  finalTargetVelPsi = 0;

  //current target value
  currentTargetPosX = 0;
  currentTargetVelX = 0;
  currentTargetPosY = 0;
  currentTargetVelY = 0;
  currentTargetPosZ = 0;
  currentTargetVelZ = 0;
  currentTargetTheta = 0;
  currentTargetVelTheta = 0;
  currentTargetPhy = 0;
  currentTargetVelPhy = 0;
  currentTargetPsi = 0;
  currentTargetVelPsi = 0;

  stopNavigation();
  stopFlight();
  setNaviCommand( IDLE_COMMAND );

}

Navigator::~Navigator()
{
  printf(" deleted navigator!\n");
  delete tfB_;
}



//*** startAble 
bool Navigator::getStartAble()
{
  return startAble;
}
void Navigator::startNavigation()
{
  startAble = true;
}
void Navigator::stopNavigation()
{
  startAble = false;
}
//*** flightAble
bool Navigator::getFlightAble()
{
  return flightAble;
}
void Navigator::startFlight()
{
  flightAble = true;
}
void Navigator::stopFlight()
{
  flightAble = false;
}
//*** command type
uint8_t Navigator::getNaviCommand()
{
  return naviCommand;
}
void Navigator::setNaviCommand(const uint8_t  command)
{
  naviCommand = command;
}

void Navigator::tfPublish()
{
  //TODO mutex
  tf::Transform map_to_target_;
  tf::Quaternion tmp_;
  
  map_to_target_.setOrigin(tf::Vector3(currentTargetPosX, currentTargetPosY, currentTargetPosZ));
  tmp_.setRPY(0.0, 0.0, currentTargetPsi); map_to_target_.setRotation(tmp_);
  ros::Time tm = ros::Time::now();
  tfB_->sendTransform(tf::StampedTransform(map_to_target_, tm, mapFrame_, targetFrame_));

}

float Navigator::getTargetPosX()
{
  // if(useOuterTargetPose & X_AXIS)
  //   return outerTargetPosX;
  // else
  //   return currentTargetPosX;
  return currentTargetPosX;
}
void Navigator::setTargetPosX(float value)
{
  // if(useOuterTargetPose & X_AXIS)
  //   outerTargetPosX = value;
  // else
  //   finalTargetPosX = value;
  finalTargetPosX = value;
}
void Navigator::addTargetPosX(float value)
{
  // if(useOuterTargetPose & X_AXIS)
  //   outerTargetPosX += value;
  // else
  //   finalTargetPosX += value;
  finalTargetPosX += value;
}
float Navigator::getTargetVelX()
{
  // if(useOuterTargetVel & X_AXIS)
  //   return  outerTargetVelX;
  // else
  //   return currentTargetVelX;
  return currentTargetVelX;
}
void Navigator::setTargetVelX(float value)
{
  // if(useOuterTargetVel & X_AXIS)
  //   outerTargetVelX = value;
  // else
  //   finalTargetVelX= value;
  finalTargetVelX= value;
}

float Navigator::getTargetPosY()
{
  // if(useOuterTargetPose & Y_AXIS)
  //   return outerTargetPosY;
  // else
  //   return currentTargetPosY;
  return currentTargetPosY;
}
void Navigator::setTargetPosY(float value)
{
  // if(useOuterTargetPose & Y_AXIS)
  //    outerTargetPosY = value;
  // else
  //    finalTargetPosY = value;
  finalTargetPosY = value;
}
void Navigator::addTargetPosY(float value)
{
  // if(useOuterTargetPose & Y_AXIS)
  //    outerTargetPosY += value;
  // else
  //    finalTargetPosY += value;
  finalTargetPosY += value;
}
float Navigator::getTargetVelY()
{
  // if(useOuterTargetVel & Y_AXIS)
  //   return outerTargetVelY;
  // else
  //   return currentTargetVelY;
  return currentTargetVelY;
}
void Navigator::setTargetVelY(float value)
{
  // if(useOuterTargetVel & Y_AXIS)
  //   outerTargetVelY = value;
  // else
  //   finalTargetVelY = value;
  finalTargetVelY = value;
}
float Navigator::getTargetPosZ()
{
  // if(useOuterTargetPose & Z_AXIS)
  //   return outerTargetPosZ;
  // else
  //   return currentTargetPosZ;
  return currentTargetPosZ;
}
void Navigator::setTargetPosZ(float value)
{
  // if(useOuterTargetPose & Z_AXIS)
  //   outerTargetPosZ = value;
  // else
  //   finalTargetPosZ = value;
  finalTargetPosZ = value;
}
void Navigator::addTargetPosZ(float value)
{
  // if(useOuterTargetPose & Z_AXIS)
  //   outerTargetPosZ += value;
  // else
  //   finalTargetPosZ += value;
  finalTargetPosZ += value;
}
float Navigator::getTargetVelZ()
{
  // if(useOuterTargetVel & Z_AXIS)
  //   return outerTargetVelZ;
  // else
  //   return currentTargetVelZ;
  return currentTargetVelZ;
}
void Navigator::setTargetVelZ(float value)
{
  // if(useOuterTargetVel & Z_AXIS)
  //   outerTargetVelZ = value;
  // else
  //   finalTargetVelZ = value;
  finalTargetVelZ = value;
}

float Navigator::getTargetTheta()
{
  // if(useOuterTargetPose & PITCH_AXIS)
  //   return outerTargetTheta;
  // else
  //   return currentTargetTheta;
  return currentTargetTheta;
}
void Navigator::setTargetTheta(float value)
{
  // if(useOuterTargetPose & PITCH_AXIS)
  //   outerTargetTheta = value;
  // else
  //   finalTargetTheta = value;
  finalTargetTheta = value;
}
float Navigator::getTargetVelTheta()
{
  //  if(useOuterTargetVel & PITCH_AXIS)
  //     return outerTargetVelTheta;
  //   else
  //     return currentTargetVelTheta;
  return currentTargetVelTheta;
 }
void Navigator::setTargetVelTheta(float value)
{
  // if(useOuterTargetVel & PITCH_AXIS)
  //   outerTargetVelTheta = value;
  // else
  //   finalTargetVelTheta = value;
  finalTargetVelTheta = value;
}
float Navigator::getTargetPhy()
{
  // if(useOuterTargetPose & ROLL_AXIS)
  //   return outerTargetPhy;
  // else
  //   return currentTargetPhy;
  return currentTargetPhy;
}
void Navigator::setTargetPhy(float value)
{
  // if(useOuterTargetPose & ROLL_AXIS)
  //   outerTargetPhy = value;
  // else
  //   finalTargetPhy = value;
  finalTargetPhy = value;
}
float Navigator::getTargetVelPhy()
{
  // if(useOuterTargetVel & ROLL_AXIS)
  //   return outerTargetVelPhy;
  // else
  //   return currentTargetVelPhy;
  return currentTargetVelPhy;
}
void Navigator::setTargetVelPhy(float value)
{
  // if(useOuterTargetVel & ROLL_AXIS)
  //   outerTargetVelPhy = value;
  // else
  //   finalTargetVelPhy = value;
  finalTargetVelPhy = value;
}
float Navigator::getTargetPsi()
{
  // if(useOuterTargetPose & YAW_AXIS)
  //   return outerTargetPsi;
  // else
  //   return currentTargetPsi;
  return currentTargetPsi;
}
void Navigator::setTargetPsi(float value)
{
  // if(useOuterTargetPose & YAW_AXIS)
  //   outerTargetPsi = value;
  // else
  //   finalTargetPsi = value;
  finalTargetPsi = value;
}
float Navigator::getTargetVelPsi()
{
  // if(useOuterTargetVel & YAW_AXIS)
  //   return outerTargetVelPsi;
  // else
  //   return currentTargetVelPsi;
  return currentTargetVelPsi;
}
void Navigator::setTargetVelPsi(float value)
{
  // if(useOuterTargetVel & YAW_AXIS)
  //   outerTargetVelPsi = value;
  // else
  //   finalTargetVelPsi = value;
  finalTargetVelPsi = value;
}


uint8_t Navigator::getFlightMode()
{
  return 0;
}

void Navigator::setXyControlMode(uint8_t mode)
{
  return;
}

uint8_t Navigator::getXyControlMode()
{
  return 0;
}

bool Navigator::getMotorStopFlag()
{
  return false;
}

void Navigator::setMotorStopFlag(bool motor_stop_flag)
{
  //do nothing
}

bool Navigator::getFreeFallFlag()
{
  return false;
}

void Navigator::resetFreeFallFlag()
{
  //do nothing
}

//not good
uint8_t Navigator::getThrowingMode()
{
  return false;
}

bool Navigator::getXyVelModePosCtrlTakeoff()
{
  return false;
}


// void Navigator::setOuterTargetPoseFlag(uint8_t axis)
// {
//   if(axis == 0)
//     useOuterTargetPose = 0;
//   else
//     useOuterTargetPose |= axis;
// }

// void Navigator::setOuterTargetVelFlag(uint8_t axis)
// {
//   if(axis == 0)
//     useOuterTargetVel = 0;
//   else
//     useOuterTargetVel |= axis;
// }

TeleopNavigator::TeleopNavigator(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 Estimator* estimator, int ctrl_loop_rate)
  :Navigator(nh, nh_private, ctrl_loop_rate)
{
  rosParamInit();

  //base navigation mode init
  flightMode = NO_CONTROL_MODE;

  //free fall mode init
  freeFallFlag = false;
  motorStopFlag =false;


  //joystick init
  imageSendingFlag = false; //deprecated
  deploymentFlag = false;   //deprecated
  dropFlag       = false;
  activateFlag   = false;
  velControlFlag = false;
  posControlFlag = false;
  altControlFlag = false;
  yawControlFlag = false;

  //throwing mode init
  throwingMode = NO_CONTROL_MODE;

  armingAckSub_ = navigatorNodeHandle_.subscribe<std_msgs::Int8>("kduino/arming_ack", 1, &TeleopNavigator::armingAckCallback, this, ros::TransportHints().tcpNoDelay());
  takeoffSub_ = navigatorNodeHandle_.subscribe<std_msgs::Empty>("teleop_command/takeoff", 1, &TeleopNavigator::takeoffCallback, this, ros::TransportHints().tcpNoDelay());
  haltSub_ = navigatorNodeHandle_.subscribe<std_msgs::Empty>("teleop_command/halt", 1, &TeleopNavigator::haltCallback, this, ros::TransportHints().tcpNoDelay());
  landSub_ = navigatorNodeHandle_.subscribe<std_msgs::Empty>("teleop_command/land", 1, boost::bind(&TeleopNavigator::landCallback, this, _1, estimator));
  startSub_ = navigatorNodeHandle_.subscribe<std_msgs::Empty>("teleop_command/start", 1,boost::bind(&TeleopNavigator::startCallback, this, _1, estimator));
  rollSub_ = navigatorNodeHandle_.subscribe<std_msgs::Int8>("teleop_command/roll", 1,boost::bind(&TeleopNavigator::rollCallback, this, _1, estimator));
  pitchSub_ = navigatorNodeHandle_.subscribe<std_msgs::Int8>("teleop_command/pitch", 1,boost::bind(&TeleopNavigator::pitchCallback, this, _1, estimator));
  yawSub_ = navigatorNodeHandle_.subscribe<std_msgs::Int8>("teleop_command/yaw", 1, boost::bind(&TeleopNavigator::yawCallback, this, _1, estimator));
  throttleSub_ = navigatorNodeHandle_.subscribe<std_msgs::Int8>("teleop_command/throttle", 1, boost::bind(&TeleopNavigator::throttleCallback, this, _1, estimator));
  ctrlModeSub_ = navigatorNodeHandle_.subscribe<std_msgs::Int8>("teleop_command/ctrl_mode", 1, &TeleopNavigator::xyControlModeCallback, this, ros::TransportHints().tcpNoDelay());

  joyStickSub_ = navigatorNodeHandle_.subscribe<sensor_msgs::Joy>("joy_stick_command", 1, boost::bind(&TeleopNavigator::joyStickControl, this, _1, estimator));

  rcCmdPub_ = navigatorNodeHandle_.advertise<jsk_quadcopter_common::RcData>("kduino/rc_cmd", 10); 

  mspCmdPub_ = navigatorNodeHandle_.advertise<std_msgs::UInt16>("kduino/msp_cmd", 10); 

  activatePub_= navigatorNodeHandle_.advertise<std_msgs::Int8>("/force_drop_activate", 2);

  stateShiftPub_= navigatorNodeHandle_.advertise<std_msgs::Int8>("/force_state_shift", 2);

  //temporarily
  jointsCtrlPub_= navigatorNodeHandle_.advertise<std_msgs::Int8>("/teleop_command/joints_ctrl", 2);


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
      if(xyControlMode_ == VEL_WORLD_BASED_CONTROL_MODE) xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;
      setNaviCommand(TAKEOFF_COMMAND);
      ROS_INFO("Takeoff command");
    }
  else  stopFlight();
}

void TeleopNavigator::startCallback(const std_msgs::EmptyConstPtr & msg, Estimator* estimator)
  {//すべて軸に対して、初期化
    setNaviCommand(START_COMMAND);
    finalTargetPosX = estimator->getStatePosX();
    finalTargetPosY = estimator->getStatePosY();
    finalTargetPosZ = takeoffHeight_;  // 0.55m

    finalTargetPsi  = estimator->getStatePsiBody();
    ROS_INFO("Start command");
  }

void TeleopNavigator::landCallback(const std_msgs::EmptyConstPtr & msg, Estimator* estimator)
  {
    if(xyControlMode_ == VEL_WORLD_BASED_CONTROL_MODE) xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;
    setNaviCommand(LAND_COMMAND);
    //更新
    finalTargetPosX = estimator->getStatePosX();
    finalTargetPosY = estimator->getStatePosY();
    finalTargetPosZ = 0; 
    finalTargetPsi  = estimator->getStatePsiBody();
    ROS_INFO("Land command");
  }

void TeleopNavigator::haltCallback(const std_msgs::EmptyConstPtr & msg)
  {
    if(xyControlMode_ == VEL_WORLD_BASED_CONTROL_MODE) xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;
    setNaviCommand(STOP_COMMAND);
    flightMode = RESET_MODE;
    ROS_INFO("Halt command");
  }

void TeleopNavigator::rollCallback(const std_msgs::Int8ConstPtr & msg, Estimator* estimator)
  {// + : right ; - : left
    if(getStartAble() && getFlightAble())
      {
	setNaviCommand(HOVER_COMMAND);
	if(msg->data == 1){
	  ROS_INFO("RIGHT");
	  if(naviFrame == MAP_FRAME)
	    finalTargetPosY -= evenMoveDistance_; //0.2m on now state
	  else if(naviFrame == BODY_FRAME)
	    {
	    if(fabs(cos(estimator->getStatePsiBody())) > 
	       fabs(sin(estimator->getStatePsiBody())))
	      {
		finalTargetPosY -= leftRightDistance_ *
		  (fabs(cos(estimator->getStatePsiBody()))/cos(estimator->getStatePsiBody()));
		finalTargetPosX = estimator->getStatePosX();
	      }
	    else if(fabs(cos(estimator->getStatePsiBody())) <
		    fabs(sin(estimator->getStatePsiBody())))
	      {
		finalTargetPosX += leftRightDistance_ *
		  (fabs(sin(estimator->getStatePsiBody()))/sin(estimator->getStatePsiBody()));
		finalTargetPosY = estimator->getStatePosY();
	      
	      }
	    }
	}else{
	  ROS_INFO("LEFT");
	  if(naviFrame == MAP_FRAME)
	    finalTargetPosY += evenMoveDistance_; //0.2m on now state
	  else if(naviFrame == BODY_FRAME)
	    {
	      if(fabs(cos(estimator->getStatePsiBody())) >
		 fabs(sin(estimator->getStatePsiBody())))
		{
		  finalTargetPosY += leftRightDistance_ * 
		    (fabs(cos(estimator->getStatePsiBody())) / cos(estimator->getStatePsiBody()));
		  finalTargetPosX = estimator->getStatePosX();
		}
	      else if(fabs(cos(estimator->getStatePsiBody())) <
		      fabs(sin(estimator->getStatePsiBody())))
		{
		  finalTargetPosX -= leftRightDistance_ * 
		    (fabs(sin(estimator->getStatePsiBody())) / sin(estimator->getStatePsiBody()));
		  finalTargetPosY = estimator->getStatePosY();
		}
	    }
	}
      }
  }

void TeleopNavigator::pitchCallback(const std_msgs::Int8ConstPtr & msg, Estimator* estimator)
{// + : backward ; - : forward
  if(getStartAble() && getFlightAble())  
    {
      setNaviCommand(HOVER_COMMAND);
      if(msg->data == 1)
	{
	  ROS_INFO("FORWARD");
	  if(naviFrame == MAP_FRAME) finalTargetPosX += evenMoveDistance_; //0.2m
	  else if(naviFrame == BODY_FRAME)
	    {
	      if(fabs(cos(estimator->getStatePsiBody())) >
		 fabs(sin(estimator->getStatePsiBody()))){
		finalTargetPosX += forwardBackwardDistance_ * 
		  (fabs(cos(estimator->getStatePsiBody())) / cos(estimator->getStatePsiBody()));
		//finalTargetPosY = estimator->getStatePosY();
	      }
	      else if(fabs(cos(estimator->getStatePsiBody())) <
		      fabs(sin(estimator->getStatePsiBody()))){
		//finalTargetPosX = estimator->getStatePosX();
		finalTargetPosY += forwardBackwardDistance_ * 
		  (fabs(sin(estimator->getStatePsiBody())) / sin(estimator->getStatePsiBody()));
	      }
	    }
	}
      else
	{
	  ROS_INFO("BACKFORWARD");
	  if(naviFrame == MAP_FRAME) finalTargetPosX -= evenMoveDistance_; //0.2m
	  else if(naviFrame == BODY_FRAME)
	    {
	      finalTargetPosX = estimator->getStatePosX();
	      finalTargetPosY = estimator->getStatePosY();
	    }
	}
    }
}


void TeleopNavigator::yawCallback(const std_msgs::Int8ConstPtr & msg, Estimator* estimator)
{
  if(getStartAble() && getFlightAble())  
    { 
      setNaviCommand(HOVER_COMMAND);

      if(naviFrame == BODY_FRAME)
	{
	  finalTargetPosX = estimator->getStatePosX();
	  finalTargetPosY = estimator->getStatePosY();
	}


      if(msg->data == -1){
        finalTargetPsi = estimator->getStatePsiBody() - 3.1415/2.0; 
	ROS_INFO("CW");
      }else if(msg->data == 1){
        finalTargetPsi = estimator->getStatePsiBody() + 3.1415/2.0; 
	ROS_INFO("CCW");
      }

      if(msg->data == -2){
	//finalTargetPsi = estimator->getStatePsi() - 0.1; //5 degree 
        finalTargetPsi -= 0.1; //bad method
	ROS_INFO("delta:CW");
      }else if(msg->data == 2){
	//finalTargetPsi = estimator->getStatePsi() + 0.1;  //5 degree
        finalTargetPsi += 0.1; //bad method
	ROS_INFO("delta:CCW");
      }
      else
        {
          //nothing
        }
      //yaw_finalTargetの補正
      if(finalTargetPsi > M_PI) finalTargetPsi -= 2 * M_PI;
      else if(finalTargetPsi < -M_PI) finalTargetPsi += 2 *M_PI;
    }
  //ROS_INFO("Yaw command");
}

void TeleopNavigator::throttleCallback(const std_msgs::Int8ConstPtr & msg, Estimator* estimator)
{
  if(getStartAble() && getFlightAble())  
    { 
      setNaviCommand(HOVER_COMMAND);

      if(msg->data == 1){
	finalTargetPosZ += upDownDistance_; 
	ROS_INFO("UP");
      }else{
	finalTargetPosZ -= upDownDistance_; 
	ROS_INFO("DOWN");
      }
    }
  //ROS_INFO("Thrust command");
}

void TeleopNavigator::xyControlModeCallback(const std_msgs::Int8ConstPtr & msg)
{
  //if(1)
    if(getStartAble())
    {
      if(msg->data == 0)
        {
          xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;
          ROS_INFO("x/y position control mode");
        }
      if(msg->data == 1)
        {
          xyControlMode_ = VEL_LOCAL_BASED_CONTROL_MODE;
          ROS_INFO("x/y velocity control mode");
        }
    }
}


void TeleopNavigator::joyStickControl(const sensor_msgs::JoyConstPtr joy_msg, Estimator* estimator)
{
  if(xyControlMode_ == POS_WORLD_BASED_CONTROL_MODE || xyControlMode_ == VEL_WORLD_BASED_CONTROL_MODE)
    {
      //start 
      if(joy_msg->buttons[3] == 1 && getNaviCommand() != START_COMMAND)
        {
          setNaviCommand(START_COMMAND);
          finalTargetPosX = estimator->getStatePosX();
          finalTargetPosY = estimator->getStatePosY();
          finalTargetPosZ = takeoffHeight_;  // 0.55m
          finalTargetPsi  = estimator->getStatePsiBody();
          ROS_INFO("Start command");
          return;
        }
      //halt
      if(joy_msg->buttons[0] == 1)
        {
          setNaviCommand(STOP_COMMAND);
          flightMode = RESET_MODE;
          if(xyControlMode_ == VEL_WORLD_BASED_CONTROL_MODE) xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;
          ROS_INFO("Halt command");
          return;
        }
      //takeoff
      if(joy_msg->buttons[7] == 1 && joy_msg->buttons[13] == 1)
        {

          if(getStartAble())  
            {
              if(xyControlMode_ == VEL_WORLD_BASED_CONTROL_MODE) xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;
              setNaviCommand(TAKEOFF_COMMAND);
              ROS_INFO("Takeoff command");
            }
          else  stopFlight();

          return;
        }
      //landing
      if(joy_msg->buttons[5] == 1 && joy_msg->buttons[15] == 1)
        {
          if(xyControlMode_ == VEL_WORLD_BASED_CONTROL_MODE) xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;

          setNaviCommand(LAND_COMMAND);
          //更新
          finalTargetPosX = estimator->getStatePosX();
          finalTargetPosY = estimator->getStatePosY();
          finalTargetPosZ = 0; 
          finalTargetPsi  = estimator->getStatePsiBody();
          ROS_INFO("Land command");

          return;
        }

      //change to vel control mode
      if(joy_msg->buttons[12] == 1 && !velControlFlag)
        {
          ROS_INFO("change to vel pos-based control");
          velControlFlag = true;
          xyControlMode_ = VEL_WORLD_BASED_CONTROL_MODE;
          finalTargetVelX = 0; currentTargetVelX = 0;
          finalTargetVelY = 0; currentTargetVelY = 0;
        }
      if(joy_msg->buttons[12] == 0 && velControlFlag)
        velControlFlag = false;

      //change to pos control mode
      if(joy_msg->buttons[14] == 1 && !posControlFlag)
        {
          ROS_INFO("change to pos control");
          posControlFlag = true;
          xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;
          finalTargetPosX = estimator->getStatePosX();
          finalTargetPosY = estimator->getStatePosY();
        }
      if(joy_msg->buttons[14] == 0 && posControlFlag)
          posControlFlag = false;

      //pitch && roll vel command for vel_mode
      if(xyControlMode_ == VEL_WORLD_BASED_CONTROL_MODE && getNaviCommand() == HOVER_COMMAND)
        {
          if(joy_msg->buttons[1] == 0)
            {//no push the left joysitck
              finalTargetVelX = joy_msg->axes[1] * fabs(joy_msg->axes[1]) * targetVelRate_;
              finalTargetVelY = joy_msg->axes[0] * fabs(joy_msg->axes[0]) * targetVelRate_;
            }
          if(joy_msg->buttons[1] == 1)
            {//push the left joysitck
              ROS_INFO("strong vel control");
              finalTargetVelX 
                = joy_msg->axes[1] * fabs(joy_msg->axes[1]) * targetVelRate_ * cmdVelLev2Gain_;
              finalTargetVelY 
                = joy_msg->axes[0] * fabs(joy_msg->axes[0]) * targetVelRate_ * cmdVelLev2Gain_;
            }
        }

      //throttle, TODO: not good
      if(fabs(joy_msg->axes[3]) > 0.2)
        {
          if(joy_msg->buttons[2] == 0 && getNaviCommand() == HOVER_COMMAND)
            {//push the right joysitck
              altControlFlag = true;
              if(joy_msg->axes[3] >= 0) 
                finalTargetPosZ += targetAltInterval_;
              else 
                finalTargetPosZ -= targetAltInterval_;
              ROS_INFO("Thrust command");
            }
        }
      else
        {
          if(altControlFlag)
            {
              altControlFlag= false;
              finalTargetPosZ = estimator->getStatePosZ();
              ROS_INFO("Fixed Alt command, targetPosZ is %f",finalTargetPosZ);
            }
        }

#if 0 //no yaw 90 control
      //yaw
      if(!yawControlFlag)
        {
          if(joy_msg->buttons[10] == 1 && getNaviCommand() == HOVER_COMMAND)
            {//CCW
              ROS_INFO("CCW, change to pos control mode");
              yawControlFlag = true;
              xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;
              //finalTargetPsi += 3.1415/2.0; 
              finalTargetPsi = estimator->getStatePsiBody() + 3.1415/2.0; 
              if(finalTargetPsi > M_PI) finalTargetPsi -= 2 * M_PI;
              else if(finalTargetPsi < -M_PI) finalTargetPsi += 2 *M_PI;

              finalTargetPosX = estimator->getStatePosX();
              finalTargetPosY = estimator->getStatePosY();
            }
          if(joy_msg->buttons[11] == 1 && getNaviCommand() == HOVER_COMMAND)
            {//CW
              ROS_INFO("CW,, change to pos control mode");
              yawControlFlag = true;
              xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;
              //finalTargetPsi -= 3.1415/2.0; 
              finalTargetPsi = estimator->getStatePsiBody() - 3.1415/2.0; 
              if(finalTargetPsi > M_PI) finalTargetPsi -= 2 * M_PI;
              else if(finalTargetPsi < -M_PI) finalTargetPsi += 2 *M_PI;

              finalTargetPosX = estimator->getStatePosX();
              finalTargetPosY = estimator->getStatePosY();
            }
        }
      if(yawControlFlag && joy_msg->buttons[10] == 0 && joy_msg->buttons[11] == 0)
        {
          yawControlFlag = false;
          ROS_INFO("stop yaw control");
        }
#endif 
      if(joy_msg->buttons[2] == 1 && getNaviCommand() == HOVER_COMMAND)
        {
          if(fabs(joy_msg->axes[2]) > 0.05)
            {
              finalTargetPsi = estimator->getStatePsiBody() + joy_msg->axes[2] * targetYawRate_;
              ROS_INFO("yaw control");
            }
          else
              finalTargetPsi = estimator->getStatePsiBody();
        }
    }
  else if(xyControlMode_ == VEL_LOCAL_BASED_CONTROL_MODE || xyControlMode_ == POS_LOCAL_BASED_CONTROL_MODE)
    {
      //start 
      if(joy_msg->buttons[3] == 1 && getNaviCommand() != START_COMMAND)
        {
          setNaviCommand(START_COMMAND);
          finalTargetPosX = estimator->getStatePosX();
          finalTargetPosY = estimator->getStatePosY();
          finalTargetPosZ = takeoffHeight_;  // 0.55m
          finalTargetPsi  = estimator->getStatePsiBody();
          ROS_INFO("Start command");
          return;
        }
      //halt
      if(joy_msg->buttons[0] == 1)
        {
          setNaviCommand(STOP_COMMAND);
          flightMode = RESET_MODE;
          ROS_INFO("Halt command");
          return;
        }
      //takeoff
      if(joy_msg->buttons[7] == 1 && joy_msg->buttons[13] == 1)
        {
          // debug    
          ROS_INFO("Takeoff1");

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
          xyControlMode_ = VEL_LOCAL_BASED_CONTROL_MODE;
          setNaviCommand(LAND_COMMAND);
          //更新
          finalTargetPosX = estimator->getStatePosX();
          finalTargetPosY = estimator->getStatePosY();
          finalTargetPosZ = 0; 
          finalTargetPsi  = estimator->getStatePsiBody();
          ROS_INFO("Land command");

          return;
        }

      //if(getNaviCommand() == HOVER_COMMAND)
      if(getStartAble() && getFlightAble() && getNaviCommand() != LAND_COMMAND)  
        {//start &  takeoff & !land
          setNaviCommand(HOVER_COMMAND);
          //pitch
          finalTargetVelX = joy_msg->axes[1] * fabs(joy_msg->axes[1]) * targetVelRate_;
          //roll
          finalTargetVelY = joy_msg->axes[0] * fabs(joy_msg->axes[0]) * targetVelRate_;
          //throttle
          if(fabs(joy_msg->axes[3]) > 0.2)
            {
              altControlFlag = true;
              if(joy_msg->axes[3] >= 0) 
                finalTargetPosZ += targetAltInterval_;
              else 
                  finalTargetPosZ -= targetAltInterval_;
                  
              ROS_INFO("Thrust command");
            }
          else
            {
              if(altControlFlag)
                {
                  altControlFlag= false;
                  finalTargetPosZ = estimator->getStatePosZ();
                  ROS_INFO("Fixed Alt command, targetPosZ is %f",finalTargetPosZ);
                }
            }
          if(finalTargetPosZ < 0.35) finalTargetPosZ = 0.35; // shuisei 
          if(finalTargetPosZ > 3) finalTargetPosZ = 3;
          //yaw 1
          finalTargetPsi = joy_msg->axes[2] * targetYawRate_;
          //debug
          if(fabs(joy_msg->axes[2]) > 0.001)
             ROS_INFO("targetPsi : %f", finalTargetPsi);
          //yaw 2
          if(joy_msg->buttons[10] == 1)
            {
              ROS_INFO("CCW");
              finalTargetPsi = targetYawRate_;
            }
          // else
          //     finalTargetPsi = 0;
          if(joy_msg->buttons[11] == 1)
            {
              ROS_INFO("CW");
              finalTargetPsi = - targetYawRate_;
            }
          // else
          //     targetPsi = 0;

        }
    }

#if 1 //Temporarily for hydra transform
  //+*+*+*+* 

  if(!activateFlag)
    {//joints angle ctrl
      if(joy_msg->buttons[8] == 1)
        {
          std_msgs::Int8 joints_ctrl_cmd;
          joints_ctrl_cmd.data = 7;
          jointsCtrlPub_.publish(joints_ctrl_cmd);
          activateFlag = true;
          ROS_INFO("to ku model");
        }
      if(joy_msg->buttons[10] == 1)
        {
          std_msgs::Int8 joints_ctrl_cmd;
          joints_ctrl_cmd.data = 8;
          jointsCtrlPub_.publish(joints_ctrl_cmd);
          activateFlag = true;
          ROS_INFO("to normal model");
        }
    }
  if(joy_msg->buttons[8] == 0 && joy_msg->buttons[10] == 0 && activateFlag)
    activateFlag = false;

  if(!dropFlag)
    {
      if(joy_msg->buttons[11] == 1)
        {
          dropFlag = true;
          std_msgs::UInt16 gainCmd;
          gainCmd.data = 163; //ROS_ROLL_PID_D_UP 
          mspCmdPub_.publish(gainCmd); 
          ROS_INFO("increase the roll d gain");
        }
      if(joy_msg->buttons[9] == 1)
        {
          dropFlag = true;
          std_msgs::UInt16 gainCmd;
          gainCmd.data = 164; //ROS_ROLL_PID_D_DOWN 
          mspCmdPub_.publish(gainCmd); 
          ROS_INFO("decrease the roll d gain");
        }
    }
  if(joy_msg->buttons[9] == 0 && joy_msg->buttons[11] == 0 && dropFlag)
    dropFlag = false;
#endif

#if 0 //SMAN
  //for router deployment
  //+*+*+ drop
  if(joy_msg->buttons[8] == 1 && !dropFlag)
    {
      dropFlag = true;
      std_msgs::UInt16 dropCmd;
      dropCmd.data = DROP_CMD;
      mspCmdPub_.publish(dropCmd); 
      ROS_INFO("drop new router");
    }
  if(joy_msg->buttons[8] == 0 && dropFlag)
    dropFlag = false;

  //+*+*+*+* activate
  if(joy_msg->buttons[9] == 1 && !activateFlag)
    {
      std_msgs::Int8 activate_cmd;
      activate_cmd.data = 0;
      activatePub_.publish(activate_cmd);
      activateFlag = true;
      ROS_INFO("activate new router");
    }
  if(joy_msg->buttons[9] == 0 && activateFlag)
    activateFlag = false;

  if(joy_msg->buttons[4] == 1)
    {
      std_msgs::Int8 state_shift_cmd;
      state_shift_cmd.data = 1;
      stateShiftPub_.publish(state_shift_cmd);
      ROS_INFO("shift to deployment state");
    }
  if(joy_msg->buttons[6] == 1)
    {
      std_msgs::Int8 state_shift_cmd;
      state_shift_cmd.data = 0;
      stateShiftPub_.publish(state_shift_cmd);
      ROS_INFO("shift to idle state");
    }
#endif 
}

uint8_t TeleopNavigator::getFlightMode()
{
  return flightMode;
}

uint8_t TeleopNavigator::getXyControlMode()
{
  return (uint8_t)xyControlMode_;
}

void TeleopNavigator::setXyControlMode(uint8_t mode)
{
  xyControlMode_ = mode;
}

bool TeleopNavigator::getMotorStopFlag()
{
  return motorStopFlag;
}

void TeleopNavigator::setMotorStopFlag(bool motor_stop_flag)
{
  motorStopFlag = motor_stop_flag;
}

bool TeleopNavigator::getFreeFallFlag()
{
  return freeFallFlag;
}

void TeleopNavigator::resetFreeFallFlag()
{
  freeFallFlag = false;
}


uint8_t TeleopNavigator::getThrowingMode()
{
  return throwingMode;
}

bool TeleopNavigator::getXyVelModePosCtrlTakeoff()
{
  return xyVelModePosCtrlTakeoff_;
}

//1 implemented in  a more higher rate loop 
//2 implement callback function
//3 now: a 40Hz function ( z_offset is not important for flight control)
void TeleopNavigator::throwingModeNavi(Estimator* estimator)
{
  static int ready_to_standby_cnt = 0;
  static int ready_to_arm_cnt = 0;
  static int ready_to_setalt_cnt = 0;
  static int ready_to_althold_cnt = 0;
  static int opticalflow_state = 0;

  float pos_z = estimator->getStatePosZ();
  float vel_z = estimator->getStateVelZ();
  float acc_z = estimator->getStateAccZb() - 9.8;
#if 1 //sonar sensor
  float vel_x = estimator->getStateVelXc();
  //float vel_y = estimator->getStateVelYc();
#else  //laser
  float vel_x = estimator->getStateVelX();
  //float vel_y = estimator->getStateVelY();
#endif
  float acc_x = estimator->getStateAccXb();
  float acc_y = estimator->getStateAccYb();

  static float prev_acc_x, prev_acc_z, prev_vel_z;

  if(throwingMode == NO_CONTROL_MODE)
    {
      //+*+*+* for alt control
      if(getNaviCommand() == IDLE_COMMAND)
        {
          if(pos_z > throwingModeStandbyAltThre_) // altitude thre + getNaviCommand() == IDLE_COMMAND
            ready_to_standby_cnt ++;
          else 
            ready_to_standby_cnt = 0;
          if(ready_to_standby_cnt > throwingModeShiftStepCntThre_ * 2)
            {
              throwingMode = THROWING_START_STANDBY;
              ROS_INFO("shift to THROWING_START_STANDBY");
            }
        }
    }

#if 1
  else if(throwingMode == THROWING_START_STANDBY)
    {
      //+*+* for alt control
      if(acc_x > throwingModeThrowAccXThre_ &&
         acc_z > throwingModeThrowAccZThre_) // altitude vel/pos/acc + x acc/vel + timer(e.g. 0.1s)
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

      if(ready_to_arm_cnt > throwingModeShiftStepCntThre_)
        {
          ready_to_arm_cnt = 0;
          //throwingMode = NO_CONTROL_MODE;
          throwingMode = THROWING_START_ARMINGON;
          setNaviCommand(START_COMMAND);
          startNavigation();
          ROS_ERROR("shift to arming mode");

          //+*+*+* for optical flow
#if 1 //no condition
         opticalflow_state = OPT_GOOD_TRACKING;
#else
          if(estimator->getStateVelXOpt() > 0)
            {
              ROS_WARN("GOOD TRACKING");
              opticalflow_state = OPT_GOOD_TRACKING;
            }
#endif
        }
    }
  else if(throwingMode == THROWING_START_ARMINGON)
    {
      ROS_INFO("arming mode");
#if 1
      //+*+* for alt control
      if(acc_z < throwingModeSetAltAccZThre_)  // -5
        {
          if(ready_to_setalt_cnt < throwingModeShiftStepCntThre_)
            {
              if(ready_to_setalt_cnt == 0)
                {
                  ready_to_setalt_cnt ++;
                  prev_acc_z = acc_z;
                }
              else
                {
                  if(acc_z < prev_acc_z)
                    ready_to_setalt_cnt ++;
                  else 
                    ready_to_setalt_cnt = 0;
                }
            }
        }
      else
        ready_to_setalt_cnt = 0;

      if(vel_z < throwingModeAltHoldVelZThre_)  //0.5
        {
          if(ready_to_setalt_cnt > throwingModeShiftStepCntThre_)
            {
              ROS_WARN("OK1");
              if(ready_to_althold_cnt == 0)
                {
                  ready_to_althold_cnt += throwingModeShiftStepCntThre_;
                  prev_vel_z = vel_z;
                  ROS_WARN("OK2");
                }
            }
        }
      else
        ready_to_althold_cnt = 0;

#else
      if(vel_z < 0) //todo: altitude vel/pos + etc
        {
          if(ready_to_althold_cnt == 0)
            {
              ready_to_althold_cnt ++;
              prev_vel_z = vel_z;
            }
          else
            {
              if(vel_z < prev_vel_z)
                ready_to_althold_cnt ++;
              else 
                ready_to_althold_cnt = 0;
            }
        }
      else
        ready_to_althold_cnt = 0;
#endif

      //+*+* for optflow
      if(opticalflow_state == OPT_GOOD_TRACKING
         && estimator->getStateVelXOpt() == 0)
        {
          opticalflow_state = OPT_ZERO_TRACKING;
          bool stop_flag = false;
          estimator->setKFMeaureFlag(estimator->X_AXIS, stop_flag);
          estimator->setKFMeaureFlag(estimator->Y_AXIS, stop_flag);
          ROS_WARN("ZERO TRACKING");
        }

      if(ready_to_setalt_cnt == throwingModeShiftStepCntThre_)
        {
          finalTargetPosZ = pos_z + 0.5 * vel_z * vel_z / 9.8 + 0.2;
          ready_to_setalt_cnt++;
          ROS_ERROR("set alt, %f", finalTargetPosZ);
        }
      if(ready_to_althold_cnt == throwingModeShiftStepCntThre_)
        {
          ready_to_setalt_cnt = 0;
          ready_to_althold_cnt = 0;
          throwingMode = THROWING_START_ALTHOLD;
          startFlight(); //not very good , temporarily
          setNaviCommand(HOVER_COMMAND); //special
          ROS_ERROR("shift to althold mode");
        }

    }
  else if(throwingMode == THROWING_START_ALTHOLD)
    {
      //+*+* for alt control
      setNaviCommand(HOVER_COMMAND); //special, escape from start res from quadcopter
      ROS_INFO("alt hold mode");
      //very simple way
      if(vel_x < 0) //todo: altitude pos + x vel + etc
        {
          ROS_INFO("takeoff mode");
          //setNaviCommand(TAKEOFF_COMMAND); //to get pos I term 
          //throwingMode = NO_CONTROL_MODE;
          throwingMode = FLIGHT_MODE; //to avoid loop
#if 1 //optical flow
          finalTargetPosX = estimator->getStatePosXc();
          finalTargetPosY = estimator->getStatePosYc();
#else  //laser
          finalTargetPosX = estimator->getStatePosX();
          finalTargetPosY = estimator->getStatePosY();
#endif
        }

      //*+*+ for optical flow
      if(opticalflow_state == OPT_ZERO_TRACKING && 
         acc_z > 0 && estimator->getStateVelXOpt() > 0.25)
        {
          opticalflow_state = OPT_RECOVER_TRACKING;
          bool start_flag = true;
          estimator->setKFMeaureFlag(estimator->X_AXIS, start_flag);
          estimator->setKFMeaureFlag(estimator->Y_AXIS, start_flag);
          ROS_WARN("RECOVER TRACKING");
        }

    }

#else //free fall

  else if(throwingMode == THROWING_START_STANDBY)
    {
      if(pos_z > 1.5)
        ready_to_arm_cnt ++;

      if(ready_to_arm_cnt > throwingModeShiftStepCntThre_)
        {
          //throwingMode = NO_CONTROL_MODE;
          throwingMode = THROWING_START_ARMINGON;
          setNaviCommand(START_COMMAND);
          startNavigation();
          ROS_ERROR("shift to arming mode");
        }
    }
  else if(throwingMode == THROWING_START_ARMINGON)
    {
      //ROS_WARN("the pos_z is %f, the acc_z is %f",pos_z,acc_z);
      if(acc_z < -4) 
        {
          //ROS_WARN("the pos_z for alhold is %f",pos_z);
          if(ready_to_althold_cnt == 0)
            {
              ready_to_althold_cnt ++;
              prev_acc_z = acc_z;
            }
          else
            {
              if(acc_z < prev_acc_z)
                ready_to_althold_cnt ++;
              else 
                ready_to_althold_cnt = 0;
            }
        }
      else
        ready_to_althold_cnt = 0;

      if(ready_to_althold_cnt > throwingModeShiftStepCntThre_)
        {
          throwingMode = THROWING_START_ALTHOLD;
          startFlight(); //not very good , temporarily
          setNaviCommand(HOVER_COMMAND); //special
          finalTargetPosZ = 1.2 ;
          ROS_ERROR("shift to althold mode, %f", finalTargetPosZ);
        }
    }
  else if(throwingMode == THROWING_START_ALTHOLD)
    {
      setNaviCommand(HOVER_COMMAND); //special, escape from start res from quadcopter
      ROS_INFO("alt hold mode");
      //very simple way
      if(vel_x < 0) //todo: altitude pos + x vel + etc
        {
          ROS_INFO("takeoff mode");
          //setNaviCommand(TAKEOFF_COMMAND); //to get pos I term 
          //throwingMode = NO_CONTROL_MODE;
#if 1 //optical flow
          finalTargetPosX = estimator->getStatePosXc();
          finalTargetPosY = estimator->getStatePosYc();
#else  //laser
          finalTargetPosX = estimator->getStatePosX();
          finalTargetPosY = estimator->getStatePosY();
#endif
        }
    }

#endif 

  else
    {
      //do nothing
    }
}

void TeleopNavigator::targetValueCorrection()
{

  //no keystone correction
  currentTargetPosX  = finalTargetPosX;
  currentTargetPosY  = finalTargetPosY;
  currentTargetPosZ  = finalTargetPosZ;
  currentTargetVelZ  = finalTargetVelZ;
  currentTargetTheta = finalTargetTheta;
  currentTargetVelTheta = finalTargetVelTheta;

  currentTargetPhy = finalTargetPhy;
  currentTargetVelPhy = finalTargetVelPhy;
  currentTargetPsi = finalTargetPsi;
  currentTargetVelPsi = finalTargetVelPsi;


#if 1 //keystone
  // for pitch and roll velocity control
  //pitch
  if(finalTargetVelX - currentTargetVelX > targetPitchRollInterval_)
    currentTargetVelX += targetPitchRollInterval_;
  else if (finalTargetVelX - currentTargetVelX < - targetPitchRollInterval_)
    currentTargetVelX -= targetPitchRollInterval_;
  else
    currentTargetVelX = finalTargetVelX;
  //roll
  if(finalTargetVelY - currentTargetVelY > targetPitchRollInterval_)
    currentTargetVelY += targetPitchRollInterval_;
  else if (finalTargetVelY - currentTargetVelY < - targetPitchRollInterval_)
    currentTargetVelY -= targetPitchRollInterval_;
  else
    currentTargetVelY = finalTargetVelY;
#else
  currentTargetVelX = finalTargetVelX;
  currentTargetVelY = finalTargetVelY;
#endif 

}


void TeleopNavigator::teleopNavigation(bool& polling_able,  Estimator* estimator)
{
  static int convergence_cnt = 0; //収束回数、つまり収束時
  static int clock_cnt = 0; //mainly for velocity control takeoff

  //throwing mode
  if(useThrowingMode_)
    throwingModeNavi(estimator);

  //keystron correction / target value process
  targetValueCorrection();

  if(getNaviCommand() == START_COMMAND)
    { //takeoff phase
      //**** 受信開始
      polling_able= true;
      flightMode = NO_CONTROL_MODE;

    }
  else if(getNaviCommand() == TAKEOFF_COMMAND)
    {	//Take OFF Phase

      flightMode = TAKEOFF_MODE;

      //TODO convergenceFunction();
      //収束確認
      //*** about height. be more strict (-30 ~ 30) ~ (-50 ~ 50)
      //*** about pitch. be more strict (-100 ~ 100) ~ (-50 ~ 50)
      if(xyControlMode_ == POS_WORLD_BASED_CONTROL_MODE)
        {
          if (fabs(getTargetPosZ() - estimator->getStatePosZ()) < 0.05 &&
              fabs(getTargetPosX() - estimator->getStatePosX()) < 0.15 &&
              fabs(getTargetPosY() - estimator->getStatePosY()) < 0.15)
            convergence_cnt++;
        }
      else if(xyControlMode_ == VEL_LOCAL_BASED_CONTROL_MODE)
        {
          if (fabs(getTargetPosZ() - estimator->getStatePosZ()) < 0.05)
              convergence_cnt++;
        }


      if (convergence_cnt > ctrlLoopRate) 
	{ //*** 安定収束した 20 ~ 40

          if(xyControlMode_ == POS_WORLD_BASED_CONTROL_MODE)
            {
              convergence_cnt = 0;
              setNaviCommand(HOVER_COMMAND); 
              startFlight();
              ROS_WARN(" x/y pos stable hovering POS_WORLD_BASED_CONTROL_MODE");
            }
          else if(xyControlMode_ == VEL_LOCAL_BASED_CONTROL_MODE)
            {
              //#if 1  // position pid control for takeoff 
              if(xyVelModePosCtrlTakeoff_)
                {
                  clock_cnt++;
                  ROS_WARN(" stable, clock_cnt: %d ", clock_cnt);
                  if(clock_cnt > (ctrlLoopRate * TAKEOFF_COUNT))
                    {
                      clock_cnt = 0;
                      convergence_cnt = 0;
                      setNaviCommand(HOVER_COMMAND); 
                      startFlight();
                      ROS_WARN(" x/y pos stable hovering VEL_LOCAL_BASED_CONTROL_MODE");
                    }
                }
              //#else  //velocity pid control for takeoff
              else
                {
                  convergence_cnt = 0;
                  setNaviCommand(HOVER_COMMAND); 
                  startFlight();
                  ROS_WARN(" no x/y pos stable hovering ");
                }
              //#endif
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
	  flightMode = RESET_MODE;
          motorStopFlag = false;
          estimator->setRocketStartFlag();
          resetFreeFallFlag();

          if(xyControlMode_ == VEL_WORLD_BASED_CONTROL_MODE) xyControlMode_ = POS_WORLD_BASED_CONTROL_MODE;
	}
      else if (getFlightAble()  && getStartAble())
	{
	  flightMode = LAND_MODE; //--> for control
          //ROS_INFO("free fall thre is %f, z is %f", freeFallThre_,  estimator->getStatePosZ());
          // if(useFreeFall_) ROS_INFO(" use free fall true");
          // else ROS_INFO(" do not use free fall false");

          if(useFreeFall_)
            {
              if(freeFallThre_ > estimator->getStatePosZ())
                {
                  ROS_WARN(" free fall mode");
                  freeFallFlag = true;
                }
              if(motorStopFlag)
                {
                  ROS_ERROR(" stop motor arming");
                  convergence_cnt = 0;
                  stopFlight();
                }
            }
          else
            { // do not use free fall
              if (fabs(getTargetPosZ() - estimator->getStatePosZ()) < 0.05)
                convergence_cnt++;
              if (convergence_cnt > ctrlLoopRate) 
                { 
                  ROS_WARN("  no free fall mode");
                  convergence_cnt = 0;
                  stopFlight();
                }
            }
	} 
      else if (!getStartAble())
	{
	  flightMode = NO_CONTROL_MODE;
	  setNaviCommand(IDLE_COMMAND);


          estimator->setRocketStartFlag();
          resetFreeFallFlag();

	}
    }
  else if(getNaviCommand() == CTRL_ACK_COMMAND)
    {
      //pid_controller->pidFunction(this, estimator, flight_ctrl_input, FLIGHT_MODE);
      flightMode = FLIGHT_MODE;

      if(fabs(getTargetPosX() - estimator->getStatePosX()) < 0.15 &&
	 fabs(getTargetPosY() - estimator->getStatePosY()) < 0.15)
	{
	  float theta_err = getTargetPsi() - estimator->getStatePsiBody();
	  if(theta_err > M_PI) theta_err = -2 * M_PI + theta_err;
	  else if(theta_err < -M_PI) theta_err = 2 * M_PI + theta_err;
	  if(fabs(theta_err) < 0.090 )//近傍、10度
	    convergence_cnt++; //収束
	}
      if (convergence_cnt > ctrlLoopRate)
	{
	  convergence_cnt = 0;
	  ROS_INFO("    Navigation Convergence");
	  setNaviCommand(HOVER_COMMAND);
	}
    }
  else if(getNaviCommand() == HOVER_COMMAND)
    {
      flightMode = FLIGHT_MODE;
    }
  else if(getNaviCommand() == IDLE_COMMAND)
    {
      flightMode = NO_CONTROL_MODE;
    }
  else if(getNaviCommand() == STOP_COMMAND)
    {
      flightMode = NO_CONTROL_MODE;
    }
  else
    {
      flightMode = NO_CONTROL_MODE;
      ROS_WARN("ERROR COMMAND, CAN NOT BE SEND TO AERIAL ROBOT");
    }
}

void TeleopNavigator::sendRcCmd(FlightCtrlInput* flight_ctrl_input)
{
  if(getNaviCommand() == START_COMMAND)
    { 
     ROS_INFO("START_COMMAND");
     //setNaviCommand(IDLE_COMMAND);
#if 1
      std_msgs::UInt16 startCmd;
      startCmd.data = ARM_ON_CMD;
      mspCmdPub_.publish(startCmd); 
#else
      jsk_quadcopter_common::RcData rcData;
      rcData.roll = 0;
      rcData.pitch = 0;
      rcData.yaw = 600;
      rcData.throttle = -600;
      rcCmdPub_.publish(rcData);
#endif
    }
  else if(getNaviCommand() == STOP_COMMAND)
    { 
#if 1
      std_msgs::UInt16 stopCmd;
      stopCmd.data = ARM_OFF_CMD;
      mspCmdPub_.publish(stopCmd);

#else
      jsk_quadcopter_common::RcData rcData;
      rcData.roll = 0;
      rcData.pitch = 0;
      rcData.yaw = -600;
      rcData.throttle = -600;
      rcCmdPub_.publish(rcData);
#endif
      // ROS_INFO("STOP_COMMAND");
    }
  else if(getNaviCommand() == TAKEOFF_COMMAND ||
          getNaviCommand() == LAND_COMMAND ||
          getNaviCommand() == CTRL_COMMAND ||
          getNaviCommand() == CTRL_ACK_COMMAND ||
          getNaviCommand() == HOVER_COMMAND)
    {
      jsk_quadcopter_common::RcData rcData;
      rcData.roll  =  flight_ctrl_input->getRollValue();
      rcData.pitch =  flight_ctrl_input->getPitchValue();
      rcData.yaw   =  flight_ctrl_input->getYawValue();
      //rcData.throttle = flight_ctrl_input->getThrottleValue() - 1520; //MIDRC
      rcData.throttle = flight_ctrl_input->getThrottleValue() ;
      rcCmdPub_.publish(rcData);

      //ROS_INFO("Flight_COMMAND");
    }
  else
    {
      //ROS_ERROR("ERROR PISITION COMMAND, CAN NOT BE SEND TO Quadcopter");
    }
}

void TeleopNavigator::rosParamInit()
{
  std::string ns = navigatorNodeHandlePrivate_.getNamespace();
  //*** teleop navigation
    if (!navigatorNodeHandlePrivate_.getParam ("useFreeFall", useFreeFall_))
      useFreeFall_ = false;
    printf("%s: useFreeFall is %s\n", ns.c_str(), useFreeFall_ ? ("true") : ("false"));
    if (!navigatorNodeHandlePrivate_.getParam ("freeFallThre", freeFallThre_))
      freeFallThre_ = 0;
    printf("%s: freeFallThre_ is %.3f\n", ns.c_str(), freeFallThre_);

  if (!navigatorNodeHandlePrivate_.getParam ("takeoffHeight", takeoffHeight_))
    takeoffHeight_ = 0;
  printf("%s: takeoffHeight_ is %.3f\n", ns.c_str(), takeoffHeight_);

  if (!navigatorNodeHandlePrivate_.getParam ("evenMoveDistance", evenMoveDistance_))
    evenMoveDistance_ = 0;
  printf("%s: evenMoveDistance_ is %.3f\n", ns.c_str(), evenMoveDistance_);

  if (!navigatorNodeHandlePrivate_.getParam ("upDownDistance", upDownDistance_))
    upDownDistance_ = 0;
  printf("%s: upDownDistance_ is %.3f\n", ns.c_str(), upDownDistance_);

  if (!navigatorNodeHandlePrivate_.getParam ("forwardBackwardDistance", forwardBackwardDistance_))
    forwardBackwardDistance_ = 0;
  printf("%s: forwardBackwardDistance_ is %.3f\n", ns.c_str(), forwardBackwardDistance_);

  if (!navigatorNodeHandlePrivate_.getParam ("leftRightDistance", leftRightDistance_))
    leftRightDistance_ = 0;
  printf("%s: leftRightDistance_ is %.3f\n", ns.c_str(), leftRightDistance_);

  if (!navigatorNodeHandlePrivate_.getParam ("targetVelRate", targetVelRate_))
    targetVelRate_ = 0;
  printf("%s: targetVelRate_ is %.3f\n", ns.c_str(), targetVelRate_);

  if (!navigatorNodeHandlePrivate_.getParam ("targetPitchRollInterval", targetPitchRollInterval_))
    targetPitchRollInterval_ = 0;
  printf("%s: targetPitchRollInterval_ is %.3f\n", ns.c_str(), targetPitchRollInterval_);

  if (!navigatorNodeHandlePrivate_.getParam ("targetAltInterval", targetAltInterval_))
    targetAltInterval_ = 0;
  printf("%s: targetAltInterval_ is %.3f\n", ns.c_str(), targetAltInterval_);

  if (!navigatorNodeHandlePrivate_.getParam ("targetYawRate", targetYawRate_))
    targetYawRate_ = 0;
  printf("%s: targetYawRate_ is %.3f\n", ns.c_str(), targetYawRate_);

  if (!navigatorNodeHandlePrivate_.getParam ("naviFrameInt", naviFrameInt_))
    naviFrameInt_ = 0;
  printf("%s: naviFrameInt_ is %d\n", ns.c_str(), naviFrameInt_);
  naviFrame = naviFrameInt_;

  if (!navigatorNodeHandlePrivate_.getParam ("mapFrame", mapFrame_))
    mapFrame_ = "unknown";
  printf("%s: mapFrame_ is %s\n", ns.c_str(), mapFrame_.c_str());

  if (!navigatorNodeHandlePrivate_.getParam ("targetFrame", targetFrame_))
    targetFrame_ = "unknown";
  printf("%s: targetFrame_ is %s\n", ns.c_str(), targetFrame_.c_str());

  if (!navigatorNodeHandlePrivate_.getParam ("xyControlMode", xyControlMode_))
    xyControlMode_ = 0;
  printf("%s: xyControlMode_ is %d\n", ns.c_str(), xyControlMode_);

  //hidden variable
  if (!navigatorNodeHandlePrivate_.getParam ("xyVelModePosCtrlTakeoff", xyVelModePosCtrlTakeoff_))
    xyVelModePosCtrlTakeoff_ = true;
  printf("%s: xyVelModePosCtrlTakeoff is %s\n", ns.c_str(), xyVelModePosCtrlTakeoff_ ? ("true") : ("false"));

  if (!navigatorNodeHandlePrivate_.getParam ("useThrowingMode", useThrowingMode_))
    useThrowingMode_ = false;
  printf("%s: useThrowingMode is %s\n", ns.c_str(), useThrowingMode_ ? ("true") : ("false"));

  if (!navigatorNodeHandlePrivate_.getParam ("throwingModeStandbyAltThre", throwingModeStandbyAltThre_))
    throwingModeStandbyAltThre_ = 0;
  printf("%s: throwingModeStandbyAltThre_ is %f\n", ns.c_str(), throwingModeStandbyAltThre_);

  if (!navigatorNodeHandlePrivate_.getParam ("throwingModeThrowAccXThre", throwingModeThrowAccXThre_))
    throwingModeThrowAccXThre_ = 0;
  printf("%s: throwingModeThrowAccXThre_ is %f\n", ns.c_str(), throwingModeThrowAccXThre_);

  if (!navigatorNodeHandlePrivate_.getParam ("throwingModeThrowAccZThre", throwingModeThrowAccZThre_))
    throwingModeThrowAccZThre_ = 0;
  printf("%s: throwingModeThrowAccZThre_ is %f\n", ns.c_str(), throwingModeThrowAccZThre_);

  if (!navigatorNodeHandlePrivate_.getParam ("throwingModeSetAltAccZThre", throwingModeSetAltAccZThre_))
    throwingModeSetAltAccZThre_ = 0;
  printf("%s: throwingModeSetAltAccZThre_ is %f\n", ns.c_str(), throwingModeSetAltAccZThre_);

  if (!navigatorNodeHandlePrivate_.getParam ("throwingModeAltHoldVelZThre", throwingModeAltHoldVelZThre_))
    throwingModeAltHoldVelZThre_ = 0;
  printf("%s: throwingModeAltHoldVelZThre_ is %f\n", ns.c_str(), throwingModeAltHoldVelZThre_);

  if (!navigatorNodeHandlePrivate_.getParam ("throwingModeShiftStepCntThre", throwingModeShiftStepCntThre_))
    throwingModeShiftStepCntThre_ = 0;
  printf("%s: throwingModeShiftStepCntThre_ is %d\n", ns.c_str(), throwingModeShiftStepCntThre_);

  if (!navigatorNodeHandlePrivate_.getParam ("cmdVelLev2Gain", cmdVelLev2Gain_))
    cmdVelLev2Gain_ = 1.0;
  printf("%s: cmdVelLev2Gain_ is %.3f\n", ns.c_str(), cmdVelLev2Gain_);
}

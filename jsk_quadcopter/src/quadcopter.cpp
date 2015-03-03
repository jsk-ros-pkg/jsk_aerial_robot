#include "jsk_quadcopter/quadcopter.h"

Quadcopter::Quadcopter(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : quadcopterNodeHandle_(nh), quadcopterNodeHandlePrivate_(nh_private)
{
  std::string ns = quadcopterNodeHandlePrivate_.getNamespace();

  if (!quadcopterNodeHandlePrivate_.getParam ("rxLoopRate", rxLoopRate_))
    rxLoopRate_ = 0;
  printf("%s: rxLoopRate_ is %.3f\n", ns.c_str(), rxLoopRate_);

  if (!quadcopterNodeHandlePrivate_.getParam ("txLoopRate", txLoopRate_))
    txLoopRate_ = 0;
  printf("%s: txLoopRate_ is %.3f\n", ns.c_str(), txLoopRate_);

}

Quadcopter::~Quadcopter()
{
}

void Quadcopter::rxFunction(const ros::TimerEvent & e)
{
}

void Quadcopter::txFunction(const ros::TimerEvent & e)
{
}


JskQuadcopter::JskQuadcopter(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : Quadcopter(nh, nh_private)
{
  rosParamInit(quadcopterNodeHandlePrivate_);

  //*** estimator1
  estimator = new Estimator1(quadcopterNodeHandle_,
                             quadcopterNodeHandlePrivate_,
                             simulationFlag_);

  if(!simulationFlag_)
    { //kalman filter のsimulation をするときは、sierial communication, navigation, controlはいらない
      //*** serial interface
      //*** control input 
      flightCtrlInput = new FlightCtrlInput();
      //*** pid controller
      controller = new PidController(quadcopterNodeHandle_,
                                     quadcopterNodeHandlePrivate_,
                                     txLoopRate_);
      if(asctecFlag_)
        {
          //controlBoard_ = ASCTEC_BOARD; 
          //kalmanFilterBoard_ = ASCTEC_BOARD;
          controller->setControlBoardToAsctec();
          estimator->setKalmanFilterBoardToAsctec();
          serialInterface = new SerialInterface(port_, speed_, everyByteInterval_);
        }

      //*** teleop navigation
      navigator = new TeleopNavigator(quadcopterNodeHandle_,
                                      quadcopterNodeHandlePrivate_,
                                      estimator, txLoopRate_);

      if(trackingFlag_)
        tracker = new Tracking(quadcopterNodeHandle_, quadcopterNodeHandlePrivate_,
                               navigator, estimator);
    }


  if(tfPubLoopRate_ <= 0)
    ROS_ERROR("Disable the tfPub function\n");
  else
    {
      tf_thread = boost::thread(boost::bind(&JskQuadcopter::tfPubFunction, this));
    }


  if(txLoopRate_ <= 0)
    ROS_ERROR("Disable the tx function\n");
  else
    {
      txTimer_ = quadcopterNodeHandlePrivate_.createTimer(ros::Duration(1.0 / txLoopRate_), &JskQuadcopter::txFunction,this);
    }


  if(rxLoopRate_ <= 0)
    ROS_ERROR("Disable the rx function\n");
  else
    rxTimer_ = quadcopterNodeHandlePrivate_.createTimer(ros::Duration(1.0 / rxLoopRate_),
							&JskQuadcopter::rxFunction,
							this);
}

JskQuadcopter::~JskQuadcopter()
{
  delete estimator;

  if(!simulationFlag_)
    {
      delete controller;
      delete serialInterface;
      delete navigator;
      delete  flightCtrlInput;
    }
  tf_thread.interrupt();
  tf_thread.join();

  printf("Deleted all!\n");
}

void JskQuadcopter::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nh.getNamespace();

  if (!nh.getParam ("simulationFlag", simulationFlag_))
    simulationFlag_ = false;
  printf("%s: simulationFlag is %s\n", ns.c_str(), simulationFlag_ ? ("true") : ("false"));

  if (!nh.getParam ("tfPubLoopRate", tfPubLoopRate_))
    tfPubLoopRate_ = 0;
  printf("%s: tfPubLoopRate_ is %.3f\n", ns.c_str(),  tfPubLoopRate_);

  if (!nh.getParam ("trackingFlag", trackingFlag_))
    trackingFlag_ = false;
  printf("%s: trackingFlag is %s\n", ns.c_str(), trackingFlag_ ? ("true") : ("false"));


  //*** asctec param (including serial comm without ros)
  if (!nh.getParam ("asctecFlag", asctecFlag_))
    asctecFlag_ = false;
  printf("%s: asctecFlag is %s\n", ns.c_str(), asctecFlag_ ? ("true") : ("false"));
  asctecDataPollingFlag_ = false;
  if (asctecFlag_) 
    {
      if (!nh.getParam ("speed", speedInt_))
        speedInt_ = 115200;
      printf("%s: speed_ is %d\n", ns.c_str(), speedInt_);
      speed_ = (uint32_t)speedInt_;

      if (!nh.getParam ("port", port_))
        port_ = "/dev/ttyUSB0";
      printf("%s: port_ is %s\n", ns.c_str(), port_.c_str());

      if (!nh.getParam ("everyByteInterval", everyByteInterval_))
        everyByteInterval_ = 0;
      printf("%s: everyByteInterval_ is %d\n", ns.c_str(), everyByteInterval_);
     }

  // if(!simulationFlag_)
  //   printf(" controlBoard_ is %d\n", controlBoard_);
  // printf(" kalmanFilterBoard_ is %d\n", kalmanFilterBoard_);
}

bool JskQuadcopter::asctecGetSpecificPacket(Estimator1* estimator, 
                                            Navigator* navigator,
                                            SerialInterface* serial_interface,
                                            bool& polling_flag)
{
  uint8_t spacket[1024];
  uint16_t packet_size;
  uint8_t packet_type;
  bool read_result = false;

  ros::Time packetTimeStamp;

  if(polling_flag)
    read_result = serial_interface->getPacket(spacket, &packet_type, &packet_size, packetTimeStamp);

  if (read_result){
    if (packet_type == PD_IMUDATA)
      {
	estimator->imuData_->asctecGetImuDataArray(spacket, packet_size, 
                                             packetTimeStamp,
                                             estimator->slamData_->getRawPsiSlamValue());
      }
    else if (packet_type == PD_CTRLINPUT)
      {
	ROS_INFO ("  Packet type is CTRL INPUT");
      }
    else if (packet_type == PD_ACK_RES)
      {
	char tmp = '0';
	memcpy (&tmp, spacket,packet_size);
	if(tmp == 's') {
	  ROS_INFO("START RES From ASCTEC");
	  navigator->startNavigation(); 
	  //navigator->ENABLE_FLAG_.startEnable = true;
	  navigator->setNaviCommand(navigator->IDLE_COMMAND);
	}else if(tmp == 'e') {
	  ROS_INFO("STOP RES From ASCTEC");
	  navigator->stopNavigation(); 
	  navigator->setNaviCommand(navigator->IDLE_COMMAND);
	  //asctecDataPollinFlag = false;
          polling_flag = false;
	}else if(tmp == 'c'){
	  ROS_INFO("        FUll Axis CTRL RES From ASCTEC");
	}else if(tmp == 't'){
	  ROS_INFO("        THRUST CONTROL RES From ASCTEC");
	}else{ 
	  //ROS_INFO("Some different or wrong response from ASCTEC");
	}
      }
    else
      {
	ROS_ERROR ("  Packet type (%#2x) is UNKNOWN", packet_type);
      }
  }
  else{
    //ROS_ERROR ("  Read failed");	
  }
  return read_result;
}


void JskQuadcopter::rxFunction(const ros::TimerEvent & e)
{
  if(!simulationFlag_)
    {
      if(asctecFlag_ )  //for asctec
        asctecGetSpecificPacket(estimator, navigator, serialInterface, asctecDataPollingFlag_);
    }
}


void JskQuadcopter::txFunction(const ros::TimerEvent & e)
{

  if(!simulationFlag_)
    {
      navigator->teleopNavigation(asctecDataPollingFlag_, estimator);
      controller->pidFunction(navigator, estimator, flightCtrlInput);
      //feed forward control
      controller->feedForwardFunction(navigator, estimator, flightCtrlInput);

      if(controller->getControlBoard() == controller->KDUINO_BOARD)
        navigator->sendRcCmd(flightCtrlInput);
      if(controller->getControlBoard() == controller->ASCTEC_BOARD)
        serialInterface->sendPacket(FLIGHT_CONTROL, navigator, estimator, flightCtrlInput);
    }
}

void JskQuadcopter::tfPubFunction()
{

  ros::Rate loop_rate(tfPubLoopRate_);

  while(ros::ok())
    {
      estimator->tfPublish();
      if(!simulationFlag_)
        navigator->tfPublish();

      ros::spinOnce();
      loop_rate.sleep();
    }
}


#ifndef FLIGHT_NAVIGATION_H
#define FLIGHT_NAVIGATION_H

//* ros
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <jsk_quadcopter_common/VirtualRC.h>
#include <jsk_quadcopter_common/RcData.h>
#include <tf/transform_broadcaster.h>
#include <jsk_quadcopter/state_estimation.h>
#include <jsk_quadcopter/control_input_array.h>
#include <jsk_quadcopter/FlightNav.h>
#include <sensor_msgs/Joy.h>



class Navigator
{
 public:
  Navigator(ros::NodeHandle nh,	ros::NodeHandle nh_private, int ctrl_loop_rate);
  virtual ~Navigator();

  //*** startAble
  bool getStartAble();
  void startNavigation();
  void stopNavigation();

  //*** flightAble
  bool getFlightAble();
  void startFlight();
  void stopFlight();
  //*** command type
  uint8_t getNaviCommand();
  void setNaviCommand(const uint8_t  command);
  void tfPublish();

  float getTargetPosX();
  void setTargetPosX(float value);
  void addTargetPosX(float value);
  float getTargetVelX();
  void setTargetVelX(float value);
  float getTargetPosY();
  void setTargetPosY(float value);
  void addTargetPosY(float value);
  float getTargetVelY();
  void setTargetVelY(float value);
  float getTargetPosZ();
  void setTargetPosZ(float value);
  void addTargetPosZ(float value);
  float getTargetVelZ();
  void setTargetVelZ(float value);
  float getTargetTheta();
  void setTargetTheta(float value);
  float getTargetVelTheta();
  void setTargetVelTheta(float value);
  float getTargetPhy();
  void setTargetPhy(float value);
  float getTargetVelPhy();
  void setTargetVelPhy(float value);
  float getTargetPsi();
  void setTargetPsi(float value);
  float getTargetVelPsi();
  void setTargetVelPsi(float value);


  virtual uint8_t getFlightMode(); //for teleop navigator
  virtual void  setXyControlMode(uint8_t mode); //for teleop navigator
  virtual uint8_t getXyControlMode(); //for teleop navigator
  virtual bool getXyVelModePosCtrlTakeoff();
  virtual  bool getMotorStopFlag();
  virtual  void setMotorStopFlag(bool motor_stop_flag);
  virtual  bool getFreeFallFlag();
  virtual  void resetFreeFallFlag();
  virtual  uint8_t getThrowingMode();
  /* virtual void setOuterTargetPoseFlag(uint8_t axis); */
  /* virtual void setOuterTargetVelFlag(uint8_t axis); */


  const static uint8_t POS_CONTROL_COMMAND = 0;
  const static uint8_t VEL_CONTROL_COMMAND = 1;

  static const uint8_t START_COMMAND = 0x00;
  static const uint8_t STOP_COMMAND = 0x01;
  static const uint8_t IDLE_COMMAND = 0x02;
  static const uint8_t TAKEOFF_COMMAND = 0x03;
  static const uint8_t LAND_COMMAND = 0x04;
  static const uint8_t CTRL_COMMAND = 0x05;
  static const uint8_t CTRL_ACK_COMMAND= 0x06;
  static const uint8_t HOVER_COMMAND= 0x07;
  static const uint8_t DISABLE_COMMAND= 0x08;

  //for throwing start, not good
  const static uint8_t THROWING_START_STANDBY = 0x31;
  const static uint8_t THROWING_START_ARMINGON = 0x32;
  const static uint8_t THROWING_START_ALTHOLD = 0x33;

  //for ros arm/disarm cmd
  const static uint8_t ARM_ON_CMD = 150;
  const static uint8_t ARM_OFF_CMD = 151;
  const static uint8_t DROP_CMD = 160;

  static const uint8_t X_AXIS = 1;
  static const uint8_t Y_AXIS = 2;
  static const uint8_t Z_AXIS = 4;
  static const uint8_t PITCH_AXIS = 8;
  static const uint8_t ROLL_AXIS = 16;
  static const uint8_t YAW_AXIS = 32;

  const static uint8_t POS_WORLD_BASED_CONTROL_MODE = 0;
  const static uint8_t POS_LOCAL_BASED_CONTROL_MODE = 1;
  const static uint8_t VEL_WORLD_BASED_CONTROL_MODE = 2;
  const static uint8_t VEL_LOCAL_BASED_CONTROL_MODE = 3;


  protected:
    ros::NodeHandle navigatorNodeHandle_;
    ros::NodeHandle navigatorNodeHandlePrivate_;
    tf::TransformBroadcaster* tfB_ ; 

    bool startAble;
    bool flightAble;
    uint8_t naviCommand;

    // final target value
    float finalTargetPosX;
    float finalTargetVelX;
    float finalTargetPosY;
    float finalTargetVelY;
    float finalTargetPosZ;
    float finalTargetVelZ;
    float finalTargetTheta;
    float finalTargetVelTheta;
    float finalTargetPhy;
    float finalTargetVelPhy;
    float finalTargetPsi;
    float finalTargetVelPsi;

    //current target value
    float currentTargetPosX;
    float currentTargetVelX;
    float currentTargetPosY;
    float currentTargetVelY;
    float currentTargetPosZ;
    float currentTargetVelZ;
    float currentTargetTheta;
    float currentTargetVelTheta;
    float currentTargetPhy;
    float currentTargetVelPhy;
    float currentTargetPsi;
    float currentTargetVelPsi;

    //outer target value
    /* float outerTargetPosX; */
    /* float outerTargetVelX; */
    /* float outerTargetPosY; */
    /* float outerTargetVelY; */
    /* float outerTargetPosZ; */
    /* float outerTargetVelZ; */
    /* float outerTargetTheta; */
    /* float outerTargetVelTheta; */
    /* float outerTargetPhy; */
    /* float outerTargetVelPhy; */
    /* float outerTargetPsi; */
    /* float outerTargetVelPsi; */

    int ctrlLoopRate;
    std::string mapFrame_;
    std::string targetFrame_;

    /* uint8_t useOuterTargetPose; */
    /* uint8_t useOuterTargetVel; */

};

class TeleopNavigator :public Navigator
{
 public:
  TeleopNavigator(ros::NodeHandle nh,
                  ros::NodeHandle nh_private,
                  Estimator* estimator,
                  int ctrl_loop_rate);
  virtual ~TeleopNavigator();

  void teleopNavigation(bool& polling_able,  Estimator* estimator);
  void sendRcCmd(FlightCtrlInput* flight_ctrl_input);

  void takeoffCallback(const std_msgs::EmptyConstPtr & msg);
  void startCallback(const std_msgs::EmptyConstPtr & msg, Estimator* estimator);
  void haltCallback(const std_msgs::EmptyConstPtr &  msg);
  void landCallback(const std_msgs::EmptyConstPtr &  msg, Estimator* estimator);
  void rollCallback(const std_msgs::Int8ConstPtr & msg, Estimator* estimator);
  void pitchCallback(const std_msgs::Int8ConstPtr & msg, Estimator* estimator);
  void yawCallback(const std_msgs::Int8ConstPtr & msg, Estimator* estimator);
  void throttleCallback(const std_msgs::Int8ConstPtr & msg, Estimator* estimator);

  void joyStickControl(const sensor_msgs::JoyConstPtr joy_msg, Estimator* estimator);
  void flightNavCallback(const jsk_quadcopter::FlightNavConstPtr & msg);

  void xyControlModeCallback(const std_msgs::Int8ConstPtr & msg);
  void armingAckCallback(const std_msgs::Int8ConstPtr& ack_msg);

  void targetValueCorrection();

  void throwingModeNavi(Estimator* estimator);
  uint8_t getThrowingMode(); 

  uint8_t getFlightMode();
  void  setXyControlMode(uint8_t mode); 
  uint8_t getXyControlMode(); 

  bool getMotorStopFlag();
  void setMotorStopFlag(bool motor_stop_flag); 

  bool getFreeFallFlag();
  void resetFreeFallFlag(); 

  bool getXyVelModePosCtrlTakeoff();

  const static uint8_t MAP_FRAME = 0;
  const static uint8_t BODY_FRAME = 1;

  //for pidFunction

  const static uint8_t TAKEOFF_MODE = 0;
  const static uint8_t FLIGHT_MODE = 1;
  const static uint8_t LAND_MODE = 2;
  const static uint8_t NO_CONTROL_MODE = 3; 
  const static uint8_t RESET_MODE = 4;

  const static int TAKEOFF_COUNT = 8;


  const static uint8_t OPT_GOOD_TRACKING = 0x01;
  const static uint8_t OPT_ZERO_TRACKING = 0x02;
  const static uint8_t OPT_RECOVER_TRACKING = 0x03;

 private:
    ros::Publisher  rcCmdPub_;
    ros::Publisher  mspCmdPub_;
    ros::Publisher  activatePub_;
    ros::Publisher  stateShiftPub_;

    ros::Subscriber armingAckSub_;
    ros::Subscriber takeoffSub_;
    ros::Subscriber landSub_;
    ros::Subscriber startSub_;
    ros::Subscriber haltSub_;
    ros::Subscriber rollSub_;
    ros::Subscriber pitchSub_;
    ros::Subscriber yawSub_;
    ros::Subscriber throttleSub_;
    ros::Subscriber ctrlModeSub_;
    ros::Subscriber joyStickSub_;
    ros::Subscriber flightNavSub_;

    //temporarily
    ros::Publisher  jointsCtrlPub_;

    //*** base navigation
    uint8_t flightMode; //important
    int  xyControlMode_;
    bool xyVelModePosCtrlTakeoff_;

    //*** free fall
    double freeFallThre_;
    bool   useFreeFall_;
    bool   freeFallFlag;
    bool   motorStopFlag;

    bool   useThrowingMode_;
    uint8_t throwingMode;
    double throwingModeStandbyAltThre_;
    double throwingModeThrowAccXThre_;
    double throwingModeThrowAccZThre_;
    double throwingModeSetAltAccZThre_;
    double throwingModeAltHoldVelZThre_;
    int    throwingModeShiftStepCntThre_;

    //*** teleop navigation
    double takeoffHeight_;
    double evenMoveDistance_;
    double upDownDistance_;
    double forwardBackwardDistance_;
    double leftRightDistance_;
    double targetVelRate_;
    double targetPitchRollInterval_;
    double targetAltInterval_;
    double targetYawRate_;

    double cmdVelLev2Gain_;
    int naviFrameInt_;
    uint8_t naviFrame;


    //for alt joy stick control
    bool  imageSendingFlag; //deprecated
    bool  deploymentFlag;  //deprecated
    bool  dropFlag;
    bool  activateFlag;
    bool  velControlFlag;
    bool  posControlFlag;
    bool  altControlFlag;
    bool  yawControlFlag;

    void rosParamInit();
};


#endif

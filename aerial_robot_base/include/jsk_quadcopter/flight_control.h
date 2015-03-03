#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

//* ros
#include <ros/ros.h>
#include <jsk_quadcopter/fourAxisPidDebug.h>
#include <jsk_quadcopter/state_estimation.h>
#include <jsk_quadcopter/control_input_array.h>
#include <jsk_quadcopter/flight_navigation.h>
#include <jsk_quadcopter_common/ITermBias.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <jsk_quadcopter_common/DynamicReconfigureLevels.h>
#include <jsk_quadcopter/PidPitchControlConfig.h>
#include <jsk_quadcopter/PidRollControlConfig.h>
#include <jsk_quadcopter/PidYawControlConfig.h>
#include <jsk_quadcopter/PidThrottleControlConfig.h>
#include <jsk_quadcopter_common/RPYCtrlOffset.h>
//+*+*+*+* 相互参照のための先方宣言
/* class Navigator; */
//class KeyboardNavigator;


class FlightController
{
public:
  FlightController();
  virtual ~FlightController();

  float limit(float value, int limit);
};

class PidController : public FlightController
{
 public:
  PidController(ros::NodeHandle nh,
                ros::NodeHandle nh_private,
                double ctrl_loop_rate);
  ~PidController();

  //for pidFunction
  /* const static uint8_t POS_CONTROL_MODE = 0; */
  /* const static uint8_t VEL_CONTROL_MODE = 1; */
  /* const static uint8_t VEL_POS_BASED_CONTROL_MODE = 2; */

  const static uint8_t TAKEOFF_MODE = 0;
  const static uint8_t FLIGHT_MODE = 1;
  const static uint8_t LAND_MODE = 2;
  const static uint8_t NO_CONTROL_MODE = 3;
  const static uint8_t RESET_MODE = 4;
  
  //+*+*+*+*+*+*+* critical point
  const static int KDUINO_BOARD = 0;
  const static int ASCTEC_BOARD = 1;


  //const static int TAKEOFF_COUNT = 10;

  
  void pidFunction(Navigator* navigator, Estimator* estimator,
		   FlightCtrlInput* flight_ctrl_input);

  void feedForwardFunction(Navigator* navigator, Estimator* estimator,
                           FlightCtrlInput* flight_ctrl_input);

  //dynamic reconfigure
  void cfgPitchCallback(jsk_quadcopter::PidPitchControlConfig &config, uint32_t level);
  void cfgRollCallback(jsk_quadcopter::PidRollControlConfig &config, uint32_t level);
  void cfgThrottleCallback(jsk_quadcopter::PidThrottleControlConfig &config, uint32_t level);
  void cfgYawCallback(jsk_quadcopter::PidYawControlConfig &config, uint32_t level);

  void setControlBoardToAsctec();
  uint8_t getControlBoard();
 private:
  ros::NodeHandle controllerNodeHandle_;
  ros::NodeHandle controllerNodeHandlePrivate_;
  ros::Publisher  pidPub_;
  ros::Publisher motorBiasSetPub_;
  ros::Subscriber motorBiasSetSub_;
  ros::Subscriber rpyCtrlOffsetSub_;

  int controlBoard_; // 0: asctec, 1: kawasaki
  int pidCtrlLoopRate_;
  int pitchCtrlCnt;
  int pitchCtrlLoopRate_;
  int rollCtrlCnt;
  int rollCtrlLoopRate_;
  int throttleCtrlCnt;
  int throttleCtrlLoopRate_;
  int yawCtrlCnt;
  int yawCtrlLoopRate_;
  
 //**** throttle
 double posPGainThrottle_;
 double posIGainThrottle_;
 double posDGainThrottle_;
 /* double velPGainThrottle_; */
 /* double velIGainThrottle_; */
 /* double velDGainThrottle_; */
 double posPGainThrottleLand_;
 double posIGainThrottleLand_;
 double posDGainThrottleLand_;
 double constPControlThreThrottleLand_; //ThrottleLand時,P Termは定数しか取らない。高度によって値は変わる
 double constPTermLev1ValueThrottleLand_; //ThrottleLand時, Level 1のP Term Value
 double constPTermLev2ValueThrottleLand_; //ThrottleLand時, Level 2のP Term Value
 double constIControlThreThrottleLand_; //ThrottleLand時,I Termは高度によって定数を取る
 double constITermValueThrottleLand_;   //ThrottleLand時,I Termが定数を取る場合の値
 /* double velPGainThrottleLand_; */
 /* double velIGainThrottleLand_; */
 /* double velDGainThrottleLand_; */
 int offsetThrottle_;
 int posLimitThrottle_;
 int posPLimitThrottle_;
 int posILimitThrottle_;
 int posDLimitThrottle_;
 int posPLimitThrottleHover_;  //P limit in hover mode
 double velValueLimitThrottleHover_;  //D limit in hover mode
 double iEnableLimitThrottleHover_;  //I limit in hover mode
 int  rocketStartInitValue_;
 int  rocketStartInitIncrementValue_;
 int  rocketStartStepValue_;
 int  freeFallStepValue_;
 int  motorStopValue_;
 int throwingModeInitValueFromRocketStart_;
 /* int velLimitThrottle_; */
 /* int velPLimitThrottle_; */
 /* int velILimitThrottle_; */
 /* int velDLimitThrottle_; */
 //**** pitch
 double posPGainPitch_;
 double posIGainPitch_;
 double posDGainPitch_;
 double posIGainPitchHover_;
 double velPGainPitch_; 
 double velIGainPitch_; 
 /* double velDGainPitch_; */
 int offsetPitch_;
 int posLimitPitch_;
 int posPLimitPitch_;
 int posILimitPitch_;
 int posDLimitPitch_;
 double velValueLimitPitch_;
 double iEnableLimitPitch_;
 /* int velLimitPitch_; */
 /* int velPLimitPitch_; */
 /* int velILimitPitch_; */
 /* int velDLimitPitch_; */
 //**** roll
 double posPGainRoll_;
 double posIGainRoll_;
 double posDGainRoll_;
 double posIGainRollHover_;
 double velPGainRoll_; 
 double velIGainRoll_; 
 int offsetRoll_;
 /* double velDGainRoll_; */
 int posLimitRoll_;
 int posPLimitRoll_;
 int posILimitRoll_;
 int posDLimitRoll_;
 double velValueLimitRoll_;
 double iEnableLimitRoll_;
 /* int velLimitRoll_; */
 /* int velPLimitRoll_; */
 /* int velILimitRoll_; */
 /* int velDLimitRoll_; */
 //**** yaw
 double posPGainYaw_;
 double posIGainYaw_;
 double posDGainYaw_;
 /* double TurnPGainYaw_; */
 /* double TurnDGainYaw_; */
 /* double velPGainYaw_; */
 /* double velIGainYaw_; */
 /* double velDGainYaw_; */
 int posLimitYaw_;
 int posPLimitYaw_;
 int posILimitYaw_;
 int posDLimitYaw_;
 double velValueLimitYaw_;
 double iEnableLimitYaw_;
 /* int velLimitYaw_; */
 /* int velPLimitYaw_; */
 /* int velILimitYaw_; */
 /* int velDLimitYaw_; */
 /* int offsetYaw_; */
 //bool xyVelModePosCtrlTakeoff_;


  float dErrPosCurrPitch;
  float dErrPosCurrRoll;
  float dErrPosCurrThrottle;
  float dErrPosCurrYaw;

  float dErrVelCurrRoll;
  float dErrVelCurrPitch;
  float dErrVelCurrYaw;
  float dErrVelCurrThrottle;
  float dErrVelPrevRoll;
  float dErrVelPrevPitch;
  float dErrVelPrevYaw;
  float dErrVelPrevThrottle;

  float posITermRoll;
  float posITermPitch;
  float posITermYaw;
  float posITermThrottle;
  float posPTermRoll;
  float posPTermPitch;
  float posPTermYaw;
  float posPTermThrottle;
  float posDTermRoll;
  float posDTermPitch;
  float posDTermYaw;
  float posDTermThrottle;

  //motor bias set
  bool motorBiasFlag;
  void motorBiasSetCallback(const std_msgs::Int8ConstPtr& msg);
  void rpyCtrlOffsetCallback(const jsk_quadcopter_common::RPYCtrlOffsetConstPtr& offset_value);

  //std::getenv()
  //dynamic reconfigure
  dynamic_reconfigure::Server<jsk_quadcopter::PidPitchControlConfig>* pitchServer;
  dynamic_reconfigure::Server<jsk_quadcopter::PidRollControlConfig>* rollServer;
  dynamic_reconfigure::Server<jsk_quadcopter::PidThrottleControlConfig>* throttleServer;
  dynamic_reconfigure::Server<jsk_quadcopter::PidYawControlConfig>* yawServer;

  dynamic_reconfigure::Server<jsk_quadcopter::PidPitchControlConfig>::CallbackType dynamicReconfFuncPitch;
  dynamic_reconfigure::Server<jsk_quadcopter::PidRollControlConfig>::CallbackType dynamicReconfFuncRoll;
  dynamic_reconfigure::Server<jsk_quadcopter::PidThrottleControlConfig>::CallbackType dynamicReconfFuncThrottle;
  dynamic_reconfigure::Server<jsk_quadcopter::PidYawControlConfig>::CallbackType dynamicReconfFuncYaw;

  void rosParamInit();
  void throttleThrowingMode(Navigator* navigator, Estimator* estimator);

};



#endif

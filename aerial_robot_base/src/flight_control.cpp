/*
I_y * dot(w_y) = - x * F => Q (符号の話)
varphi_des = - 1/g * y_des => (roll軸制御　反転の話) 
 */

#include "jsk_quadcopter/flight_control.h"

FlightController::FlightController()
{
}

FlightController::~FlightController()
{
  printf(" deleted flight controller!\n");
}

float FlightController::limit(float value, int limit)
{
  if(value > (float)limit) 
    {
      //ROS_ERROR("upprer limit, value: %f , limit: %d", value, limit );
      return (float)limit;
    }
  else if(value < - (float)limit)
    {
      //ROS_ERROR("lower limit, value: %f , limit: %d", value, limit );
      return -(float)limit;
    }
  else return value;
}


PidController::PidController(ros::NodeHandle nh,
                             ros::NodeHandle nh_private,
                             double ctrl_loop_rate)
  : FlightController(), controllerNodeHandle_(nh, "controller"), controllerNodeHandlePrivate_(nh_private, "controller")
{
  pidCtrlLoopRate_ = ctrl_loop_rate;
  controlBoard_ = KDUINO_BOARD; //default
  printf("control board is kduino\n");

  rosParamInit();

  dErrPosCurrPitch = 0;
  dErrPosCurrRoll = 0;
  dErrPosCurrThrottle = 0;
  dErrPosCurrYaw = 0;

  dErrVelCurrRoll = 0;
  dErrVelCurrPitch = 0;
  dErrVelCurrYaw = 0;
  dErrVelCurrThrottle = 0;
  dErrVelPrevRoll = 0;
  dErrVelPrevPitch = 0;
  dErrVelPrevYaw = 0;
  dErrVelPrevThrottle = 0;

  posITermRoll = 0;
  posITermPitch = 0;
  posITermYaw = 0;
  posITermThrottle = 0;
  posPTermRoll = 0;
  posPTermPitch = 0;
  posPTermYaw = 0;
  posPTermThrottle = 0;
  posDTermRoll = 0;
  posDTermPitch = 0;
  posDTermYaw = 0;
  posDTermThrottle = 0;

  pitchCtrlCnt = 0; rollCtrlCnt = 0; throttleCtrlCnt = 0; yawCtrlCnt = 0;

  //publish
  pidPub_ = controllerNodeHandle_.advertise<jsk_quadcopter::fourAxisPidDebug>("debug", 10); 

  //subscribe
  motorBiasFlag = false;
  motorBiasSetPub_ = controllerNodeHandle_.advertise<jsk_quadcopter_common::ITermBias>("/kduino/i_term_bias", 2);
  motorBiasSetSub_ = controllerNodeHandle_.subscribe<std_msgs::Int8>("/teleop_command/motor_bais_set", 1, &PidController::motorBiasSetCallback, this, ros::TransportHints().tcpNoDelay());
  rpyCtrlOffsetSub_ = controllerNodeHandle_.subscribe<jsk_quadcopter_common::RPYCtrlOffset>("/flight_control/rpy_ctrl_offset", 1, &PidController::rpyCtrlOffsetCallback, this, ros::TransportHints().tcpNoDelay());

  //dynamic reconfigure server
  pitchServer = new dynamic_reconfigure::Server<jsk_quadcopter::PidPitchControlConfig>(ros::NodeHandle(controllerNodeHandlePrivate_, "pitch"));
  dynamicReconfFuncPitch = boost::bind(&PidController::cfgPitchCallback, this, _1, _2);
  pitchServer->setCallback(dynamicReconfFuncPitch);

  rollServer = new dynamic_reconfigure::Server<jsk_quadcopter::PidRollControlConfig>(ros::NodeHandle(controllerNodeHandlePrivate_,"roll"));
  dynamicReconfFuncRoll = boost::bind(&PidController::cfgRollCallback, this, _1, _2);
  rollServer->setCallback(dynamicReconfFuncRoll);

  yawServer = new dynamic_reconfigure::Server<jsk_quadcopter::PidYawControlConfig>(ros::NodeHandle(controllerNodeHandlePrivate_, "yaw"));
  dynamicReconfFuncYaw = boost::bind(&PidController::cfgYawCallback, this, _1, _2);
  yawServer->setCallback(dynamicReconfFuncYaw);

  throttleServer = new dynamic_reconfigure::Server<jsk_quadcopter::PidThrottleControlConfig>(ros::NodeHandle(controllerNodeHandlePrivate_, "throttle"));
  dynamicReconfFuncThrottle = boost::bind(&PidController::cfgThrottleCallback, this, _1, _2);
  throttleServer->setCallback(dynamicReconfFuncThrottle);
}

PidController::~PidController()
{
  printf(" deleted pid flight control input!\n");
}

void PidController::motorBiasSetCallback(const std_msgs::Int8ConstPtr& set_flag)
{

  jsk_quadcopter_common::ITermBias msg;
  msg.bias[0] = (short)(-posITermRoll - offsetRoll_); //反転
  msg.bias[1] = (short)(posITermPitch + offsetPitch_);
  motorBiasSetPub_.publish(msg);

  motorBiasFlag = true;
  posITermRoll = 0;
  posITermPitch = 0;

}

void PidController::rpyCtrlOffsetCallback(const jsk_quadcopter_common::RPYCtrlOffsetConstPtr& offset_value)
{
  offsetRoll_ = offset_value->roll_offset;
  offsetPitch_ = offset_value->pitch_offset;

  posITermRoll = 0;
  posITermPitch = 0;
}


void PidController::pidFunction(Navigator* navigator, Estimator* estimator,
				FlightCtrlInput* flight_ctrl_input)
{
  static bool first_flag = true;
  static bool start_free_fall = false;

  //for rocket start
  static int rocketStartValueTmp = rocketStartInitValue_ - offsetThrottle_;
  static int restartCnt = 0;

  //*** リセット
  if (navigator_->getFlightMode() == Navigator::RESET_MODE) 
    {
      first_flag = true;
      pitchCtrlCnt = 0; rollCtrlCnt = 0; throttleCtrlCnt = 0; yawCtrlCnt = 0;
      posPTermPitch = 0; posITermPitch = 0; posDTermPitch = 0;
      posPTermRoll = 0; posITermRoll = 0; posDTermRoll = 0;
      posPTermYaw = 0; posITermYaw = 0; posDTermYaw = 0;
      posPTermThrottle = 0; posITermThrottle = 0; posDTermThrottle = 0;
      flight_ctrl_input->setPitchValue(0);
      flight_ctrl_input->setRollValue(0);
      flight_ctrl_input->setYawValue(0);
      flight_ctrl_input->setThrottleValue(0);

      //no necessary?
      navigator->setTargetPosX(estimator->getStatePosX());
      navigator->setTargetPosY(estimator->getStatePosY());
      navigator->setTargetPsi(estimator->getStatePsiBody());

      restartCnt++;
      rocketStartValueTmp
        = rocketStartInitValue_ - offsetThrottle_ + rocketStartInitIncrementValue_ * restartCnt;

      start_free_fall = false;
      return;
    }
  else if(navigator->getFlightMode() == NO_CONTROL_MODE)
    {
      return;
    }
  else
    {
      float statePosX;
      float stateVelX;
      float statePosY;
      float stateVelY;

      float targetPosX;
      float targetPosY;
      float targetVelX;
      float targetVelY;

      if(navigator->getXyControlMode() == navigator->POS_WORLD_BASED_CONTROL_MODE ||
         navigator->getXyControlMode() == navigator->VEL_WORLD_BASED_CONTROL_MODE) 
        {
          statePosX = estimator->getStatePosX();
          stateVelX = estimator->getStateVelX();
          statePosY = estimator->getStatePosY();
          stateVelY = estimator->getStateVelY();
        }
      else if(navigator->getXyControlMode() == navigator->POS_LOCAL_BASED_CONTROL_MODE)
        {
          statePosX = estimator->getStatePosX();
          statePosY = estimator->getStatePosY();
          stateVelX = estimator->getStateVelXc();
          stateVelY = estimator->getStateVelYc();
        }
      else if(navigator->getXyControlMode() == navigator->VEL_LOCAL_BASED_CONTROL_MODE)
        {
          statePosX = estimator->getStatePosXc();
          statePosY = estimator->getStatePosYc();
          stateVelX = estimator->getStateVelXc();
          stateVelY = estimator->getStateVelYc();
        }
      else
        {
          ROS_ERROR(" bad ");
          statePosX = 0;
          stateVelX = 0;
          statePosY = 0;
          stateVelY = 0;
        }

      targetVelX = navigator->getTargetVelX();
      targetVelY = navigator->getTargetVelY();
      targetPosX = navigator->getTargetPosX();
      targetPosY = navigator->getTargetPosY();

      float statePosZ = estimator->getStatePosZ();
      float stateVelZ = estimator->getStateVelZ();
      float statePsiCog = estimator->getStatePsiCog();
      float statePsiBody = estimator->getStatePsiBody();
      float stateVelPsiBody = estimator->getStateVelPsiBody();

      float targetPosZ = navigator->getTargetPosZ();
      float targetPsi = navigator->getTargetPsi();

      jsk_quadcopter::fourAxisPidDebug  fourAxisPidDebug_;

      fourAxisPidDebug_.header.stamp = ros::Time::now();

      if (first_flag) 
	first_flag = false;
      else
	{
          //pitch
          if(navigator->getXyControlMode() == navigator->POS_WORLD_BASED_CONTROL_MODE)
            {
              //** 座標変換
              dErrPosCurrPitch 
                = (targetPosX - statePosX) * cos(statePsiCog)
                + (targetPosY - statePosY) * sin(statePsiCog);

              dErrVelCurrPitch = - (stateVelX * cos(statePsiCog) + stateVelY * sin(statePsiCog)); 

              //**** Pの項
              posPTermPitch =
                limit(1000 * posPGainPitch_ * dErrPosCurrPitch,  posPLimitPitch_);

              //**** Iの項
              if(navigator->getFlightMode() == TAKEOFF_MODE || navigator->getFlightMode() == LAND_MODE) //takeoff or land
                posITermPitch += 1000 * 
                  dErrPosCurrPitch * (1 / (float)pitchCtrlLoopRate_) * posIGainPitch_;
              else if(navigator->getFlightMode() == FLIGHT_MODE &&
                      fabs(dErrPosCurrPitch) < iEnableLimitPitch_) //hover or land
                posITermPitch += 1000 * 
                  dErrPosCurrPitch * (1 / (float)pitchCtrlLoopRate_) * posIGainPitchHover_;
              posITermPitch = limit(posITermPitch, posILimitPitch_);

              //***** Dの項
              posDTermPitch = 
                limit(1000 * posDGainPitch_ * dErrVelCurrPitch, posDLimitPitch_);

            }
          else if(navigator->getXyControlMode() == navigator->POS_LOCAL_BASED_CONTROL_MODE)
            {
              //** 座標変換
              dErrPosCurrPitch 
                = (targetPosX - statePosX) * cos(statePsiCog)
                + (targetPosY - statePosY) * sin(statePsiCog);

              dErrVelCurrPitch = targetVelX - stateVelX;

              //**** Pの項
              posPTermPitch =
                limit(1000 * posPGainPitch_ * dErrPosCurrPitch, posPLimitPitch_);
              //**** Dの項
              posDTermPitch =
                limit(1000 * posDGainPitch_ * dErrVelCurrPitch, posDLimitPitch_);

            }
          else if(navigator->getXyControlMode() == navigator->VEL_WORLD_BASED_CONTROL_MODE)
            {
              //** 座標変換
#if 0 // for body coord base
              dErrVelCurrPitch =
                targetVelX - (stateVelX * cos(statePsiCog) + stateVelY * sin(statePsiCog)); 
#else //for world coord base
              dErrVelCurrPitch 
                = (targetVelX - stateVelX) * cos(statePsiCog) 
                + (targetVelY - stateVelY) * sin(statePsiCog); 
#endif
              //**** Pの項
              posPTermPitch =
                limit(1000 * velPGainPitch_ * dErrVelCurrPitch, posDLimitPitch_);
              posDTermPitch =  0;
            }
          else if(navigator->getXyControlMode() == navigator->VEL_LOCAL_BASED_CONTROL_MODE)
            {
              if(navigator->getFlightMode() == TAKEOFF_MODE)
                {// special mode for velocity control
                  //#if 1 // flight pid control for takeoff
                  if(navigator->getXyVelModePosCtrlTakeoff())
                    {
                      //**** Pの項
                      dErrPosCurrPitch = targetPosX - statePosX;
                      dErrVelCurrPitch = targetVelX - stateVelX;
                      //dErrVelCurrPitch = - stateVelX;

                      posPTermPitch =
                        limit( 1000 * posPGainPitch_ * dErrPosCurrPitch,  posPLimitPitch_);

                      posITermPitch += 1000 * 
                        dErrPosCurrPitch * (1 / (float)pitchCtrlLoopRate_) * posIGainPitch_;
                      posITermPitch = limit(posITermPitch, posILimitPitch_);

                      posDTermPitch = 
                        limit(1000 * posDGainPitch_ * dErrVelCurrPitch, posDLimitPitch_);
                    }
                  else
                    {
                      dErrVelCurrPitch = targetVelX - stateVelX;
                      //**** Pの項
                      posPTermPitch =
                        limit(1000 * velPGainPitch_ * dErrVelCurrPitch, posDLimitPitch_);
                      //**** Iの項
                      posITermPitch = 0;
                      //**** Dの項
                      posDTermPitch = 0;
                    }
                  //#endif
                }
              else
                { // other flight mode except takeoff
                  dErrVelCurrPitch = targetVelX - stateVelX;
                  //**** Pの項
                  posPTermPitch =
                    limit(1000 * velPGainPitch_ * dErrVelCurrPitch, posDLimitPitch_);

                  //**** Dの項
                  posDTermPitch = 0;
                }

            }
          else
            ROS_ERROR("wrong control mode");

          if(navigator->getFreeFallFlag())
            {// free fall mode
              posPTermPitch =  0;
              posITermPitch =  0;
              posDTermPitch =  0;
            }

          //*** motor bias
          if(motorBiasFlag)
            posITermPitch = 0;

          //*** 指令値算出
          short pitch_value = limit(posPTermPitch + posITermPitch + posDTermPitch 
                                    + offsetPitch_, posLimitPitch_);
          //**** 指令値反転
          if( controlBoard_ == ASCTEC_BOARD) //asctec
            pitch_value = - pitch_value;

          //*** 指令値代入
          flight_ctrl_input->setPitchValue(pitch_value);

          //**** ros pub
          fourAxisPidDebug_.pitch.total = pitch_value;
          fourAxisPidDebug_.pitch.pTerm = posPTermPitch;
          fourAxisPidDebug_.pitch.iTerm = posITermPitch;
          fourAxisPidDebug_.pitch.dTerm = posDTermPitch;
          fourAxisPidDebug_.pitch.posErrTransform = dErrPosCurrPitch;
          fourAxisPidDebug_.pitch.posErrNoTransform = targetPosX - statePosX;
          fourAxisPidDebug_.pitch.velErrTransform = targetVelX;
          fourAxisPidDebug_.pitch.velErrNoTransform = targetVelX -stateVelX;
                

          //roll
          if(navigator->getXyControlMode() == navigator->POS_WORLD_BASED_CONTROL_MODE)
            {
              //** 座標変換
              dErrPosCurrRoll 
                = - (targetPosX - statePosX) * sin(statePsiCog)
                + (targetPosY - statePosY) * cos(statePsiCog);

              dErrVelCurrRoll = - (- stateVelX * sin(statePsiCog) + stateVelY * cos(statePsiCog));

              //**** Pの項
              posPTermRoll =  
                limit(1000 * posPGainRoll_ * dErrPosCurrRoll, posPLimitRoll_);

              //**** Iの項
              if(navigator->getFlightMode() == TAKEOFF_MODE || navigator->getFlightMode() == LAND_MODE) //takeoff or land
                posITermRoll += 1000 * 
                  dErrPosCurrRoll * (1 / (float)rollCtrlLoopRate_) * posIGainRoll_;
              else if(navigator->getFlightMode() == FLIGHT_MODE &&
                      fabs(dErrPosCurrRoll) < iEnableLimitRoll_) //hover
                posITermRoll += 1000 * 
                  dErrPosCurrRoll * (1 / (float)rollCtrlLoopRate_) * posIGainRollHover_;
              posITermRoll = limit(posITermRoll, posILimitRoll_);

              //***** Dの項
              posDTermRoll = 
                limit(1000 * posDGainRoll_ * dErrVelCurrRoll, posDLimitRoll_);
            }
          else if(navigator->getXyControlMode() == navigator->POS_LOCAL_BASED_CONTROL_MODE)
            {
              //** 座標変換
              dErrPosCurrRoll 
                = - (targetPosX - statePosX) * sin(statePsiCog)
                + (targetPosY - statePosY) * cos(statePsiCog);

              dErrVelCurrRoll = targetVelY - stateVelY;

              //**** Pの項
              posPTermRoll =  
                limit(1000 * posPGainRoll_ * dErrPosCurrRoll, posPLimitRoll_);

              //**** Dの項
              posDTermRoll =
                limit(1000 * posDGainRoll_ * dErrVelCurrRoll, posDLimitRoll_);

            }
          else if(navigator->getXyControlMode() == navigator->VEL_WORLD_BASED_CONTROL_MODE)
            {
#if 0 //for body coord base
              dErrVelCurrRoll =
                targetVelY - (- stateVelX * sin(statePsiCog) + stateVelY * cos(statePsiCog));
#else 
              dErrVelCurrRoll 
                = -(targetVelX - stateVelX) * sin(statePsiCog)
                + (targetVelY - stateVelY)  * cos(statePsiCog);
#endif
              //**** Pの項
              posPTermRoll =
                limit(1000 * velPGainRoll_ * dErrVelCurrRoll, posDLimitRoll_);
              posDTermRoll = 0;
            }
          else if(navigator->getXyControlMode() == navigator->VEL_LOCAL_BASED_CONTROL_MODE)
            {
              if(navigator->getFlightMode() == TAKEOFF_MODE)
                {// special mode for velocity control
                  //#if 1 // flight pid control for takeoff
                  if(navigator->getXyVelModePosCtrlTakeoff())
                    {
                      //**** Pの項
                      dErrPosCurrRoll = targetPosY - statePosY;
                      dErrVelCurrRoll = targetVelY - stateVelY;

                      posPTermRoll =
                        limit( 1000 * posPGainRoll_ * dErrPosCurrRoll,  posPLimitRoll_);

                      posITermRoll += 1000 * 
                        dErrPosCurrRoll * (1 / (float)rollCtrlLoopRate_) * posIGainRoll_;
                      posITermRoll = limit(posITermRoll, posILimitRoll_);

                      posDTermRoll = 
                        limit(1000 * posDGainRoll_ * dErrVelCurrRoll, posDLimitRoll_);
                    }
                  //#else //velocity pid control for takeoff
                  else
                    {
                      dErrVelCurrRoll = targetVelY - stateVelY;
                      //**** Pの項
                      posPTermRoll =
                        limit(1000 * velPGainRoll_ * dErrVelCurrRoll, posDLimitRoll_);
                      //**** Iの項
                      posITermRoll = 0;
                      //**** Dの項
                      posDTermRoll = 0;
                    }
                  //#endif

                }
              else
                {
                  dErrVelCurrRoll = targetVelY - stateVelY;
                  //**** Pの項
                  posPTermRoll =
                    limit(1000 * velPGainRoll_ * dErrVelCurrRoll, posDLimitRoll_);

                  //**** Iの項
                  //1)+*+*+ no i term 
                  //posITermPitch = 0;
                  //2)+*+*+ inherit the takeoff iterm

                  //**** Dの項
                  posDTermRoll = 0;
                }
            }
          else
            ROS_ERROR("wrong control mode");

          if(navigator->getFreeFallFlag())
            {// free fall mode
              posPTermRoll =  0;
              posITermRoll =  0;
              posDTermRoll =  0;
            }

          //*** motor bias
          if(motorBiasFlag)
            posITermRoll = 0;

          //*** 指令値算出
          short roll_value = limit(posPTermRoll + posITermRoll + posDTermRoll 
                                   + offsetRoll_, posLimitRoll_);
          //**** 指令値反転
          roll_value = - roll_value;

          //*** 指令値代入
          flight_ctrl_input->setRollValue(roll_value);

          //**** ros pub
          fourAxisPidDebug_.roll.total = roll_value;
          fourAxisPidDebug_.roll.pTerm = posPTermRoll;
          fourAxisPidDebug_.roll.iTerm = posITermRoll;
          fourAxisPidDebug_.roll.dTerm = posDTermRoll;
          fourAxisPidDebug_.roll.posErrTransform = dErrPosCurrRoll;
          fourAxisPidDebug_.roll.posErrNoTransform = targetPosY - statePosY;
          fourAxisPidDebug_.roll.velErrTransform = targetVelY;
          fourAxisPidDebug_.roll.velErrNoTransform = targetVelY - stateVelY;


          //yaw
          if(navigator->getXyControlMode() == navigator->POS_WORLD_BASED_CONTROL_MODE ||
             navigator->getXyControlMode() == navigator->POS_LOCAL_BASED_CONTROL_MODE || 
             navigator->getXyControlMode() == navigator->VEL_WORLD_BASED_CONTROL_MODE)
            {
              dErrPosCurrYaw = targetPsi - statePsiBody;
              //極座標の補正
              if(dErrPosCurrYaw > M_PI)  dErrPosCurrYaw = -2 * M_PI + dErrPosCurrYaw; 
              else if(dErrPosCurrYaw < -M_PI)  dErrPosCurrYaw = 2 * M_PI + dErrPosCurrYaw;

              //**** Pの項
              posPTermYaw = 
                limit(1000 * posPGainYaw_ * dErrPosCurrYaw, posPLimitYaw_);

              //**** Iの項 : deprecated
#if 1 //bad for hydra control
              if(fabs(dErrPosCurrYaw) < iEnableLimitYaw_) 
                posITermYaw += 1000 *
                  dErrPosCurrYaw * ( 1 / (float)yawCtrlLoopRate_ ) * posIGainYaw_;
              else{ posITermYaw = 0;} // right ?
#else 
              posITermYaw += 1000 *
                dErrPosCurrYaw * ( 1 / (float)yawCtrlLoopRate_ ) * posIGainYaw_;
#endif

              posITermYaw= limit(posITermYaw, posILimitYaw_);

              //***** Dの項
              posDTermYaw = 
                limit(- 1000 * posDGainYaw_ * stateVelPsiBody, posDLimitYaw_);
            }
          else if(navigator->getXyControlMode() == navigator->VEL_LOCAL_BASED_CONTROL_MODE)
            {
              //fixed point  getStatePsi()
              //develop the p term yaw
              posPTermYaw = limit(1000 * targetPsi, posPLimitYaw_);
              posITermYaw = 0;
              posDTermYaw = 0;
            }

          //*** 指令値算出
          short yaw_value = limit(posPTermYaw + posITermYaw + posDTermYaw,
                                  posLimitYaw_);
          //**** 指令値反転
          // critical point
          //if( controlBoard_ == ASCTEC_BOARD || controlBoard_ == KDUINO_BOARD) // both
          if( controlBoard_ == ASCTEC_BOARD) //only asctec
            yaw_value = - yaw_value;
	
          //**** ros pub
          fourAxisPidDebug_.yaw.total = yaw_value;
          fourAxisPidDebug_.yaw.pTerm = posPTermYaw;
          fourAxisPidDebug_.yaw.iTerm = posITermYaw;
          fourAxisPidDebug_.yaw.dTerm = posDTermYaw;
          fourAxisPidDebug_.yaw.posErrTransform = targetPsi;
          fourAxisPidDebug_.yaw.posErrNoTransform = dErrPosCurrYaw;
          fourAxisPidDebug_.yaw.velErrTransform = stateVelPsiBody;
          fourAxisPidDebug_.yaw.velErrNoTransform = stateVelPsiBody;

          //*** 指令値代入
          flight_ctrl_input->setYawValue(yaw_value);
          //flight_ctrl_input->setYawValue(0);

          //throttle
          dErrPosCurrThrottle = targetPosZ - statePosZ;

          if(navigator->getFlightMode() == TAKEOFF_MODE)
            {
              if(estimator->getRocketStartFlag())
                {
                  ROS_INFO("rocket start");
                  posPTermThrottle = 0;
                  //increment rocket start
                  rocketStartValueTmp += rocketStartStepValue_;
                  posITermThrottle = rocketStartValueTmp;
                  posDTermThrottle = 0;
                }
              else
                {
                  //ROS_INFO("normal takeoff");
                  //**** Pの項
                  posPTermThrottle = 
                    limit(1000 * posPGainThrottle_ * dErrPosCurrThrottle, posPLimitThrottle_);
                  //**** Iの項
                  posITermThrottle += 1000 *
                    dErrPosCurrThrottle * ( 1 / (float)throttleCtrlLoopRate_) * posIGainThrottle_;
                  posITermThrottle = limit(posITermThrottle, posILimitThrottle_);
                  //***** Dの項
                  posDTermThrottle = 
                    limit(-1000 * posDGainThrottle_ * stateVelZ, posDLimitThrottle_);
                }
            }
          else if(navigator->getFlightMode() == FLIGHT_MODE) //hover
            {
              //**** Pの項
              posPTermThrottle = 
                limit(1000 * posPGainThrottle_ * dErrPosCurrThrottle, posPLimitThrottleHover_);
              //**** Iの項
#if 1     // without limit
              posITermThrottle += 1000 *
                dErrPosCurrThrottle * ( 1 / (float)throttleCtrlLoopRate_) * posIGainThrottle_;
              posITermThrottle = limit(posITermThrottle, posILimitThrottle_);
#else 	  // iEnableLimitThrottleHover :0.1
              if(fabs(dErrPosCurrThrottle) < iEnableLimitThrottleHover_) 
                {
                  posITermThrottle += 1000 *
                    dErrPosCurrThrottle * ( 1 / (float)throttleCtrlLoopRate_) * posIGainThrottle_;
                  posITermThrottle = limit(posITermThrottle, posILimitThrottle_);
                }
#endif
              //***** Dの項
              posDTermThrottle = 
                limit(-1000 * posDGainThrottle_ * stateVelZ, posDLimitThrottle_);
            }
          else if(navigator->getFlightMode() == LAND_MODE)
            {
              ROS_WARN(" land mode");

              if(navigator->getFreeFallFlag())
                {//free fall mode from free fall threshold

                  if(!start_free_fall)
                    { //first time
                      short posITermThrottleTmp = - offsetThrottle_ + limit(posPTermThrottle +
                                                                            posITermThrottle + 
                                                                            posDTermThrottle + 
                                                                            offsetThrottle_,
                                                                            posLimitThrottle_);
                      posITermThrottle = posITermThrottleTmp;
                      start_free_fall  = true;
                    }
                  posPTermThrottle = 0;
                  posITermThrottle -= freeFallStepValue_;
                  posDTermThrottle = 0;

                  if(posITermThrottle + offsetThrottle_ < motorStopValue_)
                    {
                      bool tmp = true;
                      navigator->setMotorStopFlag(tmp);
                      ROS_WARN("  stop motor ");
                    }
                }
              else
                {
                  ROS_WARN(" no free fall mode");

                  //**** Pの項
                  //TODO: dErrPosCurrThrottle < -0.1 or > -0.1 ; Original is < -0.1
                  //constPcontrolThreLand : -0.1m
                  //constPTermLev1ValueLand : -50
                  //constPTermLev2ValueLand : -90
                  if(dErrPosCurrThrottle < constPControlThreThrottleLand_) 
                    posPTermThrottle = constPTermLev1ValueThrottleLand_; 
                  else  posPTermThrottle = constPTermLev2ValueThrottleLand_;

                  //**** Iの項
                  // constIcontrolThreLand : -0.25m
                  if(dErrPosCurrThrottle > constIControlThreThrottleLand_)
                    posITermThrottle += 1000 * dErrPosCurrThrottle *
                      (1 / (float)throttleCtrlLoopRate_) * posIGainThrottleLand_;
                  else posITermThrottle -= constITermValueThrottleLand_;

                  //***** Dの項
                  posDTermThrottle =
                    limit(-1000 * posDGainThrottleLand_ * stateVelZ, posDLimitThrottle_);
                }
            }
          //throwing mode function
          throttleThrowingMode(navigator, estimator);

          //*** 指令値算出
          short throttle_value = limit(posPTermThrottle + posITermThrottle 
                                       + posDTermThrottle + offsetThrottle_, 
                                       posLimitThrottle_);
          //**** 指令値反転
          //throttle_value = - throttle_value;
	  
          //*** 指令値代入
          flight_ctrl_input->setThrottleValue(throttle_value);

          //**** ros pub
          fourAxisPidDebug_.throttle.total = throttle_value;
          fourAxisPidDebug_.throttle.pTerm = posPTermThrottle;
          fourAxisPidDebug_.throttle.iTerm = posITermThrottle;
          fourAxisPidDebug_.throttle.dTerm = posDTermThrottle;
          fourAxisPidDebug_.throttle.posErrTransform = targetPosZ;
          fourAxisPidDebug_.throttle.posErrNoTransform = dErrPosCurrThrottle;
          fourAxisPidDebug_.throttle.velErrTransform = stateVelZ;
          fourAxisPidDebug_.throttle.velErrNoTransform = stateVelZ;




	  pidPub_.publish(fourAxisPidDebug_);
	}

      //*** 更新
      //previous_secs = current_secs;
      dErrVelPrevPitch =  dErrVelCurrPitch;
      dErrVelPrevRoll =  dErrVelCurrRoll;
      dErrVelPrevThrottle = dErrVelCurrThrottle;
      dErrVelPrevYaw =  dErrVelCurrYaw;

      return;
    }
}

void PidController::feedForwardFunction(Navigator* navigator, Estimator* estimator,
                                        FlightCtrlInput* flight_ctrl_input)
{
  //do nothing
}


void PidController::throttleThrowingMode(Navigator* navigator, Estimator* estimator)
{
  static bool first_flag = true;
  if(navigator->getThrowingMode() == navigator->THROWING_START_ALTHOLD)
    {
      if(first_flag)
        {
          first_flag = false;
          posITermThrottle += (rocketStartInitValue_ - offsetThrottle_  + throwingModeInitValueFromRocketStart_); //param
          //posITermThrottle = limit(posITermThrottle, posILimitThrottle_);

          //debug
          ROS_ERROR("first time: posITermThtottle init");
        }

#if 1
      //**** Pの項
      posPTermThrottle = 
        limit(1000 * posPGainThrottle_ * dErrPosCurrThrottle, posPLimitThrottle_);
      //**** Iの項
      posITermThrottle += 1000 *
        dErrPosCurrThrottle * ( 1 / (float)throttleCtrlLoopRate_) * posIGainThrottle_;
      posITermThrottle = limit(posITermThrottle, posILimitThrottle_);
      //***** Dの項
      posDTermThrottle = 
        limit(-1000 * posDGainThrottle_ * estimator->getStateVelZ(), posDLimitThrottle_);

      ROS_ERROR("ok, dErrPosCurrThrottle: %f",dErrPosCurrThrottle );

#endif
    }
}

void PidController::cfgPitchCallback(jsk_quadcopter::PidPitchControlConfig &config, uint32_t level)
{

  if(config.pidControlFlag)
    {
      printf("Pitch Dynamic:");
      switch(level)
        {
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_CONTROL_LOOP_RATE:
          pitchCtrlLoopRate_ = config.ctrlLoopRate;
          printf("change the control loop rate\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN:
          posPGainPitch_ = config.posPGain;
          printf("change the pos p gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN:
          posIGainPitch_ = config.posIGain;
          printf("change the pos i gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN_HOVER:
          posIGainPitchHover_ = config.posIGainHover;
          printf("change the posi_hover gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN:
          posDGainPitch_ = config.posDGain;
          printf("change the pos d gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_VEL_P_GAIN:
          velPGainPitch_ = config.velPGain;
          printf("change the vel p gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_VEL_I_GAIN:
          velIGainPitch_ = config.velIGain;
          printf("change the vel i gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_OFFSET:
          offsetPitch_ = config.offset;
          printf("change the offset\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_LIMIT:
          posLimitPitch_ = config.posLimit;
          printf("change the limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT:
          posPLimitPitch_ = config.posPLimit;
          printf("change the p limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_LIMIT:
          posILimitPitch_ = config.posILimit;
          printf("change the i limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_D_LIMIT:
          posDLimitPitch_ = config.posDLimit;
          printf("change the d limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_VEL_VALUE_LIMIT_HOVER:
          velValueLimitPitch_ = config.velValueLimit;
          printf("change the vel value limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_I_ENABLE_LIMIT_HOVER:
          iEnableLimitPitch_ = config.iEnableLimit;
          printf("change the i enable limit\n");
          break;
        default :
          printf("\n");
          break;
        }
    }

}

void PidController::cfgRollCallback(jsk_quadcopter::PidRollControlConfig &config, uint32_t level)
{


  if(config.pidControlFlag)
    {
      printf("Roll Dynamic:");
      switch(level)
        {
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_CONTROL_LOOP_RATE:
          rollCtrlLoopRate_ = config.ctrlLoopRate;
          printf("change the control loop rate\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN:
          posPGainRoll_ = config.posPGain;
          printf("change the pos p gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN:
          posIGainRoll_ = config.posIGain;
          printf("change the pos i gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN_HOVER:
          posIGainRollHover_ = config.posIGainHover;
          printf("change the pos i_hover gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN:
          posDGainRoll_ = config.posDGain;
          printf("change the pos d gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_VEL_P_GAIN:
          velPGainRoll_ = config.velPGain;
          printf("change the vel p gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_VEL_I_GAIN:
          velIGainRoll_ = config.velIGain;
          printf("change the vel i gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_OFFSET:
          offsetRoll_ = config.offset;
          printf("change the offset\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_LIMIT:
          posLimitRoll_ = config.posLimit;
          printf("change the limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT:
          posPLimitRoll_ = config.posPLimit;
          printf("change the p limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_LIMIT:
          posILimitRoll_ = config.posILimit;
          printf("change the i limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_D_LIMIT:
          posDLimitRoll_ = config.posDLimit;
          printf("change the d limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_VEL_VALUE_LIMIT_HOVER:
          velValueLimitRoll_ = config.velValueLimit;
          printf("change the vel value limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_I_ENABLE_LIMIT_HOVER:
          iEnableLimitRoll_ = config.iEnableLimit;
          printf("change the i enable limit\n");
          break;
        default :   
          printf("\n");
          break;
        }
    }
}

void PidController::cfgYawCallback(jsk_quadcopter::PidYawControlConfig &config, uint32_t level)
{
  if(config.pidControlFlag)
    {
      printf("Yaw Dynamic:");
      switch(level)
        {
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_CONTROL_LOOP_RATE:
          yawCtrlLoopRate_ = config.ctrlLoopRate;
          printf("change the control loop rate\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN:
          posPGainYaw_ = config.posPGain;
          printf("change the p gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN:
          posIGainYaw_ = config.posIGain;
          printf("change the i gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN:
          posDGainYaw_ = config.posDGain;
          printf("change the d gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_LIMIT:
          posLimitYaw_ = config.posLimit;
          printf("change the limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT:
          posPLimitYaw_ = config.posPLimit;
          printf("change the p limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_LIMIT:
          posILimitYaw_ = config.posILimit;
          printf("change the i limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_D_LIMIT:
          posDLimitYaw_ = config.posDLimit;
          printf("change the d limit\n");
          break;
        default :
          printf("\n");
          break;
        }
    }
}

void PidController::cfgThrottleCallback(jsk_quadcopter::PidThrottleControlConfig &config, uint32_t level)
{

  if(config.pidControlFlag)
    {
      printf("Throttle Dynamic:");
      switch(level)
        {
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_CONTROL_LOOP_RATE:
          throttleCtrlLoopRate_ = config.ctrlLoopRate;
          printf("change the control loop rate\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN:
          posPGainThrottle_ = config.posPGain;
          printf("change the p gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN:
          posIGainThrottle_ = config.posIGain;
          printf("change the i gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN:
          posDGainThrottle_ = config.posDGain;
          printf("change the d gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN_LAND:
          posPGainThrottleLand_ = config.posPGainLand;
          printf("change the p gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN_LAND:
          posIGainThrottleLand_ = config.posIGainLand;
          printf("change the i gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN_LAND:
          posDGainThrottleLand_ = config.posDGainLand;
          printf("change the d gain\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_CONST_P_CONTROL_THRESHOLD_LAND:
          constPControlThreThrottleLand_ = config.constPControlThreLand;
          printf("change const p control threshold throttle land\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_CONST_P_TERM_LEVEL1_VALUE_LAND:
          constPTermLev1ValueThrottleLand_ = config.constPTermLev1ValueLand;
          printf("change const p term level1 value throttle land\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_CONST_P_TERM_LEVEL2_VALUE_LAND:
          constPTermLev2ValueThrottleLand_ = config.constPTermLev2ValueLand;
          printf("change const p term level2 value throttle land\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_CONST_I_CONTROL_THRESHOLD_LAND:
          constIControlThreThrottleLand_ = config.constIControlThreLand;
          printf("change const i control threshold throttle land\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_CONST_I_TERM_VALUE_LAND:
          constITermValueThrottleLand_ = config.constITermValueLand;
          printf("change const i term value throttle land\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_OFFSET:
          offsetThrottle_ = config.offset;
          printf("change the offset\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_LIMIT:
          posLimitThrottle_ = config.posLimit;
          printf("change the limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT:
          posPLimitThrottle_ = config.posPLimit;
          printf("change the p limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT_HOVER:
          posPLimitThrottleHover_ = config.posPLimitHover;
          printf("change the p hover limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_I_LIMIT:
          posILimitThrottle_ = config.posILimit;
          printf("change the i limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_POS_D_LIMIT:
          posDLimitThrottle_ = config.posDLimit;
          printf("change the d limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_VEL_VALUE_LIMIT_HOVER:
          velValueLimitThrottleHover_ = config.velValueLimitHover;
          printf("change the vel value limit\n");
          break;
        case jsk_quadcopter_common::DynamicReconfigureLevels::RECONFIGURE_I_ENABLE_LIMIT_HOVER:
          iEnableLimitThrottleHover_ = config.iEnableLimitHover;
          printf("change the i enable limit\n");
          break;
        default :
          printf("\n");
          break;
        }
    }
}

void PidController::rosParamInit()
{
  ros::NodeHandle pitch_node(controllerNodeHandlePrivate_, "pitch");
  ros::NodeHandle roll_node(controllerNodeHandlePrivate_, "roll");
  ros::NodeHandle throttle_node(controllerNodeHandlePrivate_, "throttle");
  ros::NodeHandle yaw_node(controllerNodeHandlePrivate_, "yaw");

  std::string pitch_ns = pitch_node.getNamespace();
  std::string roll_ns = roll_node.getNamespace();
  std::string throttle_ns = throttle_node.getNamespace();
  std::string yaw_ns = yaw_node.getNamespace();

  //**** throttle
  if (!throttle_node.getParam ("ctrlLoopRate", throttleCtrlLoopRate_))
    throttleCtrlLoopRate_ = 0;
  printf("%s: ctrlLoopRate_ is %d\n", throttle_ns.c_str(), throttleCtrlLoopRate_);

  if (!throttle_node.getParam ("posPGain", posPGainThrottle_))
    posPGainThrottle_ = 0;
  printf("%s: posPGain_ is %.3f\n", throttle_ns.c_str(), posPGainThrottle_);

  if (!throttle_node.getParam ("posIGain", posIGainThrottle_))
    posIGainThrottle_ = 0;
  printf("%s: posIGain_ is %.3f\n", throttle_ns.c_str(), posIGainThrottle_);

  if (!throttle_node.getParam ("posDGain", posDGainThrottle_))
    posDGainThrottle_ = 0;
  printf("%s: posDGain_ is %.3f\n", throttle_ns.c_str(), posDGainThrottle_);

  if (!throttle_node.getParam ("posPGainLand", posPGainThrottleLand_))
    posPGainThrottleLand_ = 0;
  printf("%s: posPGainLand_ is %.3f\n", throttle_ns.c_str(), posPGainThrottleLand_);

  if (!throttle_node.getParam ("posIGainLand", posIGainThrottleLand_))
    posIGainThrottleLand_ = 0;
  printf("%s: posIGainLand_ is %.3f\n", throttle_ns.c_str(), posIGainThrottleLand_);

  if (!throttle_node.getParam ("posDGainLand", posDGainThrottleLand_))
    posDGainThrottleLand_ = 0;
  printf("%s: posDGainLand_ is %.3f\n", throttle_ns.c_str(), posDGainThrottleLand_);

  if (!throttle_node.getParam ("constPControlThreLand",  constPControlThreThrottleLand_))
    constPControlThreThrottleLand_ = 0;
  printf("%s: constPControlThreLand_ is %.3f\n", throttle_ns.c_str(), constPControlThreThrottleLand_);

  if(!throttle_node.getParam("constPTermLev1ValueLand",constPTermLev1ValueThrottleLand_))
    constPTermLev1ValueThrottleLand_ = 0;
  printf("%s: constPTermLev1ValueLand_ is %.3f\n", throttle_ns.c_str(), constPTermLev1ValueThrottleLand_);

  if(!throttle_node.getParam("constPTermLev2ValueLand",constPTermLev2ValueThrottleLand_))
    constPTermLev2ValueThrottleLand_ = 0;
  printf("%s: constPTermLev2ValueLand_ is %.3f\n", throttle_ns.c_str(), constPTermLev2ValueThrottleLand_);

  if (!throttle_node.getParam ("constIControlThreLand",  constIControlThreThrottleLand_))
    constIControlThreThrottleLand_ = 0;
  printf("%s: constIControlThreLand_ is %.3f\n", throttle_ns.c_str(), constIControlThreThrottleLand_);

  if (!throttle_node.getParam ("constITermValueLand",  constITermValueThrottleLand_))
    constITermValueThrottleLand_ = 0;
  printf("%s: constITermValueLand_ is %.3f\n", throttle_ns.c_str(), constITermValueThrottleLand_);

  if (!throttle_node.getParam ("offset", offsetThrottle_))
    offsetThrottle_ = 0;
  printf("%s: offset_ is %d\n", throttle_ns.c_str(), offsetThrottle_);

  if (!throttle_node.getParam ("posLimit", posLimitThrottle_))
    posLimitThrottle_ = 0;
  printf("%s: posLimit_ is %d\n", throttle_ns.c_str(), posLimitThrottle_);

  if (!throttle_node.getParam ("posPLimit", posPLimitThrottle_))
    posPLimitThrottle_ = 0;
  printf("%s: posPLimit_ is %d\n", throttle_ns.c_str(), posPLimitThrottle_);

  if (!throttle_node.getParam ("posPLimitHover", posPLimitThrottleHover_))
    posPLimitThrottleHover_ = 0;
  printf("%s: posPLimitHover_ is %d\n", throttle_ns.c_str(), posPLimitThrottleHover_);

  if (!throttle_node.getParam ("posILimit", posILimitThrottle_))
    posILimitThrottle_ = 0;
  printf("%s: posILimit_ is %d\n", throttle_ns.c_str(), posILimitThrottle_);

  if (!throttle_node.getParam ("posDLimit", posDLimitThrottle_))
    posDLimitThrottle_ = 0;
  printf("%s: posDLimit_ is %d\n", throttle_ns.c_str(), posDLimitThrottle_);

  if (!throttle_node.getParam ("velValueLimitHover", velValueLimitThrottleHover_))
    velValueLimitThrottleHover_ = 0;
  printf("%s: velValueLimitHover_ is %.3f\n", throttle_ns.c_str(), velValueLimitThrottleHover_);

  if (!throttle_node.getParam ("iEnableLimitHover", iEnableLimitThrottleHover_))
    iEnableLimitThrottleHover_ = 0;
  printf("%s: iEnableLimitHover_ is %.3f\n", throttle_ns.c_str(), iEnableLimitThrottleHover_);

  if (!throttle_node.getParam ("rocketStartInitValue", rocketStartInitValue_))
    rocketStartInitValue_ = 0;
  printf("%s: rocketStartInitValue_ is %d\n", throttle_ns.c_str(), rocketStartInitValue_);

  if (!throttle_node.getParam ("rocketStartInitIncrementValue", rocketStartInitIncrementValue_))
    rocketStartInitIncrementValue_ = 0;
  printf("%s: rocketStartInitIncrementValue_ is %d\n", throttle_ns.c_str(), rocketStartInitIncrementValue_);

  if (!throttle_node.getParam ("rocketStartStepValue", rocketStartStepValue_))
    rocketStartStepValue_ = 0;
  printf("%s: rocketStartStepValue_ is %d\n", throttle_ns.c_str(), rocketStartStepValue_);

  if (!throttle_node.getParam ("throwingModeInitValueFromRocketStart", throwingModeInitValueFromRocketStart_))
    throwingModeInitValueFromRocketStart_ = 0;
  printf("%s: throwingModeInitValueFromRocketStart_ is %d\n", throttle_ns.c_str(), throwingModeInitValueFromRocketStart_);

  if (!throttle_node.getParam ("freeFallStepValue", freeFallStepValue_))
    freeFallStepValue_ = 0;
  printf("%s: freeFallStepValue_ is %d\n", throttle_ns.c_str(), freeFallStepValue_);

  if (!throttle_node.getParam ("motorStopValue", motorStopValue_))
    motorStopValue_ = 0;
  printf("%s: motorStopValue_ is %d\n", throttle_ns.c_str(), motorStopValue_);

  //**** pitch
  if (!pitch_node.getParam ("ctrlLoopRate", pitchCtrlLoopRate_))
    pitchCtrlLoopRate_ = 0;
  printf("%s: ctrlLoopRate_ is %d\n", pitch_ns.c_str(), pitchCtrlLoopRate_);

  if (!pitch_node.getParam ("posPGain", posPGainPitch_))
    posPGainPitch_ = 0;
  printf("%s: posPGain_ is %.3f\n", pitch_ns.c_str(), posPGainPitch_);

  if (!pitch_node.getParam ("posIGain", posIGainPitch_))
    posIGainPitch_ = 0;
  printf("%s: posIGain_ is %.3f\n", pitch_ns.c_str(), posIGainPitch_);

  if (!pitch_node.getParam ("posIGainHover", posIGainPitchHover_))
    posIGainPitchHover_ = 0;
  printf("%s: posIGainHover_ is %.3f\n", pitch_ns.c_str(), posIGainPitchHover_);

  if (!pitch_node.getParam ("posDGain", posDGainPitch_))
    posDGainPitch_ = 0;
  printf("%s: posDGain_ is %.3f\n", pitch_ns.c_str(), posDGainPitch_);

  if (!pitch_node.getParam ("velPGain", velPGainPitch_))
    velPGainPitch_ = 0;
  printf("%s: velPGain_ is %.3f\n", pitch_ns.c_str(), velPGainPitch_);

  if (!pitch_node.getParam ("velIGain", velIGainPitch_))
    velIGainPitch_ = 0;
  printf("%s: velIGain_ is %.3f\n", pitch_ns.c_str(), velIGainPitch_);


  if (!pitch_node.getParam ("offset", offsetPitch_))
    offsetPitch_ = 0;
  printf("%s: offset_ is %d\n", pitch_ns.c_str(), offsetPitch_);

  if (!pitch_node.getParam ("posLimit", posLimitPitch_))
    posLimitPitch_ = 0;
  printf("%s: posLimit_ is %d\n", pitch_ns.c_str(), posLimitPitch_);

  if (!pitch_node.getParam ("posPLimit", posPLimitPitch_))
    posPLimitPitch_ = 0;
  printf("%s: posPLimit_ is %d\n", pitch_ns.c_str(), posPLimitPitch_);

  if (!pitch_node.getParam ("posILimit", posILimitPitch_))
    posILimitPitch_ = 0;
  printf("%s: posILimit_ is %d\n", pitch_ns.c_str(), posILimitPitch_);

  if (!pitch_node.getParam ("posDLimit", posDLimitPitch_))
    posDLimitPitch_ = 0;
  printf("%s: posDLimit_ is %d\n", pitch_ns.c_str(), posDLimitPitch_);

  if (!pitch_node.getParam ("velValueLimit", velValueLimitPitch_))
    velValueLimitPitch_ = 0;
  printf("%s: velValueLimit_ is %.3f\n", pitch_ns.c_str(), velValueLimitPitch_);

  if (!pitch_node.getParam ("iEnableLimit", iEnableLimitPitch_))
    iEnableLimitPitch_ = 0;
  printf("%s: iEnableLimit_ is %.3f\n", pitch_ns.c_str(), iEnableLimitPitch_);

  //***** roll
  if (!roll_node.getParam ("ctrlLoopRate", rollCtrlLoopRate_))
    rollCtrlLoopRate_ = 0;
  printf("%s: ctrlLoopRate_ is %d\n", roll_ns.c_str(), rollCtrlLoopRate_);

  if (!roll_node.getParam ("posPGain", posPGainRoll_))
    posPGainRoll_ = 0;
  printf("%s: posPGain_ is %.3f\n", roll_ns.c_str(), posPGainRoll_);

  if (!roll_node.getParam ("posIGain", posIGainRoll_))
    posIGainRoll_ = 0;
  printf("%s: posIGain_ is %.3f\n", roll_ns.c_str(), posIGainRoll_);

  if (!roll_node.getParam ("posIGainHover", posIGainRollHover_))
    posIGainRollHover_ = 0;
  printf("%s: posIGainHover_ is %.3f\n", roll_ns.c_str(), posIGainRollHover_);

  if (!roll_node.getParam ("posDGain", posDGainRoll_))
    posDGainRoll_ = 0;
  printf("%s: posDGain_ is %.3f\n", roll_ns.c_str(), posDGainRoll_);

  if (!roll_node.getParam ("velPGain", velPGainRoll_))
    velPGainRoll_ = 0;
  printf("%s: velPGain_ is %.3f\n", roll_ns.c_str(), velPGainRoll_);

  if (!roll_node.getParam ("velIGain", velIGainRoll_))
    velIGainRoll_ = 0;
  printf("%s: velIGain_ is %.3f\n", roll_ns.c_str(), velIGainRoll_);


  if (!roll_node.getParam ("offset", offsetRoll_))
    offsetRoll_ = 0;
  printf("%s: offset_ is %d\n", roll_ns.c_str(), offsetRoll_);

  if (!roll_node.getParam ("posLimit", posLimitRoll_))
    posLimitRoll_ = 0;
  printf("%s: posLimit_ is %d\n", roll_ns.c_str(), posLimitRoll_);

  if (!roll_node.getParam ("posPLimit", posPLimitRoll_))
    posPLimitRoll_ = 0;
  printf("%s: posPLimit_ is %d\n", roll_ns.c_str(), posPLimitRoll_);

  if (!roll_node.getParam ("posILimit", posILimitRoll_))
    posILimitRoll_ = 0;
  printf("%s: posILimit_ is %d\n", roll_ns.c_str(), posILimitRoll_);

  if (!roll_node.getParam ("posDLimit", posDLimitRoll_))
    posDLimitRoll_ = 0;
  printf("%s: posDLimit_ is %d\n", roll_ns.c_str(), posDLimitRoll_);

  if (!roll_node.getParam ("velValueLimit", velValueLimitRoll_))
    velValueLimitRoll_ = 0;
  printf("%s: velValueLimit_ is %.3f\n", roll_ns.c_str(), velValueLimitRoll_);

  if (!roll_node.getParam ("iEnableLimit", iEnableLimitRoll_))
    iEnableLimitRoll_ = 0;
  printf("%s: iEnableLimit_ is %.3f\n", roll_ns.c_str(), iEnableLimitRoll_);

  //**** yaw
  if (!yaw_node.getParam ("posPGain", posPGainYaw_))
    posPGainYaw_ = 0;
  printf("%s: posPGain_ is %.3f\n", yaw_ns.c_str(), posPGainYaw_);

  if (!yaw_node.getParam ("posIGain", posIGainYaw_))
    posIGainYaw_ = 0;
  printf("%s: posIGain_ is %.3f\n", yaw_ns.c_str(), posIGainYaw_);

  if (!yaw_node.getParam ("posDGain", posDGainYaw_))
    posDGainYaw_ = 0;
  printf("%s: posDGain_ is %.3f\n", yaw_ns.c_str(), posDGainYaw_);

  if (!yaw_node.getParam ("posLimit", posLimitYaw_))
    posLimitYaw_ = 0;
  printf("%s: posLimit_ is %d\n", yaw_ns.c_str(), posLimitYaw_);

  if (!yaw_node.getParam ("posPLimit", posPLimitYaw_))
    posPLimitYaw_ = 0;
  printf("%s: posPLimit_ is %d\n", yaw_ns.c_str(), posPLimitYaw_);

  if (!yaw_node.getParam ("posILimit", posILimitYaw_))
    posILimitYaw_ = 0;
  printf("%s: posILimit_ is %d\n", yaw_ns.c_str(), posILimitYaw_);

  if (!yaw_node.getParam ("posDLimit", posDLimitYaw_))
    posDLimitYaw_ = 0;
  printf("%s: posDLimit_ is %d\n", yaw_ns.c_str(), posDLimitYaw_);

  if (!yaw_node.getParam ("velValueLimit", velValueLimitYaw_))
    velValueLimitYaw_ = 0;
  printf("%s: velValueLimit_ is %.3f\n", yaw_ns.c_str(), velValueLimitYaw_);

  if (!yaw_node.getParam ("iEnableLimit", iEnableLimitYaw_))
    iEnableLimitYaw_ = 0;
  printf("%s: iEnableLimit_ is %.3f\n", yaw_ns.c_str(), iEnableLimitYaw_);

  if (!yaw_node.getParam ("ctrlLoopRate", yawCtrlLoopRate_))
    yawCtrlLoopRate_ = 0;
  printf("%s: ctrlLoopRate_ is %d\n", yaw_ns.c_str(), yawCtrlLoopRate_);

}

void PidController::setControlBoardToAsctec()
{
  controlBoard_ = ASCTEC_BOARD;
  printf("change control board from kduino to asctec\n");
}

uint8_t PidController::getControlBoard()
{
  return controlBoard_;
}

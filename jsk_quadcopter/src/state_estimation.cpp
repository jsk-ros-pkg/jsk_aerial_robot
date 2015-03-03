
#include "jsk_quadcopter/state_estimation.h"

Estimator::Estimator()
{
  outerEstimatePosX = 0;
  outerEstimateVelX = 0;
  outerEstimatePosY = 0;
  outerEstimateVelY = 0;
  outerEstimatePosZ = 0;
  outerEstimateVelZ = 0;
  outerEstimateTheta = 0;
  outerEstimateVelTheta = 0;
  outerEstimatePhy = 0;
  outerEstimateVelPhy = 0;
  outerEstimatePsiCog = 0;
  outerEstimateVelPsiCog = 0;
  outerEstimatePsiBody = 0;
  outerEstimateVelPsiBody = 0;


  statePosZOffset = 1.0; //1m

  useOuterPoseEstimate = 0;
  useOuterVelEstimate = 0;
}

Estimator::~Estimator()
{

}

//deprecated: begin
float Estimator::getStatePosX()
{
  return 0;
}
float Estimator::getStatePosXc()
{
  return 0;
}
float Estimator::getStateVelX()
{
  return 0;
}
float Estimator::getStateVelXc()
{
  return 0;
}
float Estimator::getStateAccXb()
{
  return 0;
}
float Estimator::getStatePosY()
{
  return 0;
}
float Estimator::getStatePosYc()
{
  return 0;
}
float Estimator::getStateVelY()
{
  return 0;
}
float Estimator::getStateVelYc()
{
  return 0;
}
float Estimator::getStateAccYb()
{
  return 0;
}
float Estimator::getStatePosZ()
{
  return 0;
}
float Estimator::getStateVelZ()
{
  return 0;
}
float Estimator::getStateAccZb()
{
  return 0;
}
float Estimator::getStateTheta()
{
  return 0;
}
float Estimator::getStatePhy()
{
  return 0;
}
float Estimator::getStatePsiBody()
{
  return 0;
}
float Estimator::getStateVelPsiBody()
{
  return 0;
}

float Estimator::getStatePsiCog()
{
  return 0;
}
float Estimator::getStateVelPsiCog()
{
  return 0;
}


float Estimator::getStateVelXOpt()
{
  return 0;
}
float Estimator::getStateVelYOpt()
{
  return 0;
}


void Estimator::setOuterEstimatePoseFlag(uint8_t axis)
{
  if(axis == 0)
    useOuterPoseEstimate = 0;
  else
    useOuterPoseEstimate |= axis;
}

void Estimator::setOuterEstimateVelFlag(uint8_t axis)
{
  if(axis == 0)
    useOuterVelEstimate = 0;
  else
    useOuterVelEstimate |= axis;
}

void Estimator::setKFMeaureFlag(int axis, bool flag)
{
}

//deprecated: end

float Estimator::getPosZOffset()
{
  return  statePosZOffset;
}

void Estimator::setPosZOffset(float pos_z_offset)
{
  statePosZOffset = pos_z_offset;
}

float Estimator::getLaserToImuDistance()
{
  return 0;
}

bool Estimator::getRocketStartFlag()
{
  return true;
}



void Estimator::setRocketStartFlag()
{
  //do nothing;
}


void Estimator::setStatePosX(float value)
{
  if(useOuterPoseEstimate & X_AXIS)
    outerEstimatePosX = value;
}
void Estimator::setStateVelX(float value)
{
  if(useOuterVelEstimate & X_AXIS)
    outerEstimateVelX = value;
}
void Estimator::setStatePosY(float value)
{
  if(useOuterPoseEstimate & Y_AXIS)
    outerEstimatePosY = value;
}
void Estimator::setStateVelY(float value)
{
  if(useOuterVelEstimate & Y_AXIS)
    outerEstimateVelY = value;
}
void Estimator::setStatePosZ(float value)
{
  if(useOuterPoseEstimate & Z_AXIS)
    outerEstimatePosZ = value;
}
void Estimator::setStateVelZ(float value)
{
  if(useOuterVelEstimate & Z_AXIS)
    outerEstimateVelZ = value;
}
void Estimator::setStateTheta(float value)
{
  if(useOuterPoseEstimate & PITCH_AXIS)
    outerEstimateTheta = value;
}
void Estimator::setStateVelTheta(float value)
{
  if(useOuterVelEstimate & PITCH_AXIS)
    outerEstimateVelTheta = value;
}
void Estimator::setStatePhy(float value)
{
    if(useOuterPoseEstimate & ROLL_AXIS)
      outerEstimatePhy = value;
}
void Estimator::setStateVelPhy(float value)
{
  if(useOuterVelEstimate & ROLL_AXIS)
    outerEstimateVelPhy = value;
}
void Estimator::setStatePsiCog(float value)
{
  if(useOuterPoseEstimate & YAW_AXIS)
    outerEstimatePsiCog = value;
}
void Estimator::setStateVelPsiCog(float value)
{
  if(useOuterVelEstimate & YAW_AXIS)
    outerEstimateVelPsiCog = value;
}
void Estimator::setStatePsiBody(float value)
{
  if(useOuterPoseEstimate & YAW_AXIS)
    outerEstimatePsiBody = value;
}
void Estimator::setStateVelPsiBody(float value)
{
  if(useOuterVelEstimate & YAW_AXIS)
    outerEstimateVelPsiBody = value;
}
void Estimator::setStatePsi(float value)
{
  if(useOuterPoseEstimate & YAW_AXIS)
    {
      outerEstimatePsiCog = value;
      outerEstimatePsiBody = value;
    }
}
void Estimator::setStateVelPsi(float value)
{
  if(useOuterVelEstimate & YAW_AXIS)
    {
      outerEstimateVelPsiCog = value;
      outerEstimateVelPsiBody = value;
    }
}



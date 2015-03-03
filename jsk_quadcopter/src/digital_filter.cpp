/*
+*+*+*+* 最重要事項：関数のstatic変数はすべて同じポインタに保存されるので、同じ関数を複数呼ぶ場合、中身のstatic変数はお互い

*/

#include "jsk_quadcopter/digital_filter.h"

Filter::Filter()
{
}

Filter::~Filter()
{
}

void Filter::filterFunction()
{
}




FirFilter::FirFilter()
{
}

FirFilter::FirFilter(float filter_factor)
{
  filterFactor = filter_factor;
  intermediateResult = 0;
}

FirFilter::~FirFilter()
{
}

float FirFilter::getFilteredResult()
{
  return  intermediateResult / filterFactor;
}

void FirFilter::filterFunction(float input)
{
  intermediateResult -= intermediateResult / filterFactor;
  intermediateResult += input;
}

IirFilter::IirFilter()
{
}

IirFilter::IirFilter(float rx_freq,
		     float cutoff_pos_freq, float cutoff_vel_freq,
		     float vel_val_thre, float vel_change_rate_thre): Filter()
{
  setRxFreq(rx_freq);
  setCutoffVelFreq(cutoff_vel_freq);
  setCutoffPosFreq(cutoff_pos_freq);
  setVelValThre( vel_val_thre);
  setVelChangeRateThre( vel_change_rate_thre);
  setInitParam();
}

IirFilter::IirFilter(float rx_freq,
		     float cutoff_pos_freq): Filter()
{
  setRxFreq(rx_freq);
  setCutoffVelFreq(0);
  setCutoffPosFreq(cutoff_pos_freq);
  setVelValThre(0);
  setVelChangeRateThre(0);
  setInitParam();
}

IirFilter::IirFilter(float rx_freq,
		     float cutoff_pos_freq,
                     float cutoff_vel_freq): Filter()
{
  setRxFreq(rx_freq);
  setCutoffVelFreq(cutoff_vel_freq);
  setCutoffPosFreq(cutoff_pos_freq);
  setVelValThre(0);
  setVelChangeRateThre(0);
  setInitParam();
}


IirFilter::~IirFilter()
{
}

void IirFilter:: setRxFreq(float rx_freq)
{
  rxFreq = rx_freq;
}
void IirFilter:: setCutoffVelFreq(float cutoff_vel_freq)
{
  cutoffVelFreq = cutoff_vel_freq;
}
void IirFilter:: setCutoffPosFreq(float cutoff_pos_freq)
{
  cutoffPosFreq = cutoff_pos_freq;
}
void IirFilter:: setVelValThre(float vel_val_thre)
{
  velValThre = vel_val_thre;
}
void IirFilter:: setVelChangeRateThre(float vel_change_rate_thre)
{
  velChangeRateThre = vel_change_rate_thre;
}
void IirFilter:: setAxis(const int& axis)
{
  fAxis = axis;
}

void IirFilter::setInitParam()
{
  //** pos
  w0Pos = tan(M_PI * cutoffPosFreq / rxFreq);
  aPos  = sin(w0Pos) / 0.707; 
  a1Pos = 2 * cos(w0Pos) / (1 + aPos);
  a2Pos = (aPos - 1) / (aPos + 1);
  b0Pos = (1 - cos(w0Pos)) / 2 / (1 + aPos);
  b1Pos =  (1 - cos(w0Pos)) / (1 + aPos);
  b2Pos = (1 - cos(w0Pos)) / 2 / (1 + aPos);

  //  ROS_WARN(" w0:%f , a:%f, a1:%f, a2:%f, b0:%f, b1:%f, b2:%f", w0Pos,aPos,a1Pos,a2Pos,b0Pos,b1Pos,b2Pos);

  //** vel
  if(cutoffVelFreq == 0)
    {
      w0Vel = 0;
      aVel  = 0;
      a1Vel = 0;
      a2Vel = 0;
      b0Vel = 0;
      b1Vel = 0;
      b2Vel = 0;
    }
  else
    {
      w0Vel = tan(M_PI * cutoffVelFreq / rxFreq);
      aVel  = sin(w0Vel) / 0.707; 
      a1Vel = 2 * cos(w0Vel) / (1 + aVel);
      a2Vel = (aVel - 1) / (aVel + 1);
      b0Vel = (1 - cos(w0Vel)) / 2 / (1 + aVel);
      b1Vel =  (1 - cos(w0Vel)) / (1 + aVel);
      b2Vel = (1 - cos(w0Vel)) / 2 / (1 + aVel);
    }

  pre1_pos=0; 
  pre2_pos=0; 
  pre1_vel=0;
  pre2_vel=0;
  prev_raw_vel_val =0;
  first_flag = true;
}

#if 0 //deprecated
void IirFilter::filterFunction(double pos_input, double& pos_output,
			       double vel_input, double& vel_output)
{
  
  //+*+*+*+* 最重要事項：関数のstatic変数はすべて同じポインタに保存されるので、同じ関数を複数呼ぶ場合、中身のstatic変数はお互い
  
  //static double pre1_pos=0, pre2_pos=0, pre1_vel=0, pre2_vel=0, prev_raw_vel_val =0;
  double reg_pos=0 ,reg_vel=0;
  //static bool first_flag = true;
  
  float raw_pos_val = pos_input;
  float raw_vel_val = vel_input;
     
  //debug**
  // static int i = 0;
  // ROS_WARN(" i  = %d ", i++);

  if( first_flag ) first_flag = false;
  else
    {
      //*** pos
      reg_pos = raw_pos_val + a1Pos * pre1_pos + a2Pos * pre2_pos;
      pos_output = b0Pos * reg_pos + b1Pos *pre1_pos + b2Pos * pre2_pos;
      pre2_pos = pre1_pos;
      pre1_pos = reg_pos;

      //debug
      // if(raw_pos_val > 0.3)
      // 	ROS_WARN("raw pos is %f, pos output is %f, pre1_pos is %f, pre2_pos is %f", raw_pos_val, pos_output, pre1_pos, pre2_pos);


      //*** vel
      if(abs(raw_vel_val) > velValThre &&
      	 abs(raw_vel_val) > velChangeRateThre * abs(prev_raw_vel_val) &&
      	 prev_raw_vel_val != 0)
      	{
      	  raw_vel_val = prev_raw_vel_val;
      	}
      reg_vel = raw_vel_val + a1Vel * pre1_vel + a2Vel * pre2_vel;
      vel_output = b0Vel * reg_vel + b1Vel *pre1_vel + b2Vel * pre2_vel;
      pre2_vel = pre1_vel;
      pre1_vel = reg_vel;
    }
  //更新
  prev_raw_vel_val = raw_vel_val;
}
#endif

void IirFilter::filterFunction(double pos_input, double& pos_output,
			       double vel_input, double& vel_output)
{
  double reg_pos=0 ,reg_vel=0;
  float raw_pos_val = pos_input;
  float raw_vel_val = vel_input;

  if( first_flag ) first_flag = false;
  else
    {
      //*** pos
      reg_pos = raw_pos_val + a1Pos * pre1_pos + a2Pos * pre2_pos;
      pos_output = b0Pos * reg_pos + b1Pos *pre1_pos + b2Pos * pre2_pos;
      pre2_pos = pre1_pos;
      pre1_pos = reg_pos;

      //*** vel
      reg_vel = raw_vel_val + a1Vel * pre1_vel + a2Vel * pre2_vel;
      vel_output = b0Vel * reg_vel + b1Vel *pre1_vel + b2Vel * pre2_vel;
      pre2_vel = pre1_vel;
      pre1_vel = reg_vel;
    }
}

void IirFilter::filterFunction(double pos_input, double& pos_output)
{
  double reg_pos=0;
  float raw_pos_val = pos_input;
     
  //debug**
  // static int i = 0;
  // ROS_WARN(" i  = %d ", i++);


  if( first_flag ) first_flag = false;
  else
    {
      reg_pos = raw_pos_val + a1Pos * pre1_pos + a2Pos * pre2_pos;
      pos_output = b0Pos * reg_pos + b1Pos *pre1_pos + b2Pos * pre2_pos;
      pre2_pos = pre1_pos;
      pre1_pos = reg_pos;
    }
}
 

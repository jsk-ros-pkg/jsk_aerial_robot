#ifndef DIGITAL_FILTER_H
#define DIGITAL_FILTER_H

//* ros
#include <ros/ros.h>

#include <iostream>



class Filter
{
 public:
  Filter();
  virtual ~Filter();

  virtual void filterFunction();

  const static int X = 0;
  const static int Y = 1;
  const static int Z = 2;
  const static int THETA = 3;
  const static int PHY = 4;
  const static int PSI = 5;
  const static int DEBUG1 = 6;
  const static int DEBUG2 = 7;
};


class IirFilter : public Filter
{
 private:
  float rxFreq;

  float cutoffPosFreq;
  double aPos;
  double w0Pos;
  double a1Pos;
  double a2Pos;
  double b0Pos;
  double b1Pos;
  double b2Pos;

  float cutoffVelFreq;
  double aVel;
  double w0Vel;
  double a1Vel;
  double a2Vel;
  double b0Vel;
  double b1Vel;
  double b2Vel;

  double pre1_pos;
  double pre2_pos;
  double pre1_vel;
  double pre2_vel;
  double prev_raw_vel_val;
  bool first_flag;

  int fAxis; //filtering axis
  float velValThre;
  float velChangeRateThre;


 public:
  IirFilter(); 
  
  //deprecated
  IirFilter(const int& axis, float rx_freq, 
	    float cutoff_pos_freq, float cutoff_vel_freq,
	    float vel_val_thre, float vel_change_rate_thre);
  IirFilter(float rx_freq, 
	    float cutoff_pos_freq, float cutoff_vel_freq,
	    float vel_val_thre, float vel_change_rate_thre);
  IirFilter(float rx_freq, float cutoff_pos_freq);
  IirFilter(float rx_freq, float cutoff_pos_freq, float cutoff_vel_freq);

  ~IirFilter();

  void setRxFreq(float rx_freq);
  void setCutoffVelFreq(float cutoff_vel_freq);
  void setCutoffPosFreq(float cutoff_pos_freq);
  void setVelValThre(float vel_val_thre);
  void setVelChangeRateThre(float vel_change_rate_thre);
  void setInitParam();
  void setAxis(const int& axis);
  void filterFunction(double pos_input, double& pos_output,
		      double vel_input, double& vel_output);
  void filterFunction(double pos_input, double& pos_output);

};

class FirFilter : public Filter
{
 private:
  float filterFactor;
  float intermediateResult;



 public:
  FirFilter(); 
  FirFilter(float filter_factor);
  ~FirFilter();
  float getFilteredResult();
  void filterFunction(float input);

};

#endif

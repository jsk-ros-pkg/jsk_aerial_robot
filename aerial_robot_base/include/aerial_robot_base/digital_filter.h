#ifndef DIGITAL_FILTER_H
#define DIGITAL_FILTER_H

//* ros
#include <ros/ros.h>
#include <iostream>



class Filter
{
 public:
  Filter(){}
  virtual ~Filter(){}

  virtual void filterFunction(){}

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
  float rx_freq_;

  float cutoff_pos_freq_;
  double a_pos_;
  double w0_pos_;
  double a1_pos_;
  double a2_pos_;
  double b0_pos_;
  double b1_pos_;
  double b2_pos_;

  float cutoff_vel_freq_;
  double a_vel_;
  double w0_vel_;
  double a1_vel_;
  double a2_vel_;
  double b0_vel_;
  double b1_vel_;
  double b2_vel_;

  double pre1_pos_;
  double pre2_pos_;
  double pre1_vel_;
  double pre2_vel_;
  double prev_raw_vel_val_;
  bool first_flag_;

  float vel_val_thre_;
  float vel_change_rate_thre_;


 public:
  IirFilter(){} 
  
  IirFilter(float rx_freq, 
	    float cutoff_pos_freq, float cutoff_vel_freq,
	    float vel_val_thre, float vel_change_rate_thre): Filter()
    {
      setRxFreq(rx_freq);
      setCutoffVelFreq(cutoff_vel_freq);
      setCutoffPosFreq(cutoff_pos_freq);
      setVelValThre( vel_val_thre);
      setVelChangeRateThre(vel_change_rate_thre);
      setInitParam();
    }

 IirFilter(float rx_freq, float cutoff_pos_freq): Filter()
    {
      setRxFreq(rx_freq);
      setCutoffVelFreq(0);
      setCutoffPosFreq(cutoff_pos_freq);
      setVelValThre(0);
      setVelChangeRateThre(0);
      setInitParam();
    }

 IirFilter(float rx_freq, float cutoff_pos_freq, float cutoff_vel_freq): Filter()
    {
      setRxFreq(rx_freq);
      setCutoffVelFreq(cutoff_vel_freq);
      setCutoffPosFreq(cutoff_pos_freq);
      setVelValThre(0);
      setVelChangeRateThre(0);
      setInitParam();
    }

  ~IirFilter(){}

  inline void setRxFreq(float rx_freq){  rx_freq_ = rx_freq;}
  inline void setCutoffVelFreq(float cutoff_vel_freq){  cutoff_vel_freq_ = cutoff_vel_freq;}
  inline void setCutoffPosFreq(float cutoff_pos_freq){  cutoff_pos_freq_ = cutoff_pos_freq;}
  inline void setVelValThre(float vel_val_thre){ vel_val_thre_ = vel_val_thre;}
  inline void setVelChangeRateThre(float vel_change_rate_thre)  { vel_change_rate_thre_ = vel_change_rate_thre;}

  void setInitParam()
  {
    //** pos
    w0_pos_ = tan(M_PI * cutoff_pos_freq_ / rx_freq_);
    a_pos_  = sin(w0_pos_) / 0.707; 
    a1_pos_ = 2 * cos(w0_pos_) / (1 + a_pos_ );
    a2_pos_ = (a_pos_ - 1) / (a_pos_ + 1);
    b0_pos_ = (1 - cos(w0_pos_)) / 2 / (1 + a_pos_);
    b1_pos_ =  (1 - cos(w0_pos_)) / (1 + a_pos_);
    b2_pos_ = (1 - cos(w0_pos_)) / 2 / (1 + a_pos_);

    //** vel_
    if(cutoff_vel_freq_ == 0)
      {
        w0_vel_ = 0;
        a_vel_  = 0;
        a1_vel_ = 0;
        a2_vel_ = 0;
        b0_vel_ = 0;
        b1_vel_ = 0;
        b2_vel_ = 0;
      }
    else
      {
        w0_vel_ = tan(M_PI * cutoff_vel_freq_ / rx_freq_);
        a_vel_  = sin(w0_vel_) / 0.707; 
        a1_vel_ = 2 * cos(w0_vel_) / (1 + a_vel_);
        a2_vel_ = (a_vel_ - 1) / (a_vel_ + 1);
        b0_vel_ = (1 - cos(w0_vel_)) / 2 / (1 + a_vel_);
        b1_vel_ =  (1 - cos(w0_vel_)) / (1 + a_vel_);
        b2_vel_ = (1 - cos(w0_vel_)) / 2 / (1 + a_vel_);
      }

    pre1_pos_ =0; 
    pre2_pos_ =0; 
    pre1_vel_ =0;
    pre2_vel_ =0;
    prev_raw_vel_val_ =0;
    //first_flag_ = true;
  }

  void filterFunction(double pos_input, double& pos_output,
		      double vel_input, double& vel_output)
  {
    double reg_pos=0 ,reg_vel=0;
    float raw_pos_val = pos_input;
    float raw_vel_val = vel_input;

#if 0
    if( first_flag ) first_flag = false;
    else
      {
        //*** pos
        reg_pos = raw_pos_val + a1_pos_ * pre1_pos_ + a2_pos_ * pre2_pos_;
        pos_output = b0_pos_ * reg_pos + b1_pos_ * pre1_pos_ + b2_pos_ * pre2_pos_;
        pre2_pos_ = pre1_pos_;
        pre1_pos_ = reg_pos;

        //*** vel
        reg_vel = raw_vel_val + a1_vel_ * pre1_vel_ + a2_vel_ * pre2_vel_;
        vel_output = b0_vel_ * reg_vel + b1_vel_ * pre1_vel_ + b2_vel_ * pre2_vel_;
        pre2_vel_ = pre1_vel_;
        pre1_vel_ = reg_vel;
      }
#else
        //*** pos
        reg_pos = raw_pos_val + a1_pos_ * pre1_pos_ + a2_pos_ * pre2_pos_;
        pos_output = b0_pos_ * reg_pos + b1_pos_ * pre1_pos_ + b2_pos_ * pre2_pos_;
        pre2_pos_ = pre1_pos_;
        pre1_pos_ = reg_pos;

        //*** vel
        reg_vel = raw_vel_val + a1_vel_ * pre1_vel_ + a2_vel_ * pre2_vel_;
        vel_output = b0_vel_ * reg_vel + b1_vel_ * pre1_vel_ + b2_vel_ * pre2_vel_;
        pre2_vel_ = pre1_vel_;
        pre1_vel_ = reg_vel;
#endif

  }
  void filterFunction(double pos_input, double& pos_output)
  {
    double reg_pos=0;
    float raw_pos_val = pos_input;

#if 0
    if( first_flag ) first_flag = false;
    else
      {
        reg_pos = raw_pos_val + a1_pos_ * pre1_pos_ + a2_pos_ * pre2_pos_;
        pos_output = b0_pos_ * reg_pos + b1_pos_ * pre1_pos_ + b2_pos_ * pre2_pos_;
        pre2_pos_ = pre1_pos_;
        pre1_pos_ = reg_pos;
      }
#else
    reg_pos = raw_pos_val + a1_pos_ * pre1_pos_ + a2_pos_ * pre2_pos_;
    pos_output = b0_pos_ * reg_pos + b1_pos_ * pre1_pos_ + b2_pos_ * pre2_pos_;
    pre2_pos_ = pre1_pos_;
    pre1_pos_ = reg_pos;

#endif
  }

};

class FirFilter : public Filter
{
 private:
  float filter_factor_;
  float intermediate_result_;


 public:
  FirFilter(){}
  FirFilter(float filter_factor)
    {
      filter_factor_ = filter_factor;
      intermediate_result_ = 0;
    }

  ~FirFilter(){}
  inline float getFilteredResult(){  return  intermediate_result_ / filter_factor_;}

  void filterFunction(float input)
  {
    intermediate_result_ -= intermediate_result_ / filter_factor_;
    intermediate_result_ += input;
  }

};

#endif

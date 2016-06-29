#ifndef CONTROL_INPUT_ARRAY_H
#define CONTROL_INPUT_ARRAY_H

//* ros
#include <ros/ros.h>
#include <vector>

class FlightCtrlInput
{
 public:
  FlightCtrlInput (int motor_num)
{
  motor_num_ = motor_num;

  yaw_.resize(motor_num_);
  throttle_.resize(motor_num_);

  for(int i = 0; i < motor_num_; i++)
    {
      yaw_[i] = 0;
      throttle_[i] = 0;
    }

  pitch_ = 0;
  roll_ = 0;

}

  ~FlightCtrlInput (){ }

  inline  void setPitchValue(float pitch_value){  pitch_ = pitch_value;}
  inline  void setRollValue(float roll_value){  roll_ = roll_value; }
  inline  void setYawValues(std::vector<float> yaw_value){  yaw_ = yaw_value; }
  inline  void setThrottleValues(std::vector<uint16_t> throttle_value){  throttle_ = throttle_value;}
  inline  void setYawValue(float yaw_value, int motor_no){  yaw_[motor_no] = yaw_value; }
  inline  void setThrottleValue(uint16_t throttle_value, int motor_no){  throttle_[motor_no] = throttle_value;}


  inline  float getPitchValue(){ return pitch_;}
  inline  float getRollValue(){ return roll_;}
  inline  std::vector<float> getYawValue(){ return yaw_;}
  inline  std::vector<uint16_t> getThrottleValue(){ return throttle_;}

  inline  int getMotorNumber(){return motor_num_;}

  void reset()
  {
    pitch_ = 0;
    roll_ = 0;
    std::vector<float> reset_data1(motor_num_, 0); 
    std::vector<uint16_t> reset_data2(motor_num_, 0);  
    yaw_ = reset_data1;
    throttle_ = reset_data2;
  }

  void addFFValues(std::vector<int16_t> ff_value)
  {  
    if(ff_value.size() != motor_num_) 
      {
        ROS_FATAL("bad size matching for ff");
        return;
      }

    for(int i = 0; i < motor_num_; i++)
      {
        int16_t tmp =  (int16_t)throttle_[i]  + ff_value[i];
        //hard code
        if(tmp < 1200 && tmp > 1800) 
          {
            ROS_ERROR("bad range matching for ff");
            return;
          }
      }
    for(int i = 0; i < motor_num_; i++)
        throttle_[i]  += ff_value[i];
  }

 private:
  int motor_num_;

  float pitch_; //[0.1degree]
  float roll_;  //[0.1degree]
  std::vector<float> yaw_; //14bit shift, f[N]
  std::vector<uint16_t> throttle_; //PWM (1000 ~ 2000, exaclty: 1300 ~ 1750)

};

#endif

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
  inline  void setThrottleValues(std::vector<float> throttle_value){  throttle_ = throttle_value;}
  inline  void setYawValue(float yaw_value, int motor_no){  yaw_[motor_no] = yaw_value; }
  inline  void setThrottleValue(float throttle_value, int motor_no){  throttle_[motor_no] = throttle_value;}

  inline  float getPitchValue(){ return pitch_;}
  inline  float getRollValue(){ return roll_;}
  inline  std::vector<float> getYawValue(){ return yaw_;}
  inline  std::vector<float> getThrottleValue(){ return throttle_;}

  inline  int getMotorNumber(){return motor_num_;}

  void reset()
  {
    pitch_ = 0;
    roll_ = 0;
    std::vector<float> reset_data(motor_num_, 0);
    yaw_ = reset_data;
    throttle_ = reset_data;
  }

  void addFFValues(std::vector<float> ff_value, bool force_add_flag = false)
  {
    if(ff_value.size() != motor_num_)
      {
        ROS_FATAL("bad size matching for ff");
        return;
      }

    /* F[N] based */
    /* chekck range */
    if(!force_add_flag)
      {
        for(int i = 0; i < motor_num_; i++)
          {
            float tmp =  throttle_[i]  + ff_value[i];

            if(tmp < 0)
              {
                ROS_ERROR("bad range matching for ff");
                return;
              }
          }
      }

    for(int i = 0; i < motor_num_; i++)
      throttle_[i]  += ff_value[i];
  }

 private:
  int motor_num_;

  float pitch_; //[rad]
  float roll_;  //[rad]
  std::vector<float> yaw_; //[N], Nm -> N conversio rate is contained in gain map
  std::vector<float> throttle_; //[N]

};

#endif

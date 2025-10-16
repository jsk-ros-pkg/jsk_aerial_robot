#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "stdbool.h"
#include "main.h"
#ifndef SIMULATION
#include "config.h"
#endif

class Buzzer
{
public:
  Buzzer();
  ~Buzzer(){}
  void init(TIM_HandleTypeDef* htim, int ch);
  void on(float hz);
  void mute();
  void alert();
  void init_sound();
private:
  TIM_HandleTypeDef* pwm_htim_;
  int pwm_ch_;
};


#endif /* INC_BUZZER_H_ */

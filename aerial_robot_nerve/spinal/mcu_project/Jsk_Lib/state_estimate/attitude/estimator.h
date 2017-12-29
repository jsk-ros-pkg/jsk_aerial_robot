/*
******************************************************************************
* File Name          : estimator.h
* Description        : super  class for attitude estiamte algorithm
******************************************************************************
*/
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __ATTITUDE_ESTIMATOR_H
#define __ATTITUDE_ESTIMATOR_H

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <math/AP_Math.h>
#include <array>

#define DELTA_T 0.001f
#define MAG_DECLINIATION -0.13002702927f //[rad] = -7.27[deg], http://www.magnetic-declination.com/
#define GYRO_AMP 1.1395f  //if value= 1, it means normal complementrary filter

namespace Axis
{
enum {ROLL = 0, PITCH = 1, YAW = 2, };
}
namespace Frame
{
enum { BODY = 0, VIRTUAL = 1,};
}

class EstimatorAlgorithm
{

public:

	EstimatorAlgorithm():
    acc_(), gyro_(), mag_(), prev_mag_(), pre1_(), pre2_()
  {
    r_.identity();
    gyro_amp_ = 1.0f;


    /* IIR LPF */
    rx_freq_ = 1000.0f;
    cutoff_freq_ = 10.0f;
    w0_ = tan(M_PI * cutoff_freq_ / rx_freq_);
    a_  = sin(w0_) / 0.707;
    a1_ = 2 * cos(w0_) / (1 + a_ );
    a2_ = (a_ - 1) / (a_ + 1);
    b0_ = (1 - cos(w0_)) / 2 / (1 + a_);
    b1_ =  (1 - cos(w0_)) / (1 + a_);
    b2_ = (1 - cos(w0_)) / 2 / (1 + a_);

  };

  ~EstimatorAlgorithm(){}

  /* coodrinate change  */
  void coordinateUpdate(float desire_attitude_roll, float desire_attitude_pitch, float desire_attitude_yaw)
  {
    r_.from_euler(desire_attitude_roll, desire_attitude_pitch, desire_attitude_yaw);
  }


  void update(const Vector3f& gyro, const Vector3f& acc, const Vector3f& mag)
  {
    /* the sensor data in body frame */
    acc_[Frame::BODY] = acc;
    gyro_[Frame::BODY] = gyro;
    mag_[Frame::BODY] = mag;

    /* the sensor data in virtual frame */
    acc_[Frame::VIRTUAL] = r_* acc_[Frame::BODY];
    gyro_[Frame::VIRTUAL] = r_*  gyro_[Frame::BODY];
    mag_[Frame::VIRTUAL] = r_ * mag_[Frame::BODY];

    /* LPF */
    filterFunction(gyro_[Frame::BODY], gyro_smooth_[Frame::BODY]);
    gyro_smooth_[Frame::VIRTUAL] = r_*  gyro_smooth_[Frame::BODY];

    estimation();
  }

  /* we observe that if there is amplified rate about 1.13, the control will be more stable */
  void gyroIntegralAmp(bool amp_flag)
  {
    //if(amp_flag) gyro_amp_ = GYRO_AMP;
    if(amp_flag) gyro_amp_ = 1.0f;
    else gyro_amp_ = 1.0f;
  }

  virtual void estimation(){}; //please implementation!

  static const uint8_t BODY_FRAME = 0;
  static const uint8_t RELATIVE_COORD = 1;
  static const uint8_t X = 0;
  static const uint8_t Y = 1;
  static const uint8_t Z = 2;

  Vector3f getAttitude(uint8_t frame){return rpy_[frame];}
  Vector3f getAngular(uint8_t frame){return gyro_[frame];}
  Vector3f getSmoothAngular(uint8_t frame){return gyro_smooth_[frame];}
  Vector3f getAcc(uint8_t frame){return acc_[frame];}
  Vector3f getMag(uint8_t frame){return mag_[frame];}

protected:
  std::array<Vector3f, 2> acc_, gyro_, mag_, gyro_smooth_;
  Vector3f prev_mag_; /* for lpf filter method for mag */
  Matrix3f r_;
  std::array<Vector3f, 2> rpy_;

  /* IIR lpf */
  float rx_freq_;
  float cutoff_freq_;
  double a_;
  double w0_;
  double a1_;
  double a2_;
  double b0_;
  double b1_;
  double b2_;
  Vector3f pre1_, pre2_;

  void filterFunction(Vector3f input, Vector3f& output)
  {
    Vector3f reg = input + pre1_  *  a1_ + pre2_ * a2_;
    output = reg  * b0_ +   pre1_ * b1_ + pre2_ * b2_;
    pre2_ = pre1_;
    pre1_ = reg;
  }

  float gyro_amp_; // the gyro integaral attitude estiamtion amplification rate
};

#endif

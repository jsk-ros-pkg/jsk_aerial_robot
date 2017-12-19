/*
******************************************************************************
* File Name          : madgwick_ahrs.h
* Description        : ahrs estimation by madgiwck 
******************************************************************************
*/
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __MADGWICK_AHRS_h
#define __MADGWICK_AHRS_h

#include "state_estimate/attitude/estimator.h"

#define betaDef 0.1f // 2 * proportional gain
#define ACC_GYRO 0
#define ACC_GYRO_MAG 1
#define ESTIMATE_METHOD ACC_GYRO_MAG

class MadgwickAHRS : public EstimatorAlgorithm
{
public:
  MadgwickAHRS():EstimatorAlgorithm()
  {
    beta = betaDef; // 2 * proportional gain (Kp)
  }

  virtual void  estimation()
  {
    if ( prev_mag_b_ == mag_b_ )
      {
        accGyroEstimate(gyro_b_[0], gyro_b_[1], gyro_b_[2], acc_b_[0], acc_b_[1], acc_b_[2]);
      }
    else
      {
        accGyroMagEstimate(gyro_b_[0], gyro_b_[1], gyro_b_[2], acc_b_[0], acc_b_[1], acc_b_[2], mag_b_[0], mag_b_[1], mag_b_[2]);
        prev_mag_b_ = mag_b_;
      }
  }

private:
  float beta;// algorithm gain

  void accGyroMagEstimate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
  {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
      accGyroEstimate(gx, gy, gz, ax, ay, az);
      return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q_[1] * gx - q_[2] * gy - q_[3] * gz);
    qDot2 = 0.5f * (q_[0] * gx + q_[2] * gz - q_[3] * gy);
    qDot3 = 0.5f * (q_[0] * gy - q_[1] * gz + q_[3] * gx);
    qDot4 = 0.5f * (q_[0] * gz + q_[1] * gy - q_[2] * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

      // Normalise accelerometer measurement
      recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Normalise magnetometer measurement
      recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
      mx *= recipNorm;
      my *= recipNorm;
      mz *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      _2q0mx = 2.0f * q_[0] * mx;
      _2q0my = 2.0f * q_[0] * my;
      _2q0mz = 2.0f * q_[0] * mz;
      _2q1mx = 2.0f * q_[1] * mx;
      _2q0 = 2.0f * q_[0];
      _2q1 = 2.0f * q_[1];
      _2q2 = 2.0f * q_[2];
      _2q3 = 2.0f * q_[3];
      _2q0q2 = 2.0f * q_[0] * q_[2];
      _2q2q3 = 2.0f * q_[2] * q_[3];
      q0q0 = q_[0] * q_[0];
      q0q1 = q_[0] * q_[1];
      q0q2 = q_[0] * q_[2];
      q0q3 = q_[0] * q_[3];
      q1q1 = q_[1] * q_[1];
      q1q2 = q_[1] * q_[2];
      q1q3 = q_[1] * q_[3];
      q2q2 = q_[2] * q_[2];
      q2q3 = q_[2] * q_[3];
      q3q3 = q_[3] * q_[3];

      // Reference direction of Earth's magnetic field
      hx = mx * q0q0 - _2q0my * q_[3] + _2q0mz * q_[2] + mx * q1q1 + _2q1 * my * q_[2] + _2q1 * mz * q_[3] - mx * q2q2 - mx * q3q3;
      hy = _2q0mx * q_[3] + my * q0q0 - _2q0mz * q_[1] + _2q1mx * q_[2] - my * q1q1 + my * q2q2 + _2q2 * mz * q_[3] - my * q3q3;
      _2bx = sqrt(hx * hx + hy * hy);
      _2bz = -_2q0mx * q_[2] + _2q0my * q_[1] + mz * q0q0 + _2q1mx * q_[3] - mz * q1q1 + _2q2 * my * q_[3] - mz * q2q2 + mz * q3q3;
      _4bx = 2.0f * _2bx;
      _4bz = 2.0f * _2bz;

      // Gradient decent algorithm corrective step
      s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q_[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q_[3] + _2bz * q_[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q_[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q_[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q_[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q_[2] + _2bz * q_[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q_[3] - _4bz * q_[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q_[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q_[2] - _2bz * q_[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q_[1] + _2bz * q_[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q_[0] - _4bz * q_[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q_[3] + _2bz * q_[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q_[0] + _2bz * q_[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q_[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q_[0] += qDot1 * DELTA_T;
    q_[1] += qDot2 * DELTA_T;
    q_[2] += qDot3 * DELTA_T;
    q_[3] += qDot4 * DELTA_T;

    // Normalise quaternion
    recipNorm = inv_sqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
    q_[0] *= recipNorm;
    q_[1] *= recipNorm;
    q_[2] *= recipNorm;
    q_[3] *= recipNorm;
  }

  void accGyroEstimate(float gx, float gy, float gz, float ax, float ay, float az)
  {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q_[1] * gx - q_[2] * gy - q_[3] * gz);
    qDot2 = 0.5f * (q_[0] * gx + q_[2] * gz - q_[3] * gy);
    qDot3 = 0.5f * (q_[0] * gy - q_[1] * gz + q_[3] * gx);
    qDot4 = 0.5f * (q_[0] * gz + q_[1] * gy - q_[2] * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

      // Normalise accelerometer measurement
      recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;   

      // Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0f * q_[0];
      _2q1 = 2.0f * q_[1];
      _2q2 = 2.0f * q_[2];
      _2q3 = 2.0f * q_[3];
      _4q0 = 4.0f * q_[0];
      _4q1 = 4.0f * q_[1];
      _4q2 = 4.0f * q_[2];
      _8q1 = 8.0f * q_[1];
      _8q2 = 8.0f * q_[2];
      q0q0 = q_[0] * q_[0];
      q1q1 = q_[1] * q_[1];
      q2q2 = q_[2] * q_[2];
      q3q3 = q_[3] * q_[3];

      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2* ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q_[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q_[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q_[3] - _2q1 * ax + 4.0f * q2q2 * q_[3] - _2q2 * ay;
      recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q_[0] += qDot1 * DELTA_T;
    q_[1] += qDot2 * DELTA_T;
    q_[2] += qDot3 * DELTA_T;
    q_[3] += qDot4 * DELTA_T;

    // Normalise quaternion
    recipNorm = inv_sqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
    q_[0] *= recipNorm;
    q_[1] *= recipNorm;
    q_[2] *= recipNorm;
    q_[3] *= recipNorm;
  }
};
#endif

/*
******************************************************************************
* File Name          : complementary_ahrs.h
* Description        : ahrs estimation by complementary filter
******************************************************************************
*/
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif


#ifndef __COMPLEMENTARY_AHRS_H
#define __COMPLEMENTARY_AHRS_H

#include "state_estimate/attitude/estimator.h"

#define GYR_CMPF_FACTOR 600
#define GYR_CMPFM_FACTOR 250
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#define PRESCLAER_ACC 3 // if value=1, it means same rate with gyro, for genral attitude estimation

#define G_MIN 72
#define G_MAX 133

class ComplementaryAHRS: public EstimatorAlgorithm
{
public:
  ComplementaryAHRS():EstimatorAlgorithm(), prev_mag_()
  {}

  ap::Quaternion getQuaternion()
  {
    ap::Quaternion q;
    q.from_rotation_matrix(rot_);
    return q;
  }

private:

  ap::Vector3f prev_mag_; /* for lpf filter method for mag */

  /* core esitmation process, using body frame */
  void estimation() 
  {
    EstimatorAlgorithm::estimation();

    int  valid_acc = 0;
    static int cnt = 0;

    float acc_magnitude = acc_ * acc_; //norm?
    ap::Vector3f est_g_b_tmp = est_g_;
    ap::Vector3f est_m_b_tmp = est_m_;

    ap::Vector3f gyro_rotate = gyro_  * DELTA_T;

    est_m_ += (est_m_b_tmp % gyro_rotate  ); //rotation by gyro
    est_g_ += (est_g_b_tmp % gyro_rotate ); //rotation by gyro

    if( G_MIN < acc_magnitude && acc_magnitude < G_MAX) valid_acc = 1;
    else valid_acc = 0;

    //*********************************************************************
    //** Apply complimentary filter (Gyro drift correction) 
    //** If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range 
    //**    => we neutralize the effect of accelerometers in the angle estimation. 
    //** To do that, we just skip filter, as EstV already rotated by Gyro 
    //*********************************************************************

    est_g_b_tmp = est_g_;
    est_m_b_tmp = est_m_;

    /* acc correction */
    if ( valid_acc == 1 && cnt == 0)
      est_g_ = (est_g_b_tmp * GYR_CMPF_FACTOR + acc_) * INV_GYR_CMPF_FACTOR;

    /* mag correction */
    if ( prev_mag_ != mag_ )
      {
        prev_mag_ = mag_;
        est_m_ = (est_m_b_tmp * GYR_CMPFM_FACTOR  + mag_) * INV_GYR_CMPFM_FACTOR;
      }

    ap::Vector3f wz_b = est_g_.normalized();
    ap::Vector3f wy_b = wz_b % est_m_;
    wy_b.normalize();
    ap::Vector3f wx_b = wy_b % wz_b;
    wx_b.normalize();

    ap::Matrix3f rot = ap::Matrix3f(wx_b, wy_b, wz_b); // rotation of body frame w.r.t. the world frame. Transpose from row to col

    /* consider mag declination */
    rot_ = mag_dec_rot_ * rot;

    /* avoid illness */
    if(rot_.is_nan()) rot_.identity();

    /* update */
    if(valid_acc) cnt++;
    if(cnt == PRESCLAER_ACC) cnt = 0;
  }
};


#endif

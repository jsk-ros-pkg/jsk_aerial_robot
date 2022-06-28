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
  ComplementaryAHRS():EstimatorAlgorithm(), est_g_(),  est_m_(){}

private:
  std::array<ap::Vector3f, 2> est_g_,  est_m_;

  /* core esitmation process, using body frame */
  void estimation() 
  {
    EstimatorAlgorithm::estimation();

    int  valid_acc = 0;
    static int cnt = 0;

    float acc_magnitude = acc_[Frame::BODY] * acc_[Frame::BODY]; //norm?
    ap::Vector3f est_g_b_tmp = est_g_[Frame::BODY];
    ap::Vector3f est_m_b_tmp = est_m_[Frame::BODY];

    ap::Vector3f gyro_rotate = gyro_[Frame::BODY]  * DELTA_T;

    est_m_[Frame::BODY] += (est_m_b_tmp % gyro_rotate  ); //rotation by gyro
    est_g_[Frame::BODY] += (est_g_b_tmp % gyro_rotate ); //rotation by gyro

    if( G_MIN < acc_magnitude && acc_magnitude < G_MAX) valid_acc = 1;
    else valid_acc = 0;

    //*********************************************************************
    //** Apply complimentary filter (Gyro drift correction) 
    //** If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range 
    //**    => we neutralize the effect of accelerometers in the angle estimation. 
    //** To do that, we just skip filter, as EstV already rotated by Gyro 
    //*********************************************************************

    est_g_b_tmp = est_g_[Frame::BODY];
    est_m_b_tmp = est_m_[Frame::BODY];

    /* acc correction */
    if ( valid_acc == 1 && cnt == 0)
      est_g_[Frame::BODY] = (est_g_b_tmp * GYR_CMPF_FACTOR + acc_[Frame::BODY]) * INV_GYR_CMPF_FACTOR;

    /* mag correction */
    if ( prev_mag_ != mag_[Frame::BODY] )
      {
        prev_mag_ = mag_[Frame::BODY];
        est_m_[Frame::BODY] = (est_m_b_tmp * GYR_CMPFM_FACTOR  + mag_[Frame::BODY]) * INV_GYR_CMPFM_FACTOR;
      }

    // Attitude of the estimated vector
    float sq_g_x_sq_g_z = est_g_[Frame::BODY].x * est_g_[Frame::BODY].x + est_g_[Frame::BODY].z * est_g_[Frame::BODY].z;
    float sq_g_y_sq_g_z = est_g_[Frame::BODY].y * est_g_[Frame::BODY].y + est_g_[Frame::BODY].z * est_g_[Frame::BODY].z;
    float invG = ap::inv_sqrt(sq_g_x_sq_g_z + est_g_[Frame::BODY].y * est_g_[Frame::BODY].y);
    rpy_[Frame::BODY].x = atan2f(est_g_[Frame::BODY].y , est_g_[Frame::BODY].z);
    rpy_[Frame::BODY].y = atan2f(-est_g_[Frame::BODY].x , ap::inv_sqrt(sq_g_y_sq_g_z)* sq_g_y_sq_g_z);
    rpy_[Frame::BODY].z = atan2f( est_m_[Frame::BODY].z * est_g_[Frame::BODY].y - est_m_[Frame::BODY].y * est_g_[Frame::BODY].z,
                     est_m_[Frame::BODY].x * invG * sq_g_y_sq_g_z  - (est_m_[Frame::BODY].y * est_g_[Frame::BODY].y + est_m_[Frame::BODY].z * est_g_[Frame::BODY].z) * invG * est_g_[Frame::BODY].x ) + mag_declination_;


    /* virtual(CoG) frame */
    est_g_[Frame::VIRTUAL] = r_ * est_g_[Frame::BODY];
    est_m_[Frame::VIRTUAL] = r_ * est_m_[Frame::BODY];
    sq_g_x_sq_g_z = est_g_[Frame::VIRTUAL].x * est_g_[Frame::VIRTUAL].x + est_g_[Frame::VIRTUAL].z * est_g_[Frame::VIRTUAL].z;
    sq_g_y_sq_g_z = est_g_[Frame::VIRTUAL].y * est_g_[Frame::VIRTUAL].y + est_g_[Frame::VIRTUAL].z * est_g_[Frame::VIRTUAL].z;
    invG = ap::inv_sqrt(sq_g_x_sq_g_z + est_g_[Frame::VIRTUAL].y * est_g_[Frame::VIRTUAL].y);
    rpy_[Frame::VIRTUAL].x = atan2f(est_g_[Frame::VIRTUAL].y , est_g_[Frame::VIRTUAL].z);
    rpy_[Frame::VIRTUAL].y = atan2f(-est_g_[Frame::VIRTUAL].x , ap::inv_sqrt(sq_g_y_sq_g_z)* sq_g_y_sq_g_z);
    rpy_[Frame::VIRTUAL].z = atan2f( est_m_[Frame::VIRTUAL].z * est_g_[Frame::VIRTUAL].y - est_m_[Frame::VIRTUAL].y * est_g_[Frame::VIRTUAL].z,
                     est_m_[Frame::VIRTUAL].x * invG * sq_g_y_sq_g_z  - (est_m_[Frame::VIRTUAL].y * est_g_[Frame::VIRTUAL].y + est_m_[Frame::VIRTUAL].z * est_g_[Frame::VIRTUAL].z) * invG * est_g_[Frame::VIRTUAL].x ) + mag_declination_;
    //********************************************************************************:
    //** refrence1: https://sites.google.com/site/myimuestimationexperience/sensors/magnetometer
    //** refrence2: http://uav.xenocross.net/hdg.html
    //********************************************************************************

    /* update */
    if(valid_acc) cnt++;
    if(cnt == PRESCLAER_ACC) cnt = 0;
  }
};


#endif

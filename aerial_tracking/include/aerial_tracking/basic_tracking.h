/*
 1. nodelet lization => the topic betwen nodes.
 */

#ifndef BASIC_TRACKING_H
#define BASIC_TRACKING_H

#include <aerial_robot_base/FlightNav.h>
#include <aerial_robot_base/States.h>

class BasicTracking
{
 public:
  BasicTracking(){}
  ~BasicTracking(){}

  virtual void navigation(aerial_robot_base::FlightNav navi_command) { }

  inline void setPosX(float pos_x) {pos_x_ = pos_x; }
  inline float getPosX() {return pos_x_; }
  inline void setVelX(float vel_x) {vel_x_ = vel_x; }
  inline float getVelX() {return vel_x_; }
  inline void setPosY(float pos_y) {pos_y_ = pos_y; }
  inline float getPosY() {return pos_y_; }
  inline void setVelY(float vel_y) {vel_y_ = vel_y; }
  inline float getVelY() {return vel_y_; }
  inline void setPosZ(float pos_z) {pos_z_ = pos_z; }
  inline float getPosZ() {return pos_z_; }
  inline void setVelZ(float vel_z) {vel_z_ = vel_z; }
  inline float getVelZ() {return vel_z_; }
  inline void setPsi(float psi) {psi_ = psi; }
  inline float getPsi() {return psi_; }
  inline void setVelPsi(float vel_psi) {vel_psi_ = vel_psi; }
  inline float getVelPsi() {return vel_psi_; }
  inline void setTheta(float theta) {theta_ = theta; }
  inline float getTheta() {return theta_; }
  inline void setVelTheta(float vel_theta) {vel_theta_ = vel_theta; }
  inline float getVelTheta() {return vel_theta_; }
  inline void setPhy(float phy) {phy_ = phy; }
  inline float getPhy() {return phy_; }
  inline void setVelPhy(float vel_phy) {vel_phy_ = vel_phy; }
  inline float getVelPhy() {return vel_phy_; }


 protected:
  float pos_x_;
  float vel_x_;
  float pos_y_;
  float vel_y_;
  float pos_z_;
  float vel_z_;
  float psi_;
  float vel_psi_;
  float phy_;
  float vel_phy_;
  float theta_;
  float vel_theta_;

};



#endif



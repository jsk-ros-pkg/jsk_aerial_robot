#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <spinal/FourAxisCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>
#include <aerial_robot_msgs/FlightNav.h>

class ThetaModel
{
public:
  std::vector<double> P_grid;
  std::vector<double> T_grid;
  std::vector<std::vector<double>> theta_grid;

  ThetaModel() = default;
  void buildSurface();

  /// Bicubic Hermite 補間
  double f_theta_bicubic(double P, double T) const;

private:
  bool built_ = false;
  std::vector<std::vector<double>> fx_, fy_, fxy_; // ∂θ/∂P, ∂θ/∂T, ∂²θ/(∂P∂T)

  static inline double clampDouble(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }
};

/**
 * @brief z->P_ref(z) と θ_est(P,T) を計算し、状態に応じて pressure_cmd を出すノード
 *
 * 状態:
 *  - APPROACH: パーチング開始距離より遠い。P は低め（ほぼ0〜少し）で姿勢安定優先
 *  - PRE_PERCH: パーチング準備ゾーン。z に応じて線形に 30kPa まで上げる
 *  - PERCH: パーチング実行ゾーン。P を P_perch_hold_ (例: 50kPa) にする
 */
class DeformationPlanning
{
public:
  enum class Phase
  {
    APPROACH = 0,
    PRE_PERCH,
    PERCH
  };

  DeformationPlanning(ros::NodeHandle& nh);
  void spin();

private:
  ros::NodeHandle nh_;

  ros::Subscriber dist_sub_;
  ros::Subscriber thrust_sub_;
  ros::Subscriber pressure_cur_sub_;
  ros::Subscriber interaction_state_sub_;

  ros::Publisher pressure_cmd_joint_pub_;
  ros::Publisher pressure_cmd_bottom_pub_;
  ros::Publisher pressure_sim_pub_;
  ros::Publisher move_cmd_pub_;
  ros::Publisher theta_est_pub_;
  ros::Publisher halt_pub_;
  ros::Publisher land_pub_;
  ros::Publisher phase_pub_;        // 現在の状態を 0,1,2 で出す
  ros::Publisher interaction_pub_;


  ThetaModel theta_model_;


  Phase phase_ = Phase::APPROACH;

  // z [m]
  double z_meas_  = std::numeric_limits<double>::quiet_NaN();
  double z_lpf_   = std::numeric_limits<double>::quiet_NaN();
  bool   z_inited_ = false;

  double z_cmd_ = 0.0;
  bool   z_cmd_inited_ =false;
  double v_down_approach_ = -0.5;
  double v_down_preperch_ = -0.02;
  double z_vel_est_ = 0.0;
  double z_prev_ = 0.0;
  double Kp_vel_z_ = 0.1;

  // pressure
  double P_cur_meas_ = 0.0;
  double P_cmd_filt_ = 0.0;
  bool   P_cmd_inited_ = false;
  double P_cmd_bottom_ = 0.0;
  
  // thrust
  std::vector<float> thrust_cur_; // 現在の4モータ推力
  double thrust_avr_ = 0.0;            // 代表推力（平均値）

  ros::Time last_update_;

  //z
  double z_perch_start_;  // [m] PRE_PERCH に入るしきい値 (この距離以下で準備開始)
  double z_perch_touch_;  // [m] PERCH に入るしきい値 (ほぼ腕の高さ)
  double P_preperch_max_prs_; // PRE_PERCHゾーンでの最大圧
  double P_approach_prs_;

  double P_perch_hold_;   // [kPa] PERCH時の圧（例: 50kPa）

  // LPF / レート制限
  double lpf_z_tau_sec_;        // zのLPF時定数
  double lpf_pressure_tau_sec_; // P_cmdのLPF時定数
  double max_pressure_rate_kpa_per_sec_; // 圧力指令の最大変化速度

  int halt_cnt_;

  void initThetaModel();

  void distanceCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void distanceCallback(const std_msgs::Int16::ConstPtr& msg);
  void thrustCallback(const spinal::FourAxisCommand::ConstPtr& msg);
  void pressureCurCallback(const std_msgs::Float32::ConstPtr& msg);
  void interactionStateCallback(const std_msgs::Int8::ConstPtr& msg);

  void updateZ(double dt);
  void updateThrustavr();
  void updateZCommand(double dt);
  void updateZVelocity(double dt);
  void publishNavCommand();

  void updatePhase();         // 状態遷移
  double computePRef(); // 現在の phase_ と z_lpf_ から P_ref を決める

  void updatePressureCmd(double P_ref, double dt);

  int interaction_state_;
  bool finished_ = false;
  bool finish_published_ = false;

  static inline double clampDouble(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }
};

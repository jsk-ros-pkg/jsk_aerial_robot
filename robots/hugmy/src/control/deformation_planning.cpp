#include<hugmy/control/deformation_planning.h>

void ThetaModel::buildSurface()
{
  const int nP = static_cast<int>(P_grid.size());
  const int nT = static_cast<int>(T_grid.size());
  if (nP == 0 || nT == 0 || theta_grid.size() != static_cast<size_t>(nP)) {
    ROS_ERROR("[ThetaModel] invalid grid size");
    built_ = false;
    return;
  }

  fx_.assign(nP, std::vector<double>(nT, 0.0));
  fy_.assign(nP, std::vector<double>(nT, 0.0));
  fxy_.assign(nP, std::vector<double>(nT, 0.0));

  // fx (∂θ/∂P)
  for (int i = 0; i < nP; ++i) {
    for (int j = 0; j < nT; ++j) {
      if (i == 0) {
        double dP = P_grid[i+1] - P_grid[i];
        fx_[i][j] = (theta_grid[i+1][j] - theta_grid[i][j]) / dP;
      } else if (i == nP-1) {
        double dP = P_grid[i] - P_grid[i-1];
        fx_[i][j] = (theta_grid[i][j] - theta_grid[i-1][j]) / dP;
      } else {
        double dP = P_grid[i+1] - P_grid[i-1];
        fx_[i][j] = (theta_grid[i+1][j] - theta_grid[i-1][j]) / dP;
      }
    }
  }

  // fy (∂θ/∂T)
  for (int i = 0; i < nP; ++i) {
    for (int j = 0; j < nT; ++j) {
      if (j == 0) {
        double dT = T_grid[j+1] - T_grid[j];
        fy_[i][j] = (theta_grid[i][j+1] - theta_grid[i][j]) / dT;
      } else if (j == nT-1) {
        double dT = T_grid[j] - T_grid[j-1];
        fy_[i][j] = (theta_grid[i][j] - theta_grid[i][j-1]) / dT;
      } else {
        double dT = T_grid[j+1] - T_grid[j-1];
        fy_[i][j] = (theta_grid[i][j+1] - theta_grid[i][j-1]) / dT;
      }
    }
  }

  // fxy (∂²θ/∂P∂T)
  for (int i = 0; i < nP; ++i) {
    for (int j = 0; j < nT; ++j) {
      int i0 = std::max(0, i-1), i1 = std::min(nP-1, i+1);
      int j0 = std::max(0, j-1), j1 = std::min(nT-1, j+1);
      double dP = P_grid[i1] - P_grid[i0];
      double dT = T_grid[j1] - T_grid[j0];
      if (dP > 0 && dT > 0) {
        fxy_[i][j] =
          (theta_grid[i1][j1] - theta_grid[i1][j0]
          -theta_grid[i0][j1] + theta_grid[i0][j0]) / (dP * dT);
      } else {
        fxy_[i][j] = 0.0;
      }
    }
  }

  built_ = true;
}

double ThetaModel::f_theta_bicubic(double P, double T) const
{
  if (!built_) {
    ROS_WARN_THROTTLE(2.0, "[ThetaModel] surface not built, returning 0");
    return 0.0;
  }

  // 1) クランプ
  P = clampDouble(P, P_grid.front(), P_grid.back());
  T = clampDouble(T, T_grid.front(), T_grid.back());

  // 2) セルを探す
  int i = static_cast<int>(std::lower_bound(P_grid.begin(), P_grid.end(), P) - P_grid.begin()) - 1;
  int j = static_cast<int>(std::lower_bound(T_grid.begin(), T_grid.end(), T) - T_grid.begin()) - 1;
  if (i < 0) i = 0;
  if (j < 0) j = 0;
  if (i >= static_cast<int>(P_grid.size()) - 1) i = static_cast<int>(P_grid.size()) - 2;
  if (j >= static_cast<int>(T_grid.size()) - 1) j = static_cast<int>(T_grid.size()) - 2;

  double P0 = P_grid[i],   P1 = P_grid[i+1];
  double T0 = T_grid[j],   T1 = T_grid[j+1];
  double dx = P1 - P0,     dy = T1 - T0;
  double u = (dx > 0) ? (P - P0) / dx : 0.0;
  double v = (dy > 0) ? (T - T0) / dy : 0.0;

  auto h00 = [](double t){ return  2*t*t*t - 3*t*t + 1; };
  auto h10 = [](double t){ return      t*t*t - 2*t*t + t; };
  auto h01 = [](double t){ return -2*t*t*t + 3*t*t; };
  auto h11 = [](double t){ return      t*t*t -   t*t; };

  double f00 = theta_grid[i  ][j  ]; double f10 = theta_grid[i+1][j  ];
  double f01 = theta_grid[i  ][j+1]; double f11 = theta_grid[i+1][j+1];

  double fx00 = fx_[i  ][j  ]; double fx10 = fx_[i+1][j  ];
  double fx01 = fx_[i  ][j+1]; double fx11 = fx_[i+1][j+1];

  double fy00 = fy_[i  ][j  ]; double fy10 = fy_[i+1][j  ];
  double fy01 = fy_[i  ][j+1]; double fy11 = fy_[i+1][j+1];

  double fxy00 = fxy_[i  ][j  ]; double fxy10 = fxy_[i+1][j  ];
  double fxy01 = fxy_[i  ][j+1]; double fxy11 = fxy_[i+1][j+1];

  double val =
    h00(u)*h00(v)*f00 + h01(u)*h00(v)*f10 + h00(u)*h01(v)*f01 + h01(u)*h01(v)*f11
  + h10(u)*h00(v)*(fx00*dx) + h11(u)*h00(v)*(fx10*dx) + h10(u)*h01(v)*(fx01*dx) + h11(u)*h01(v)*(fx11*dx)
  + h00(u)*h10(v)*(fy00*dy) + h01(u)*h10(v)*(fy10*dy) + h00(u)*h11(v)*(fy01*dy) + h01(u)*h11(v)*(fy11*dy)
  + h10(u)*h10(v)*(fxy00*dx*dy) + h11(u)*h10(v)*(fxy10*dx*dy)
  + h10(u)*h11(v)*(fxy01*dx*dy) + h11(u)*h11(v)*(fxy11*dx*dy);

  return val;
}

// =========================================

DeformationPlanning::DeformationPlanning(ros::NodeHandle& nh)
  : nh_(nh)
{
  nh_.param("z_perch_start",  z_perch_start_,  0.8);   // [m]
  nh_.param("z_perch_touch",  z_perch_touch_,  0.5);   // [m]
  nh_.param("P_preperch_max", P_preperch_max_prs_, 30.0);  // [kPa]
  nh_.param("P_approach",     P_approach_prs_, 0.0); // [kPa]
  nh_.param("P_perch_hold",   P_perch_hold_,   50.0);  // [kPa]

  nh_.param("lpf_z_tau_sec",        lpf_z_tau_sec_,        0.1);
  nh_.param("lpf_pressure_tau_sec", lpf_pressure_tau_sec_, 0.2);
  nh_.param("max_pressure_rate_kpa_per_sec", max_pressure_rate_kpa_per_sec_, 30.0);

  nh_.param("v_down_approach",  v_down_approach_,  -0.1);
  nh_.param("v_down_preperch",  v_down_preperch_,  -0.05);


  // dist_sub_ = nh_.subscribe<std_msgs::Float32>("/perch/target_distance_z", 1, &DeformationPlanning::distanceCallback, this);
  dist_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/quadrotor/mocap/pose", 1, &DeformationPlanning::distanceCallback, this);
  // thrust_sub_ = nh_.subscribe<spinal::Thrust>("/quadrotor/target_thrust", 1, &DeformationPlanning::thrustCallback, this);
  thrust_sub_ = nh_.subscribe<spinal::FourAxisCommand>("/quadrotor/four_axes/command", 1, &DeformationPlanning::thrustCallback, this);
  pressure_cur_sub_ = nh_.subscribe<std_msgs::Float32>("/quadrotor/arm/filterd_joint_cur_pressure", 1, &DeformationPlanning::pressureCurCallback, this);
  // pressure_cur_sub_ = nh_.subscribe<std_msgs::Float32>("/quadrotor/arm/sim_pressure", 1, &DeformationPlanning::pressureCurCallback, this);
  halt_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/halt", 1);
  pressure_cmd_pub_ = nh_.advertise<std_msgs::Int8>("/air/target_joint", 1);
  //simulation
  // pressure_sim_pub_ = nh_.advertise<std_msgs::Float32>("/quadrotor/arm/sim_pressure", 1);
  theta_est_pub_ = nh_.advertise<std_msgs::Float32>("/quadrotor/debug/arm/theta_est", 1);
  phase_pub_ = nh_.advertise<std_msgs::Float32>("/quadrotor/debug/arm/phase", 1); // 0:APPROACH,1:PRE_PERCH,2:PERCH
  move_cmd_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>("/quadrotor/uav/nav", 1);
  
  last_update_ = ros::Time::now();

  initThetaModel();
}

void DeformationPlanning::initThetaModel()
{
  theta_model_.P_grid = {0,5,10,15,20,25,30,35,40,45,50};
  theta_model_.T_grid = {0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7};
  theta_model_.theta_grid = {
    {139.768,134.493,128.551,124.460,119.977,113.315,101.055,89.273,63.946,3.289,9.258,0.227,0.472,0.430,0},
    {144.643,139.467,137.555,132.286,128.972,121.224,111.347,104.471,89.099,42.255,46.203,1.461,1.736,0.496,0.549},
    {147.598,145.220,141.971,137.005,132.727,125.932,123.080,115.013,100.787,33.402,57.619,1.770,5.143,2.577,3.129},
    {152.154,149.182,146.960,141.890,138.102,131.389,126.483,120.379,110.474,93.181,92.292,31.991,13.342,11.450,7.433},
    {157.131,156.080,152.754,146.997,143.930,137.597,131.136,127.663,120.223,104.781,98.179,43.062,43.385,30.329,27.754},
    {161.013,158.571,155.689,153.529,147.594,143.465,139.130,133.446,125.879,124.570,105.655,86.253,53.938,46.389,41.418},
    {164.280,162.752,160.557,155.001,153.065,148.041,143.542,138.050,132.759,175.925,116.943,98.069,93.442,92.621,56.854},
    {167.100,164.741,163.243,158.070,155.979,152.618,147.705,141.981,133.668,176.184,123.087,111.130,106.004,104.060,93.947},
    {168.751,168.114,163.771,162.540,159.212,156.223,153.237,148.180,141.759,129.246,128.565,105.031,114.042,109.748,104.201},
    {170.385,169.522,166.386,163.503,163.294,160.484,156.870,152.493,145.866,133.232,135.627,111.236,172.964,115.230,115.908},
    {174.290,166.915,170.521,169.391,166.849,163.922,159.505,158.103,151.329,139.333,143.576,116.591,126.471,125.471,122.980}
  };

  theta_model_.buildSurface();
}

void DeformationPlanning::distanceCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  z_meas_ = msg->pose.position.z;
}

void DeformationPlanning::thrustCallback(const spinal::FourAxisCommand::ConstPtr& msg)
{
  thrust_cur_ = msg->base_thrust;
}

void DeformationPlanning::pressureCurCallback(const std_msgs::Float32::ConstPtr& msg)
{
  P_cur_meas_ = msg->data;
}

void DeformationPlanning::updateZ(double dt)
{
  if (std::isnan(z_meas_)) return;
  double alpha = dt / (lpf_z_tau_sec_ + dt);
  if (!z_inited_) {
    z_lpf_ = z_meas_;
    z_inited_ = true;
  } else {
    z_lpf_ += alpha * (z_meas_ - z_lpf_);
  }
}

void DeformationPlanning::updateThrustavr()
{
  if (thrust_cur_.empty()) return;
  double sum = 0.0;
  for (float t : thrust_cur_) sum += t;
  thrust_avr_ = sum / static_cast<double>(thrust_cur_.size());
}


void DeformationPlanning::updateZCommand(double dt)
{
  if (!z_inited_) return;

  if (!z_cmd_inited_) {
    z_cmd_ = z_lpf_;
    z_cmd_inited_ = true;
  }

  double v_down = 0.0;
  double z_floor = z_perch_start_;
  double z_cmd_pos;

  switch (phase_) {
  case Phase::APPROACH:
    v_down = v_down_approach_;
    z_floor = z_perch_start_;
    break;

  case Phase::PRE_PERCH:
    v_down = v_down_preperch_;
    z_floor = z_perch_touch_;
    break;

  case Phase::PERCH:
    v_down = 0.0;
    z_floor = z_perch_touch_;
    break;
  }

  z_cmd_ = v_down;
  // z_cmd_pos = z_cmd_ * dt;
  // if (z_cmd_pos < z_floor) z_cmd_ = 0.0;
}


void DeformationPlanning::publishNavCommand()
{
  if (!z_cmd_inited_) return;
  aerial_robot_msgs::FlightNav nav;
  nav.control_frame = 1;
  nav.target = 1;
  nav.pos_z_nav_mode = 1;
  nav.target_vel_z = z_cmd_;

  move_cmd_pub_.publish(nav);
}


void DeformationPlanning::updatePhase()
{
  if (!z_inited_) return;
  switch (phase_) {
  case Phase::APPROACH:
    halt_cnt_ = 0;
    // ROS_INFO("z_inited: %d", z_inited_);
    if (z_lpf_ <= z_perch_start_) {
      phase_ = Phase::PRE_PERCH;
      ROS_INFO("[Deformationplanning] Phase -> PRE_PERCH");
    }
    break;

  case Phase::PRE_PERCH:
    halt_cnt_ = 0;
    if (z_lpf_ <= z_perch_touch_) {
      phase_ = Phase::PERCH;
      ROS_INFO("[Deformationplanning] Phase -> PERCH");
    } else if (z_lpf_ > z_perch_start_ + 0.05) {
      phase_ = Phase::APPROACH;
      ROS_INFO("[Deformationplanning] Phase -> APPROACH");
    }
    break;

  case Phase::PERCH:
    halt_cnt_+= 1;
    if (halt_cnt_ > 100){
      std_msgs::Empty e;
      halt_pub_.publish(e);
    }
    // 離脱条件を書く（z がまた大きくなりすぎたらAPPROACHに戻す, roll,pitchおかしい，小さくなりすぎたら復帰（Approachに戻す））
    if (z_lpf_ > z_perch_start_ + 0.1) {
      phase_ = Phase::APPROACH;
      ROS_WARN("[Deformationplanning] Phase PERCH aborted -> APPROACH");
    }
    break;
  }
}

double DeformationPlanning::computePRef() const
{
  if (!z_inited_) {
    return P_approach_prs_;
  }

  switch (phase_) {
  case Phase::APPROACH:
    return P_approach_prs_;

  case Phase::PRE_PERCH:
    {
      // z_perch_start_ ->  z_perch_touch_ の間で P_approach_level_ ->  P_preperch_max_ へ線形に上げる
      double z0 = z_perch_start_;
      double z1 = z_perch_touch_;
      double z   = clampDouble(z_lpf_, z1, z0); // z1 <= z <= z0
      double s   = (z0 - z) / std::max(1e-6, (z0 - z1)); // z=z0 -> 0, z=z1 -> 1
      double P   = P_approach_prs_ + s * (P_preperch_max_prs_ - P_approach_prs_);
      return clampDouble(P, 0.0, P_preperch_max_prs_);
    }

  case Phase::PERCH:
    return P_perch_hold_;
  }

  return P_approach_prs_;
}

void DeformationPlanning::updatePressureCmd(double P_ref, double dt)
{
  const double tau = std::max(1e-3, lpf_pressure_tau_sec_);
  const double alpha = dt / (tau + dt);

  if (!P_cmd_inited_) {
    P_cmd_filt_ = P_ref;
    P_cmd_inited_ = true;
  } else {
    double prev = P_cmd_filt_;
    double raw  = P_ref;
    double lpf_out = prev + alpha * (raw - prev);

    double max_step = max_pressure_rate_kpa_per_sec_ * dt;
    double delta = lpf_out - prev;
    if (delta > max_step) lpf_out = prev + max_step;
    if (delta < -max_step) lpf_out = prev - max_step;

    P_cmd_filt_ = lpf_out;
  }
}

void DeformationPlanning::spin()
{
  ros::Rate rate(100.0);
  last_update_ = ros::Time::now();

  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    double dt = (now - last_update_).toSec();
    if (dt <= 0.0) dt = 1e-3;
    last_update_ = now;

    updateZ(dt);
    updateThrustavr();

    updateZCommand(dt);

    updatePhase();

    double P_ref = computePRef();
    updatePressureCmd(P_ref, dt);

    double theta_est_deg = theta_model_.f_theta_bicubic(P_cmd_filt_, thrust_avr_);

    // publish
    std_msgs::Float32 msg_simP, msgTheta, msgPhase;
    std_msgs::Int8 msgP;
    msg_simP.data = static_cast<float>(P_cmd_filt_);
    msgP.data     = static_cast<int>(P_cmd_filt_);
    msgTheta.data = static_cast<float>(theta_est_deg);
    msgPhase.data = static_cast<float>(
      (phase_ == Phase::APPROACH) ? 0.0f :
      (phase_ == Phase::PRE_PERCH) ? 1.0f : 2.0f
    );

    pressure_sim_pub_.publish(msg_simP);
    pressure_cmd_pub_.publish(msgP);
    theta_est_pub_.publish(msgTheta);
    phase_pub_.publish(msgPhase);
    publishNavCommand();

    ROS_INFO_STREAM_THROTTLE(0.5,
      "[Deformationplanning] phase=" << static_cast<int>(msgPhase.data)
      << " z_lpf=" << (z_inited_ ? z_lpf_ : -1.0)
      << " P_ref=" << P_ref
      << " P_cmd=" << P_cmd_filt_
      << " T_avr=" << thrust_avr_
      << " theta_est=" << theta_est_deg
    );

    ros::spinOnce();
    rate.sleep();
  }
}


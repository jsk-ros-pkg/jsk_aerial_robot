#include <hugmy/control/air_pressure_controller.h>

AirPressureController::AirPressureController(ros::NodeHandle& nh) {
    sensor_joint_sub_ = nh.subscribe("/sensor", 1, &AirPressureController::sensorCb, this);
    sensor_bottom_sub_ = nh.subscribe("/sensor_1", 1, &AirPressureController::sensor1Cb, this);
    pwm_air_pub_ = nh.advertise<spinal::PwmTest>("/pwm_cmd/air", 1);
    pwm_pub_ = nh.advertise<spinal::PwmTest>("/quadrotor/pwm_test", 1);
    // pwm_pub_ = nh.advertise<spinal::PwmTest>("/pwm_test", 1);
    joint_filtered_pub_  = nh.advertise<std_msgs::Float32>("/quadrotor/arm/filterd_joint_cur_pressure", 1);

    pwm_air_cmd_.motor_index.clear();
    pwm_air_cmd_.pwms.clear();
    last_published_pwm_ = pwm_air_cmd_;

    ros::NodeHandle pnh("~");

    pnh.param("test_mode", test_mode_, true); //if test_mode = false, merge haptics
    pnh.param("kp_joint",  kp_joint_,  0.38);
    pnh.param("ki_joint",  ki_joint_,  0.03);
    pnh.param("kd_joint",  kd_joint_,  0.003);
    pnh.param("kp_bottom", kp_bottom_, 0.04);
    pnh.param("ki_bottom", ki_bottom_, 0.005);
    pnh.param("kd_bottom", kd_bottom_, 0.001);
    pnh.param("u_limit",   u_limit_,   0.8);


    pnh.param("leak_calib_enable",   leak_calib_enable_,   true);
    pnh.param("calib_sensor_index",  calib_sensor_index_,  0);
    pnh.param("calib_target_pressure", calib_target_pressure_, 20);
    pnh.param("calib_duration",      calib_duration_,      8.0);
    pnh.param("calib_min_pg",        calib_min_pg_,        5.0);

    pnh.param("enable_leak_ff",      enable_leak_ff_,      true);
    pnh.param("k_ff",                k_ff_,                0.01);

    pnh.param("enable_gain_sched",   enable_gain_sched_,   true);
    pnh.param("alpha_sched",         alpha_sched_,         0.1);
    pnh.param("kp_leak_scale",       kp_leak_scale_,       0.0);
    pnh.param("ki_leak_scale",       ki_leak_scale_,       0.02);
    pnh.param("external_mode",       external_mode_,       false); //if external_mode = true, control_loop is turning on  
    pnh.param("control_rate_hz",     control_rate_hz_,     50.0);

    pnh.param("lpf_tau",  lpf_tau_,  0.05);
    ros::Duration(5.0).sleep();
    ROS_INFO("SLEEP");
    if (!leak_calib_finished_){
      leak_timer_ = nh.createTimer(ros::Duration(0.01), &AirPressureController::leakTimerCb, this, /*oneshot=*/false, /*autostart=*/false);
      if (leak_calib_enable_){
        startLeakCalibration(calib_sensor_index_);
      }
      ROS_INFO("leakcalib: %d", leak_calib_enable_);
    }
    ROS_INFO("leakcalib: %d", leak_calib_finished_);
    target_joint_sub_  = nh.subscribe<std_msgs::Int8>("/air/target_joint",  1, &AirPressureController::targetJointCb,  this);
    target_bottom_sub_ = nh.subscribe<std_msgs::Int8>("/air/target_bottom", 1, &AirPressureController::targetBottomCb, this);
    const double dt = 1.0 / std::max(1.0, control_rate_hz_);
    control_timer_ = nh.createTimer(ros::Duration(dt), &AirPressureController::controlLoopCb, this);
}

// void AirPressureController::update(){
//     adjustAirPressure();
// }

void AirPressureController::sensorCb(const std_msgs::Int8::ConstPtr& msg) {
    air_pressure_joint_ = msg->data;
    std_msgs::Float32 x;
    x.data = static_cast<float> (air_pressure_joint_);
    //  if(!joint_filt_inited_){
    //   joint_filt_ = x;
    //   joint_filt_inited_ = true;
    // }else{
    //   const double dt = (now - joint_last_).toSec();
    //   const double alpha = (dt > 0.0)? dt / (lpf_tau_ + dt) :1.0;
    //   joint_filt_ += alpha * (x - joint_filt_);
    // }
    // joint_last_ = now;

    // std_msgs::Float32 out;
    // out.data = static_cast<float>(joint_filt_);
    joint_filtered_pub_.publish(x);
}


void AirPressureController::sensor1Cb(const std_msgs::Int8::ConstPtr& msg) {
    air_pressure_bottom_ = msg->data;
}

void AirPressureController::stopCb(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data) {
    initializePneumatics();
    ROS_WARN("[Air] received STOP, all pneumatics off");
  }
}


void AirPressureController::startLeakCalibration(int sensor_index)
{
  if (leak_calib_running_) return;
  calib_sensor_index_ = sensor_index;
  t_log_.clear();
  pg_log_.clear();

  if (calib_sensor_index_ == 0){
    stopAllSV();
    ros::Duration(0.3).sleep();
    // initializePneumatics();
  }else{
    startSVSwitch();
    ros::Duration(0.3).sleep();
    stopSVExhaust();
    ros::Duration(0.3).sleep();
  }
  ROS_INFO("leakcalib");
  leak_pressurizing_ = true;
  press_start_ = ros::Time::now();
  ros::NodeHandle nh;
  press_timer_ = nh.createTimer(ros::Duration(press_check_period_),&AirPressureController::pressTimerCb, this, false,true);
  const int sensor_now = (calib_sensor_index_ == 0) ? air_pressure_joint_ : air_pressure_bottom_;
  if (sensor_now < calib_target_pressure_ - 1){
    ROS_ERROR("[LeakCalib] done sensor=%d, sensor now=%d", calib_target_pressure_, sensor_now);
    calPressure(calib_target_pressure_, calib_sensor_index_);
    inicialPump();
    ROS_ERROR("adjust pump done");
    return;
  }
  stopPump();
  ROS_INFO("stop pump done");
  calib_start_ = ros::Time::now();
  leak_calib_running_ = true;
  leak_timer_.start();
  ROS_WARN("[LeakCalib] started on sensor=%d target= %d kPa", calib_sensor_index_, calib_target_pressure_);
}

void AirPressureController::pressTimerCb(const ros::TimerEvent&)
{
  if (!leak_pressurizing_) return;

  const int sensor_now = (calib_sensor_index_ == 0) ? air_pressure_joint_ : air_pressure_bottom_;
  // const int target = calib_target_pressure_;

  if (sensor_now < calib_target_pressure_ - 1) {
    ROS_INFO("[LeakCalib] done sensor=%d, sensor now=%d", calib_target_pressure_, sensor_now);
    calPressure(calib_target_pressure_, calib_sensor_index_);
    adjustPump();

    if ((ros::Time::now() - press_start_).toSec() > press_timeout_sec_) {
      ROS_WARN("[LeakCalib] pressurizing timeout. Stopping pump for safety.");
      stopPump();
      leak_pressurizing_ = false;
      press_timer_.stop();
      return;
    }
    return; //continue
  }

  //reach, measured
  ROS_INFO("stop pump done");
  stopPump();

  leak_pressurizing_ = false;
  press_timer_.stop();

  calib_start_ = ros::Time::now();
  leak_calib_running_ = true;
  leak_timer_.start();
  ROS_INFO("[LeakCalib] calib started on sensor=%d target= %d kPa", sensor_now, calib_target_pressure_);
}


void AirPressureController::leakTimerCb(const ros::TimerEvent& ev)
{
  if (!leak_calib_running_) return;

  ros::Time now = ros::Time::now();
  double t = (now - calib_start_).toSec();
  int sensor_now = (calib_sensor_index_ == 0) ? air_pressure_joint_ : air_pressure_bottom_;
  double Pg = static_cast<double>(sensor_now);
  ROS_WARN("[LeakCalib] calibed on sensor=%f target= %d kPa", Pg, calib_target_pressure_);

  t_log_.push_back(t);
  pg_log_.push_back(Pg);
  ROS_WARN("[LeakCalib] t_log_.size() = %zu, pg_log_.size() = %zu", t_log_.size(), pg_log_.size());


  if (t >= calib_duration_) {
    leak_timer_.stop();
    leak_calib_running_ = false;
    leak_calib_finished_ = true;
    ROS_INFO("[LeakCalib] t >= calib_duration_ (t=%.3f)", t);

    // std::vector<double> t_fit, pg_fit;
    // for (size_t i = 0; i < t_log_.size(); ++i) {
    //   // if (i >= pg_log_.size()) {
    //   //   break;
    //   // }else{
    //   // if (pg_log_[i] >= calib_min_pg_) {
    //   t_fit.push_back(t_log_[i]);
    //   pg_fit.push_back(pg_log_[i]);
    //   ROS_INFO("forfor");
    // }
      // }

    // ROS_WARN("[LeakCalib] before estimateLeak: samples=%zu (>=%.1f kPa)", t_fit.size(), calib_min_pg_);
    // const double g = estimateLeak(t_fit, pg_fit);
    double g = estimateLeak(t_log_, pg_log_);
    ROS_INFO("[LeakCalib] estimateLeak returned: g=%.4f", g);
    if (calib_sensor_index_ == 0){
      g_leak_joint_ = std::max(0.0, g);
    }else{
      g_leak_bottom_ = std::max(0.0, g);
    }
    
    ROS_INFO("[LeakCalib] done sensor=%d -> g_leak=%.4f 1/s (samples=%zu)", calib_sensor_index_, g, t_log_.size());
    initializePneumatics();
    return;
  }
}

double AirPressureController::estimateLeak(const std::vector<double>& t, const std::vector<double>& pg)
{
  if (t.size() < 5) return 0.0;
  double Sx=0, Sy=0, Sxx=0, Sxy=0; size_t n=0;
  for (size_t i=0;i<t.size();++i){
    if (pg[i] <= 0.0) continue;
    double x = t[i];
    double y = std::log(pg[i]);
    Sx += x; Sy += y; Sxx += x*x; Sxy += x*y; ++n;
  }

  if (n < 5) return 0.0;
  if (std::fabs(n*Sxx - Sx*Sx) < 1e-9) return 0.0;
  double slope = (n*Sxy - Sx*Sy) / (n*Sxx - Sx*Sx);
  return std::max(0.0, -slope);
}

// void AirPressureController::setupExternalMode()
// {
//   if (!external_mode_) return;

//   ros::NodeHandle nh;
//   target_joint_sub_  = nh.subscribe<std_msgs::Int8>("/air/target_joint",  1, &AirPressureController::targetJointCb,  this);
//   target_bottom_sub_ = nh.subscribe<std_msgs::Int8>("/air/target_bottom", 1, &AirPressureController::targetBottomCb, this);

//   const double dt = 1.0 / std::max(1.0, control_rate_hz_);
//   control_timer_ = nh.createTimer(ros::Duration(dt), &AirPressureController::controlLoopCb, this);

//   external_setup_done = true;
//   ROS_INFO("[Air] external_setpoint_mode ON (rate=%.1f Hz)", control_rate_hz_);
// }


void AirPressureController::switchSingleMode(const std::vector<uint8_t>& induces, const std::vector<float>& pwm_values){
  if(!test_mode_){
    setAirPwm(induces, pwm_values);
    ROS_INFO("not test mode");
  }else{
    publishAirPwm(induces, pwm_values, false);
    ROS_INFO("test mode");
  }
}

void AirPressureController::switchSingleMode(uint8_t induce, float pwm_value){
  if(!test_mode_){
    setAirPwm(induce, pwm_value);
  }else{
    publishAirPwm({induce}, {pwm_value}, false);
    
  }
}

void AirPressureController::inicialPump(){
  ROS_INFO("inicial pump");
  switchSingleMode({4,6},{0.3, 0.3});
}

void AirPressureController::adjustPump(){
  switchSingleMode({4,6},{output_, output_});
  std::cout<<"pwm_value:"<<output_ <<"\n";
}

void AirPressureController::maxWorkPumptoJoint(){
  publishAirPwm({4,6,5,7},{0.9f,0.9f,0.0f,0.0f}, false);
}

void AirPressureController::startSVSwitch(){
  switchSingleMode(5, 1.0f);
}

void AirPressureController::stopSVSwitch(){
  switchSingleMode(5, 0.0f);
}

void AirPressureController::startSVExhaust(){
  switchSingleMode(7, 1.0f);
}

void AirPressureController::stopSVExhaust(){
  switchSingleMode(7, 0.0f);
}

void AirPressureController::stopAllSV(){
  switchSingleMode({5,7},{0.0, 0.0});
}

void AirPressureController::startAllSV(){
  switchSingleMode({5,7},{1.0, 1.0});
}
void AirPressureController::stopPump(){
  switchSingleMode({4,6},{0.0, 0.0});
}

void AirPressureController::stopAllPneumatics() {
  publishAirPwm({4,5,6,7},{0.0, 0.0, 0.0, 0.0}, false);
}

void AirPressureController::initializePneumatics() {
  publishAirPwm({4,5,6,7},{0.0, 0.0, 0.0, 1.0}, false); // ポンプ停止、排気開始
    output_ = 0.0; // 出力をリセット
    ROS_INFO("All pneumatics stopped.");
}

void AirPressureController::setAirPwm(uint8_t index, float pwm_value) {
    pwm_state_[index] = pwm_value;
}

void AirPressureController::setAirPwm(const std::vector<uint8_t>& induces, const std::vector<float>& pwm_values) {
    for (size_t i = 0; i < induces.size(); ++i) {
      pwm_state_[induces[i]] = pwm_values[i];
    }
  }

void AirPressureController::publishAirPwmMerged() {
    std::vector<uint8_t> indices;
    std::vector<float> values;
    for (const auto& kv : pwm_state_) {
      indices.push_back(kv.first);
      values.push_back(kv.second);
    }
    publishAirPwm(indices, values, /*to_air_bus=*/!test_mode_);
    pwm_state_.clear();
  }


void AirPressureController::publishAirPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms, bool to_air_bus)
{
    pwm_air_cmd_.motor_index = indices;
    pwm_air_cmd_.pwms        = pwms;

    if (to_air_bus) {
        pwm_air_pub_.publish(pwm_air_cmd_); //testmode=false
    } else {
      // for (size_t i = 0; i < indices.size(); ++i) {
      //   ROS_INFO("pwm_air_cmd_.motor_index: %d",pwm_air_cmd_.motor_index.at(i));
      // }
      // for (size_t i = 0; i < pwms.size(); ++i) {
      //   ROS_INFO("pwm_air_cmd_.pwms: %f",pwm_air_cmd_.pwms.at(i));
      // }
      pwm_pub_.publish(pwm_air_cmd_); //test
    }
    last_published_pwm_ = pwm_air_cmd_;
}


double AirPressureController::pid(double target, double measured, int sensor_index){
  PIDState& st = (sensor_index == 0) ? pid_joint_ : pid_bottom_;
  double kp0 = (sensor_index == 0)? kp_joint_ : kp_bottom_;
  double ki0 = (sensor_index == 0)? ki_joint_ : ki_bottom_;
  double kd0 = (sensor_index == 0)? kd_joint_ : kd_bottom_;
  double gL  = (sensor_index == 0)? g_leak_joint_ : g_leak_bottom_;
  double Pg  = std::max(0.0, measured);

  double sched = enable_gain_sched_
      ? (alpha_sched_ + (1.0 - alpha_sched_) * clamp(Pg / std::max(1e-6, static_cast<double>(joint_limit_pressure_)), 0.0, 1.0))
      : 1.0;
  double kp = kp0 * sched * (1.0 - kp_leak_scale_ * gL);
  double ki = ki0 * sched * (1.0 - ki_leak_scale_ * gL);
  double kd = kd0;
  double e = target - measured;
  ros::Time now = ros::Time::now();
  if (!st.inited) {
    st.t_prev = now;
    st.e_prev = e;
    st.I = 0.0;
    st.inited = true;
  }
  double dt = (now - st.t_prev).toSec();
  if (dt <= 0.0) dt = 1e-3;

  double de = (e - st.e_prev) / dt;
  const double u_min = 0.17;
  double uff = 0.0;
  //e>0のときでも可
  if (enable_leak_ff_){
    uff = k_ff_ * gL * Pg;
  }

  double u_lin_raw = kp * e + ki * st.I + kd * de + uff;
  double u_raw     = u_lin_raw + u_min;
  double u_sat = clamp(u_raw, 0.0, u_limit_);

  const bool saturating = (std::fabs(u_raw - u_sat) > 1e-12);

  bool unwind_direction = false;
  if (saturating) {
    if (u_raw > u_sat) unwind_direction = (e < 0.0);
    else               unwind_direction = (e > 0.0);
  }

  const double k_aw = 0.2;
  if (!saturating || unwind_direction) {
    st.I += (e + k_aw * (u_sat - u_raw)) * dt;
  }
  const double I_min = -50.0;
  const double I_max =  50.0;
  st.I = clamp(st.I, I_min, I_max);

  if (std::fabs(e) < 0.5) {
    const double I_decay = 0.05;
    st.I *= (1.0 - I_decay * dt);
  }

  st.e_prev = e;
  st.t_prev = now;

  ROS_INFO("[PID-%s] tgt=%.1f cur=%.1f e=%.1f | P=%.3f I=%.3f D=%.3f FF=%.3f -> u_raw=%.3f u_sat=%.3f,Istate=%.3f",
           (sensor_index==0 ? "joint" : "bottom"), target, measured, e, kp, ki, kd, uff, u_raw, u_sat, st.I);

  return u_sat;
}




void AirPressureController::calPressure(int target_pressure, int sensor_index) {
    int sensor_data = (sensor_index == 0) ? air_pressure_joint_ : air_pressure_bottom_;
    const double measured = static_cast<double>(sensor_data);

    double u = pid(static_cast<double>(target_pressure), measured, sensor_index);
    output_ = u;

    ROS_INFO("[PID] sensor:%d target:%d current:%d out:%.2f", sensor_index, target_pressure, sensor_data, output_);
}
//     if (error > thre) {
//         output_ = std::min(error*0.02 +0.3, 0.7);
//     } else {
//         output_ = std::max(error*0.02 + 0.32, 0.0);
//     }
// }

void AirPressureController::targetJointCb(const std_msgs::Int8::ConstPtr& msg) {
  target_joint_cmd_ = clamp_int(msg->data, 0, joint_limit_pressure_);
  has_target_joint_ = true;
}

void AirPressureController::targetBottomCb(const std_msgs::Int8::ConstPtr& msg) {
  target_bottom_cmd_ = clamp_int(msg->data, 0, bottom_limit_pressure_);
  has_target_bottom_ = true;
}

void AirPressureController::controlLoopCb(const ros::TimerEvent& e)
{
  if(!external_mode_) return;

  failsafe();
  // ROS_INFO("emergency %d", static_cast<int>(emergency_stop_.load()));
  if (emergency_stop_) {
    stopAllPneumatics();
    return;
  }

  if(target_bottom_cmd_ > 0){
    startSVSwitch();
    stopSVExhaust();
    int error = target_bottom_cmd_ - air_pressure_bottom_;
    calPressure(target_bottom_cmd_, 1);
    adjustPump();
    // if (error >= 0){
    // }else{
    //   output_ = std::max(error*0.02 + 0.20, 0.0);
    //   adjustPump();
    // }
  }
  if(has_target_joint_){
    stopAllSV();
    if (target_joint_cmd_ - air_pressure_joint_ >= 0){
      calPressure(target_joint_cmd_, 0);
      adjustPump();
    }else{
      stopPump();
    }
  }
  publishAirPwmMerged();
}

void AirPressureController::bottomPressurePrepare()
{
    startAllSV();
    ROS_INFO("bottom prepare");
    failsafe();
    if (air_pressure_bottom_ < bottom_approaching_pressure_) {
        calPressure(bottom_approaching_pressure_, 1);
        adjustPump();
    } else {
        stopPump(); 
    }
    publishAirPwmMerged();
}
void AirPressureController::readyPerching()
{   
    failsafe();
    if (air_pressure_bottom_ < bottom_ready_pressure_) {
        calPressure(bottom_ready_pressure_, 1);
        adjustPump();
        publishAirPwmMerged();
    } else {
        ROS_WARN("==============ready perching==================");
        stopAllPneumatics();
        ROS_WARN("joint to bottom!!!");
        ros::Duration(2.0).sleep();
        perching_flag_ = 2;
    }
}

void AirPressureController::startPerching()
{
    ROS_WARN("==============perching==================");
    if (air_pressure_joint_ <= joint_max_pressure_) {
        if (air_pressure_joint_ >= joint_flex_pressure_) {
            if (prepare_finished_== 0){
                prepare_finished_ = 1;
            }
            maxWorkPumptoJoint();
        } else {
            ROS_INFO("joint!!!");
            maxWorkPumptoJoint();
        }
        if (air_pressure_joint_ == joint_max_pressure_) {
            stopPump();
            publishAirPwmMerged();
            perching_flag_ = 3;
        }
    } else {
        stopPump();
        publishAirPwmMerged();
        perching_flag_ = 3;
    }
}

// void AirPressureController::adjustAirPressure() {
//     std::vector<uint8_t> indices = {4, 5, 6, 7};
//     std::vector<float> pwms(4, 0.0);  // 初期値はすべて0.0

//     if (air_pressure_bottom_ <= bottom_perching_pressure_) {
//         pwms[0] = static_cast<float>(output_);  // 4番: ポンプ
//         pwms[1] = 1.0f;                         // 5番: スイッチ
//         pwms[2] = static_cast<float>(output_);  // 6番: ポンプ
//         pwms[3] = 0.0f;                         // 7番: 排気
//         calPressure(bottom_perching_pressure_, 1);
//     } else {
//         if (air_pressure_joint_ <= joint_perching_pressure_) {
//             pwms[0] = static_cast<float>(output_);
//             pwms[1] = 0.0f;
//             pwms[2] = static_cast<float>(output_);
//             pwms[3] = 0.0f;                       
//             calPressure(joint_perching_pressure_, 0);
//         } else {
//             pwms[0] = 0.0f;
//             pwms[1] = 0.0f;
//             pwms[2] = 0.0f;
//             pwms[3] = 0.0f;
//         }
//     }

//     if (air_pressure_joint_ >= joint_max_pressure_ || air_pressure_bottom_ >= bottom_max_pressure_) {
//         ROS_WARN("Air pressure exceeded maximum limits! Stopping pump.");
//         pwms = {0.0f, 0.0f, 0.0f, 1.0f};
//     }
//     // 一括でpublish
//     publishAirPwm(indices, pwms);
//     ROS_INFO("Air Pressure Joint: %d, Bottom: %d, Output: %.2f", air_pressure_joint_, air_pressure_bottom_, output_);
// }

void AirPressureController::keepPerching()
{
    // ROS_INFO("%d", cnt_);
    // cnt_ += 1;
    // if (cnt_ >= 50) {
    //     cnt_ = 0;
    //     ros::Duration(1.0).sleep();
    //     ROS_INFO("deperch ready");
    //     perching_flag_ = 4;
    // } else {
    if (air_pressure_bottom_ <= bottom_perching_pressure_) {
      startSVSwitch();
      calPressure(bottom_perching_pressure_, 1);
      adjustPump();
      publishAirPwmMerged();
    } else {
      if (air_pressure_joint_ < joint_perching_pressure_) {
        stopSVSwitch();
        stopSVExhaust();
        calPressure(joint_perching_pressure_, 0);
        adjustPump();
        publishAirPwmMerged();
      } else {
        stopPump();
        publishAirPwmMerged();
      }
    failsafe();
    }
}

void AirPressureController::failsafe(){
    if (air_pressure_joint_ >= joint_limit_pressure_ || air_pressure_bottom_ >= bottom_limit_pressure_) {
        ROS_WARN("Air pressure exceeded maximum limits! Stopping pump.");
        stopAllPneumatics();
    }
}

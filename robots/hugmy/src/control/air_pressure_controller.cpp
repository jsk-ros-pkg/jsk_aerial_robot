#include <hugmy/control/air_pressure_controller.h>

AirPressureController::AirPressureController(ros::NodeHandle& nh) {
    sensor_joint_sub_ = nh.subscribe("/sensor", 1, &AirPressureController::sensorCb, this);
    sensor_bottom_sub_ = nh.subscribe("/sensor_1", 1, &AirPressureController::sensor1Cb, this);
    pwm_air_pub_ = nh.advertise<spinal::PwmTest>("/pwm_cmd/air", 1);
    pwm_pub_ = nh.advertise<spinal::PwmTest>("/quadrotor/pwm_test", 1);

    pwm_air_cmd_.motor_index.clear();
    pwm_air_cmd_.pwms.clear();
    last_published_pwm_ = pwm_air_cmd_;
}

// void AirPressureController::update(){
//     adjustAirPressure();
// }

void AirPressureController::sensorCb(const std_msgs::Int8::ConstPtr& msg) {
    air_pressure_joint_ = msg->data;
}
void AirPressureController::sensor1Cb(const std_msgs::Int8::ConstPtr& msg) {
    air_pressure_bottom_ = msg->data;
}
void AirPressureController::adjustPump(){
    setAirPwm({4,6}, {output_, output_});
}

void AirPressureController::maxWorkPumptoJoint(){
    publishAirPwm({4,6,5,7},{0.9f,0.9f,0.0f,0.0f});
}

void AirPressureController::startSVSwitch(){
    setAirPwm(5, 1.0f);
}

void AirPressureController::stopSVSwitch(){
    setAirPwm(5, 0.0f);
}

void AirPressureController::startSVExhaust(){
    setAirPwm(7, 1.0f);
}

void AirPressureController::stopSVExhaust(){
    setAirPwm(7, 0.0f);
}

void AirPressureController::stopAllSV(){
    setAirPwm({5,7},{0.0,0.0});
}

void AirPressureController::startAllSV(){
    setAirPwm({5,7},{1.0,1.0});
}
void AirPressureController::stopPump(){
    setAirPwm({4,6},{0.0,0.0});
}

void AirPressureController::stopAllPneumatics() {
    publishAirPwm({4,5,6,7},{0.0, 0.0, 0.0, 0.0});
}

void AirPressureController::initializePneumatics() {
    publishAirPwm({4,5,6,7},{0.0, 0.0, 0.0, 1.0}); // ポンプ停止、排気開始
    output_ = 0.0; // 出力をリセット
    ROS_INFO("All pneumatics stopped.");
}

void AirPressureController::setAirPwm(uint8_t index, float pwm_value) {
    pwm_state_[index] = pwm_value;
}

void AirPressureController::setAirPwm(const std::vector<uint8_t>& indices,
                const std::vector<float>& pwm_values) {
    for (size_t i = 0; i < indices.size(); ++i) {
      pwm_state_[indices[i]] = pwm_values[i];
    }
  }

void AirPressureController::publishAirPwmMerged() {
    std::vector<uint8_t> indices;
    std::vector<float> values;
    for (const auto& kv : pwm_state_) {
      indices.push_back(kv.first);
      values.push_back(kv.second);
    }
    publishAirPwm(indices, values);
  }

void AirPressureController::publishAirPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms) {
    pwm_air_cmd_.motor_index = indices;
    pwm_air_cmd_.pwms = pwms;
    pwm_air_pub_.publish(pwm_air_cmd_);
    pwm_pub_.publish(pwm_air_cmd_);
    last_published_pwm_ = pwm_air_cmd_;
    pwm_state_.clear();
}

void AirPressureController::calPressure(int target_pressure, int sensor_index) {
    int sensor_data = (sensor_index == 0) ? air_pressure_joint_ : air_pressure_bottom_;
    int thre = 1;
    int error = target_pressure - sensor_data;

    if (error > thre) {
        output_ = std::min(error*0.02 +0.3, 0.7);
    } else {
        output_ = std::max(error*0.02 + 0.32, 0.0);
    }
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

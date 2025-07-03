#include <hugmy/control/air_pressure_controller.h>

AirPressureController::AirPressureController(ros::NodeHandle& nh) {
    sensor_joint_sub_ = nh.subscribe("/sensor", 1, &AirPressureController::sensorCb, this);
    sensor_bottom_sub_ = nh.subscribe("/sensor_1", 1, &AirPressureController::sensor1Cb, this);
    pwm_air_pub_ = nh.advertise<spinal::PwmTest>("/pwm_cmd/air", 1);
    air_pressure_joint_ = 0;
    air_pressure_bottom_ = 0;
    output_ = 0.0;

    joint_max_pressure_ = 50;
    bottom_max_pressure_ = 20;
    joint_perching_pressure_ = 40;
    bottom_usual_pressure_ = 10;
}

void AirPressureController::update(){
    adjustAirPressure();
}
double AirPressureController::getOutput() const {
    return output_;
}
int AirPressureController::getAirPressureJoint() const {
    return air_pressure_joint_;
}
int AirPressureController::getAirPressureBottom() const {
    return air_pressure_bottom_;
}

spinal::PwmTest AirPressureController::getAirPwm() const {
    return last_published_pwm_;
}

void AirPressureController::stopPump(){
    publishAirPwm({4,6},{0.0,0.0});
}

void AirPressureController::stopAllPneumatics() {
    publishAirPwm({4,5,6,7},{0.0, 0.0, 0.0, 1.0}); // ポンプ停止、排気開始
    output_ = 0.0; // 出力をリセット
    ROS_INFO("All pneumatics stopped.");
}


void AirPressureController::sensorCb(const std_msgs::Int8::ConstPtr& msg) {
    air_pressure_joint_ = msg->data;
}
void AirPressureController::sensor1Cb(const std_msgs::Int8::ConstPtr& msg) {
    air_pressure_bottom_ = msg->data;
}

// void AirPressureController::adjustPump(){
//     publishAirPwm({4,6},{static_cast<float>(output_), static_cast<float>(output_)});
// }

// void AirPressureController::startSVSwitch(){
//     publishAirPwm({5},{1.0});
// }

// void AirPressureController::stopSVSwitch(){
//     publishAirPwm({5},{0.0});
// }

// void AirPressureController::startSVExhaust(){
//     publishAirPwm({7},{1.0});
// }

// void AirPressureController::stopSVExhaust(){
//     publishAirPwm({7},{0.0});
// }

void AirPressureController::publishAirPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms) {
    pwm_air_cmd_.motor_index = indices;
    pwm_air_cmd_.pwms = pwms;
    pwm_air_pub_.publish(pwm_air_cmd_);
    last_published_pwm_ = pwm_air_cmd_;
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

void AirPressureController::adjustAirPressure() {
    std::vector<uint8_t> indices = {4, 5, 6, 7};
    std::vector<float> pwms(4, 0.0);  // 初期値はすべて0.0
    if (air_pressure_bottom_ <= bottom_usual_pressure_) {
        pwms[0] = static_cast<float>(output_);  // 4番: ポンプ
        pwms[1] = 1.0f;                         // 5番: スイッチ
        pwms[2] = static_cast<float>(output_);  // 6番: ポンプ
        pwms[3] = 0.0f;                         // 7番: 排気
        calPressure(bottom_usual_pressure_, 1);
    } else {
        if (air_pressure_joint_ <= joint_perching_pressure_) {
            pwms[0] = static_cast<float>(output_);  // 4番: ポンプ
            pwms[1] = 0.0f;                         // 5番: スイッチ
            pwms[2] = static_cast<float>(output_);  // 6番: ポンプ
            pwms[3] = 0.0f;                         // 7番: 排気
            calPressure(joint_perching_pressure_, 0);
        } else {
            pwms[0] = 0.0f;
            pwms[1] = 0.0f;
            pwms[2] = 0.0f;
            pwms[3] = 0.0f;
        }
    }

    if (air_pressure_joint_ >= joint_max_pressure_ || air_pressure_bottom_ >= bottom_max_pressure_) {
        ROS_WARN("Air pressure exceeded maximum limits! Stopping pump.");
        pwms = {0.0f, 0.0f, 0.0f, 1.0f};
    }

    // 一括でpublish
    publishAirPwm(indices, pwms);
    ROS_INFO("Air Pressure Joint: %d, Bottom: %d, Output: %.2f", air_pressure_joint_, air_pressure_bottom_, output_);
}

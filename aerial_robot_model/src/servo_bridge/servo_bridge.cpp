// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <aerial_robot_model/servo_bridge.h>
#include <spinal/GetBoardInfo.h>
#include <spinal/ServoInfo.h>
#include <spinal/SetDirectServoConfig.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <set>
#include <stdexcept>
#include <vector>

namespace
{
constexpr double DIRECT_SERVO_CONFIG_CALL_INTERVAL = 0.02;
constexpr double DIRECT_SERVO_CONFIG_STARTUP_DELAY = 1.0;
constexpr double DIRECT_SERVO_CONFIG_TIMEOUT = 10.0;
constexpr double STARTUP_FEEDBACK_TIMEOUT = 2.0;
constexpr double STARTUP_TORQUE_CONFIRM_TIMEOUT = 2.0;

struct OptionalInt
{
  bool set = false;
  int32_t value = 0;
};

struct DirectServoStartupConfig
{
  uint8_t global_index = 0;
  uint8_t board_id = 0;
  uint8_t servo_index = 0;
  std::string name;
  uint8_t bus_type = spinal::ServoInfo::BUS_UNKNOWN;
  uint8_t mode = spinal::ServoInfo::MODE_UNKNOWN;
  bool mode_set = false;
  bool mit_config_set = false;
  int32_t mit_kp_integer = 0;
  int32_t mit_kp_decimal = 0;
  int32_t mit_kd_integer = 0;
  int32_t mit_kd_decimal = 0;
  int32_t mit_current_limit = 0;
  OptionalInt p_gain;
  OptionalInt i_gain;
  OptionalInt d_gain;
  OptionalInt profile_velocity;
  OptionalInt current_limit;
  OptionalInt send_data_flag;
  OptionalInt external_encoder_flag;
  OptionalInt joint_resolution;
  OptionalInt servo_resolution;
};

struct DirectServoBoardConfig
{
  int feedback_frequency = 50;
  int watchdog_ms = 100;
  bool mixed_set = false;
  int mixed = 0;
};

bool getInheritedValue(XmlRpc::XmlRpcValue& controller,
                       XmlRpc::XmlRpcValue& group,
                       const std::string& key,
                       XmlRpc::XmlRpcValue& value)
{
  if (controller.hasMember(key)) {
    value = controller[key];
    return true;
  }
  if (group.hasMember(key)) {
    value = group[key];
    return true;
  }
  return false;
}

double xmlRpcNumber(const XmlRpc::XmlRpcValue& value,
                    const std::string& name)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    return static_cast<double>(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    return static_cast<int>(value);
  throw std::runtime_error(name + " must be numeric");
}

int32_t xmlRpcInteger(const XmlRpc::XmlRpcValue& value,
                      const std::string& name)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    return static_cast<int>(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    return static_cast<bool>(value) ? 1 : 0;
  throw std::runtime_error(name + " must be an integer or boolean");
}

uint8_t parseModeName(std::string mode)
{
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) {return std::toupper(c);});
  if (mode == "DP") return spinal::ServoInfo::MODE_DP;
  if (mode == "DEP") return spinal::ServoInfo::MODE_DEP;
  if (mode == "DV") return spinal::ServoInfo::MODE_DV;
  if (mode == "DC") return spinal::ServoInfo::MODE_DC;
  if (mode == "MIT") return spinal::ServoInfo::MODE_MIT;
  if (mode == "CBP") return spinal::ServoInfo::MODE_CBP;
  throw std::runtime_error("mode must be DP, DEP, DV, DC, MIT or CbP");
}

bool parseDirectServoMode(XmlRpc::XmlRpcValue& controller,
                          XmlRpc::XmlRpcValue& group,
                          uint8_t& mode)
{
  XmlRpc::XmlRpcValue value;
  if (!getInheritedValue(controller, group, "mode", value))
    return false;
  if (value.getType() != XmlRpc::XmlRpcValue::TypeString)
    throw std::runtime_error("mode must be a string");
  mode = parseModeName(static_cast<std::string>(value));
  return true;
}

uint8_t parseBusType(XmlRpc::XmlRpcValue& controller,
                     XmlRpc::XmlRpcValue& group)
{
  XmlRpc::XmlRpcValue value;
  if (!getInheritedValue(controller, group, "bus_type", value))
    return spinal::ServoInfo::BUS_UNKNOWN;
  if (value.getType() != XmlRpc::XmlRpcValue::TypeString)
    throw std::runtime_error("bus_type must be R or T");
  std::string type = static_cast<std::string>(value);
  std::transform(type.begin(), type.end(), type.begin(),
                 [](unsigned char c) {return std::toupper(c);});
  if (type == "R" || type == "RS485")
    return spinal::ServoInfo::BUS_RS485;
  if (type == "T" || type == "TTL")
    return spinal::ServoInfo::BUS_TTL;
  throw std::runtime_error("bus_type must be R/RS485 or T/TTL");
}

OptionalInt inheritedOptionalInt(XmlRpc::XmlRpcValue& controller,
                                 XmlRpc::XmlRpcValue& group,
                                 const std::string& key,
                                 int32_t minimum, int32_t maximum)
{
  OptionalInt result;
  XmlRpc::XmlRpcValue value;
  if (!getInheritedValue(controller, group, key, value)) return result;
  result.value = xmlRpcInteger(value, key);
  if (result.value < minimum || result.value > maximum)
    throw std::runtime_error(key + " is outside the supported range");
  result.set = true;
  return result;
}

void encodeMitGain(double raw_gain, const std::string& name,
                   int32_t& integer_part, int32_t& decimal_part)
{
  if (!std::isfinite(raw_gain) || raw_gain < 0.0)
    throw std::runtime_error(name + " produces an invalid raw gain");
  const int64_t scale = spinal::ServoInfo::GAIN_DECIMAL_SCALE;
  const int64_t scaled_gain = std::llround(raw_gain * scale);
  const int64_t maximum =
    static_cast<int64_t>(INT16_MAX) * scale + scale - 1;
  if (scaled_gain > maximum)
    throw std::runtime_error(name + " exceeds the MCU MIT gain range");
  integer_part = static_cast<int32_t>(scaled_gain / scale);
  decimal_part = static_cast<int32_t>(scaled_gain % scale);
}

uint8_t expectedDynamixelMode(uint8_t mode)
{
  switch (mode) {
  case spinal::ServoInfo::MODE_DP:
    return spinal::ServoInfo::SERVO_MODE_POSITION;
  case spinal::ServoInfo::MODE_DEP:
    return spinal::ServoInfo::SERVO_MODE_EXTENDED_POSITION;
  case spinal::ServoInfo::MODE_DV:
    return spinal::ServoInfo::SERVO_MODE_VELOCITY;
  case spinal::ServoInfo::MODE_DC:
  case spinal::ServoInfo::MODE_MIT:
    return spinal::ServoInfo::SERVO_MODE_CURRENT;
  case spinal::ServoInfo::MODE_CBP:
    return spinal::ServoInfo::SERVO_MODE_CURRENT_BASED_POSITION;
  default:
    return spinal::ServoInfo::SERVO_MODE_UNKNOWN;
  }
}

DirectServoStartupConfig makeDirectServoConfig(
  int index, const std::string& name,
  XmlRpc::XmlRpcValue& controller, XmlRpc::XmlRpcValue& group,
  double angle_scale, double velocity_scale,
  double torque_scale)
{
  if (index < 0 || index > UINT8_MAX)
    throw std::runtime_error(name + ": id must fit in uint8");

  DirectServoStartupConfig config;
  config.global_index = static_cast<uint8_t>(index);
  config.name = name;
  config.bus_type = parseBusType(controller, group);
  config.mode_set = parseDirectServoMode(controller, group, config.mode);
  config.p_gain = inheritedOptionalInt(
    controller, group, "p_gain", 0, UINT16_MAX);
  config.i_gain = inheritedOptionalInt(
    controller, group, "i_gain", 0, UINT16_MAX);
  config.d_gain = inheritedOptionalInt(
    controller, group, "d_gain", 0, UINT16_MAX);
  config.profile_velocity = inheritedOptionalInt(
    controller, group, "profile_velocity", 0, UINT16_MAX);
  config.current_limit = inheritedOptionalInt(
    controller, group, "current_limit", 0, UINT16_MAX);
  config.send_data_flag = inheritedOptionalInt(
    controller, group, "send_data_flag", 0, 1);
  config.external_encoder_flag = inheritedOptionalInt(
    controller, group, "external_encoder_flag", 0, 1);
  config.joint_resolution = inheritedOptionalInt(
    controller, group, "joint_resolution", 1, UINT16_MAX - 1);
  config.servo_resolution = inheritedOptionalInt(
    controller, group, "servo_resolution", 1, UINT16_MAX - 1);

  XmlRpc::XmlRpcValue kp_value;
  XmlRpc::XmlRpcValue kd_value;
  XmlRpc::XmlRpcValue effort_limit_value;
  const bool kp_set =
    getInheritedValue(controller, group, "mit_kp_si", kp_value);
  const bool kd_set =
    getInheritedValue(controller, group, "mit_kd_si", kd_value);
  const bool effort_limit_set = getInheritedValue(
    controller, group, "mit_effort_limit_si", effort_limit_value);
  if (kp_set || kd_set || effort_limit_set) {
    if (!kp_set || !kd_set || !effort_limit_set) {
      throw std::runtime_error(
        name + ": mit_kp_si, mit_kd_si and mit_effort_limit_si "
        "must be configured together");
    }
    if (!(angle_scale > 0.0) || !(torque_scale > 0.0))
      throw std::runtime_error(name + ": mit_kp_si requires positive angle_scale and torque_scale");
    encodeMitGain(
      xmlRpcNumber(kp_value, name + ".mit_kp_si") *
        angle_scale / torque_scale,
      name + ".mit_kp_si", config.mit_kp_integer, config.mit_kp_decimal);
    if (!(velocity_scale > 0.0) || !(torque_scale > 0.0))
      throw std::runtime_error(name + ": mit_kd_si requires positive velocity_scale and torque_scale");
    encodeMitGain(
      xmlRpcNumber(kd_value, name + ".mit_kd_si") *
        velocity_scale / torque_scale,
      name + ".mit_kd_si", config.mit_kd_integer, config.mit_kd_decimal);
    const double effort_limit_si = xmlRpcNumber(
      effort_limit_value, name + ".mit_effort_limit_si");
    if (!std::isfinite(effort_limit_si) || effort_limit_si < 0.0)
      throw std::runtime_error(
        name + ": mit_effort_limit_si must be finite and non-negative");
    const double raw_limit = std::floor(effort_limit_si / torque_scale);
    if (raw_limit > UINT16_MAX)
      throw std::runtime_error(
        name + ": mit_effort_limit_si exceeds the MCU current range");
    config.mit_current_limit = static_cast<int32_t>(raw_limit);
    config.mit_config_set = true;
  }
  return config;
}

const spinal::ServoInfo* findServoInfo(
  const spinal::BoardInfo& board, uint8_t servo_index)
{
  if (servo_index >= board.servos.size()) return nullptr;
  return &board.servos[servo_index];
}

bool getServoBoards(ros::ServiceClient& client,
                    std::vector<spinal::BoardInfo>& boards)
{
  spinal::GetBoardInfo srv;
  const bool called = client.call(srv);
  ros::WallDuration(DIRECT_SERVO_CONFIG_CALL_INTERVAL).sleep();
  if (!called) return false;
  boards = srv.response.boards;
  return !boards.empty();
}

const spinal::ServoInfo* resolveGlobalServoAddress(
  const std::vector<spinal::BoardInfo>& boards, uint8_t global_index,
  uint8_t& board_id, uint8_t& servo_index)
{
  size_t remaining_index = global_index;
  for (const spinal::BoardInfo& board : boards) {
    if (remaining_index < board.servos.size()) {
      board_id = board.slave_id;
      servo_index = static_cast<uint8_t>(remaining_index);
      return &board.servos[remaining_index];
    }
    remaining_index -= board.servos.size();
  }
  return nullptr;
}

const spinal::BoardInfo* findBoardInfo(
  const std::vector<spinal::BoardInfo>& boards, uint8_t board_id)
{
  for (const spinal::BoardInfo& board : boards) {
    if (board.slave_id == board_id) return &board;
  }
  return nullptr;
}

bool configureDirectServos(
  ros::NodeHandle& nh,
  const DirectServoBoardConfig& board_config,
  const std::vector<DirectServoStartupConfig>& requested_configs,
  double timeout, std::set<uint8_t>& feedback_servo_ids)
{
  feedback_servo_ids.clear();
  std::vector<DirectServoStartupConfig> configs = requested_configs;
  ros::ServiceClient config_client =
    nh.serviceClient<spinal::SetDirectServoConfig>(
      "direct_servo_config");
  ros::ServiceClient board_info_client =
    nh.serviceClient<spinal::GetBoardInfo>("get_board_info");
  if (!config_client.waitForExistence(ros::Duration(timeout)) ||
      !board_info_client.waitForExistence(ros::Duration(timeout))) {
    ROS_ERROR("Timed out after %.1f s waiting for DirectServo configuration services",
              timeout);
    return false;
  }

  auto write_config = [&config_client](
    uint8_t board_id, uint8_t servo_index, uint8_t command,
    const std::vector<int32_t>& data,
    const std::string& description) {
    spinal::SetDirectServoConfig srv;
    srv.request.board_id = board_id;
    srv.request.servo_index = servo_index;
    srv.request.command = command;
    srv.request.data = data;
    const bool called = config_client.call(srv);
    ros::WallDuration(DIRECT_SERVO_CONFIG_CALL_INTERVAL).sleep();
    if (!called || !srv.response.success) {
      ROS_ERROR("DirectServo board %u rejected %s",
                static_cast<unsigned int>(board_id), description.c_str());
      return false;
    }
    return true;
  };

  std::vector<spinal::BoardInfo> boards;
  if (!getServoBoards(board_info_client, boards)) {
    ROS_ERROR("No servo board was present in get_board_info");
    return false;
  }

  auto write_servo_config = [&write_config](
    const DirectServoStartupConfig& config, uint8_t command,
    const std::vector<int32_t>& data, const std::string& description) {
    return write_config(config.board_id, config.servo_index, command,
                        data, description);
  };
  for (auto& config : configs) {
    if (resolveGlobalServoAddress(
          boards, config.global_index, config.board_id,
          config.servo_index) == nullptr) {
      ROS_ERROR("%s maps to missing global servo id %u",
                config.name.c_str(),
                static_cast<unsigned int>(config.global_index));
      return false;
    }
  }

  for (const auto& detected_board : boards) {
    if (detected_board.servos.empty()) continue;
    if (detected_board.servo_feedback_frequency !=
          board_config.feedback_frequency &&
        !write_config(
          detected_board.slave_id,
          0,
          spinal::SetDirectServoConfig::Request::SET_SERVO_FEEDBACK_FREQUENCY,
          {board_config.feedback_frequency}, "feedback frequency")) {
      return false;
    }
    const int current_watchdog_ms = static_cast<int>(
      std::lround(
        detected_board.servo_target_watchdog_timeout * 1000.0));
    if (current_watchdog_ms != board_config.watchdog_ms &&
        !write_config(
          detected_board.slave_id,
          0,
          spinal::SetDirectServoConfig::Request::SET_SERVO_TARGET_WATCHDOG_TIMEOUT,
          {board_config.watchdog_ms}, "target watchdog")) {
      return false;
    }
  }

  for (auto& config : configs) {
    const spinal::BoardInfo* config_board =
      findBoardInfo(boards, config.board_id);
    const spinal::ServoInfo* servo = config_board == nullptr
      ? nullptr : findServoInfo(*config_board, config.servo_index);
    if (servo == nullptr) {
      ROS_ERROR("YAML global servo id %u was not detected",
                static_cast<unsigned int>(config.global_index));
      return false;
    }
    if (config.bus_type == spinal::ServoInfo::BUS_UNKNOWN) {
      if (!servo->configured ||
          (servo->bus_type != spinal::ServoInfo::BUS_RS485 &&
           servo->bus_type != spinal::ServoInfo::BUS_TTL)) {
        ROS_ERROR("%s has no bus_type in YAML or MCU Flash",
                  config.name.c_str());
        return false;
      }
      config.bus_type = servo->bus_type;
    }
    if ((!servo->configured || servo->bus_type != config.bus_type) &&
        !write_servo_config(config,
          spinal::SetDirectServoConfig::Request::SET_SERVO_BUS_TYPE,
          {config.bus_type},
          config.name + " bus_type")) {
      return false;
    }
  }

  if (board_config.mixed_set) {
    for (const auto& detected_board : boards) {
      if (detected_board.servos.empty()) continue;
      if (detected_board.dynamixel_ttl_rs485_mixed != board_config.mixed &&
          !write_config(
            detected_board.slave_id,
            0,
            spinal::SetDirectServoConfig::Request::SET_DYNAMIXEL_TTL_RS485_MIXED,
            {board_config.mixed}, "R/T grouping enable")) {
        return false;
      }
    }
  }

  if (!getServoBoards(board_info_client, boards)) {
    ROS_ERROR("Failed to read servo configuration after bus setup");
    return false;
  }

  for (const auto& config : configs) {
    if (!config.mode_set) continue;
    const spinal::BoardInfo* config_board =
      findBoardInfo(boards, config.board_id);
    const spinal::ServoInfo* servo = config_board == nullptr
      ? nullptr : findServoInfo(*config_board, config.servo_index);
    if (servo == nullptr) {
      ROS_ERROR("YAML global servo id %u was not detected",
                static_cast<unsigned int>(config.global_index));
      return false;
    }
    const uint8_t expected_mode = expectedDynamixelMode(config.mode);
    const bool mode_matches = servo->mode == config.mode &&
                              servo->servo_mode == expected_mode;
    if (!mode_matches &&
        !write_servo_config(config,
          spinal::SetDirectServoConfig::Request::SET_SERVO_CONTROL_MODE,
          {config.mode},
          config.name + " operating mode")) {
      return false;
    }
  }

  if (!getServoBoards(board_info_client, boards)) {
    ROS_ERROR("Failed to read servo configuration after mode setup");
    return false;
  }

  for (const auto& config : configs) {
    const spinal::BoardInfo* config_board =
      findBoardInfo(boards, config.board_id);
    const spinal::ServoInfo* servo = config_board == nullptr
      ? nullptr : findServoInfo(*config_board, config.servo_index);
    if (servo == nullptr) return false;

    if (config.p_gain.set || config.i_gain.set ||
        config.d_gain.set) {
      const int32_t p = config.p_gain.set
        ? config.p_gain.value : servo->p_gain;
      const int32_t i = config.i_gain.set
        ? config.i_gain.value : servo->i_gain;
      const int32_t d = config.d_gain.set
        ? config.d_gain.value : servo->d_gain;
      if ((p != servo->p_gain || i != servo->i_gain ||
           d != servo->d_gain) &&
          !write_servo_config(config,
            spinal::SetDirectServoConfig::Request::SET_SERVO_PID_GAIN,
            {p, i, d},
            config.name + " position gains")) {
        return false;
      }
    }
    if (config.profile_velocity.set &&
        config.profile_velocity.value != servo->profile_velocity &&
        !write_servo_config(config,
          spinal::SetDirectServoConfig::Request::SET_SERVO_PROFILE_VEL,
          {config.profile_velocity.value},
          config.name + " profile velocity")) {
      return false;
    }
    if (config.current_limit.set &&
        config.current_limit.value != servo->current_limit &&
        !write_servo_config(config,
          spinal::SetDirectServoConfig::Request::SET_SERVO_CURRENT_LIMIT,
          {config.current_limit.value},
          config.name + " current limit")) {
      return false;
    }
    if (config.send_data_flag.set &&
        config.send_data_flag.value != servo->send_data_flag &&
        !write_servo_config(config,
          spinal::SetDirectServoConfig::Request::SET_SERVO_SEND_DATA_FLAG,
          {config.send_data_flag.value},
          config.name + " send_data_flag")) {
      return false;
    }
    if (config.external_encoder_flag.set &&
        config.external_encoder_flag.value !=
          servo->external_encoder_flag &&
        !write_servo_config(config,
          spinal::SetDirectServoConfig::Request::SET_SERVO_EXTERNAL_ENCODER_FLAG,
          {config.external_encoder_flag.value},
          config.name + " external_encoder_flag")) {
      return false;
    }
    if (config.joint_resolution.set ||
        config.servo_resolution.set) {
      const int32_t current_joint_resolution =
        static_cast<uint16_t>(servo->joint_resolution);
      const int32_t current_servo_resolution =
        static_cast<uint16_t>(servo->servo_resolution);
      const int32_t joint_resolution =
        config.joint_resolution.set
        ? config.joint_resolution.value : current_joint_resolution;
      const int32_t servo_resolution =
        config.servo_resolution.set
        ? config.servo_resolution.value : current_servo_resolution;
      if ((joint_resolution != current_joint_resolution ||
           servo_resolution != current_servo_resolution) &&
          !write_servo_config(config,
            spinal::SetDirectServoConfig::Request::SET_SERVO_RESOLUTION_RATIO,
            {joint_resolution, servo_resolution},
            config.name + " resolution ratio")) {
        return false;
      }
    }
    if (config.mit_config_set &&
        (config.mit_kp_integer != servo->mit_kp_integer ||
         config.mit_kp_decimal != servo->mit_kp_decimal ||
         config.mit_kd_integer != servo->mit_kd_integer ||
         config.mit_kd_decimal != servo->mit_kd_decimal ||
         config.mit_current_limit != servo->mit_current_limit) &&
        !write_servo_config(config,
          spinal::SetDirectServoConfig::Request::SET_SERVO_MIT_PD,
          {config.mit_kp_integer, config.mit_kp_decimal,
           config.mit_kd_integer, config.mit_kd_decimal,
           config.mit_current_limit},
          config.name + " MIT configuration")) {
      return false;
    }
  }

  if (!getServoBoards(board_info_client, boards)) {
    ROS_ERROR("Failed to read final servo configuration");
    return false;
  }
  for (const auto& detected_board : boards) {
    if (detected_board.servos.empty()) continue;
    const int final_watchdog_ms = static_cast<int>(
      std::lround(
        detected_board.servo_target_watchdog_timeout * 1000.0));
    if (detected_board.servo_feedback_frequency !=
          board_config.feedback_frequency ||
        final_watchdog_ms != board_config.watchdog_ms ||
        (board_config.mixed_set &&
         detected_board.dynamixel_ttl_rs485_mixed != board_config.mixed)) {
      ROS_ERROR("Final servo board %u readback failed",
                static_cast<unsigned int>(detected_board.slave_id));
      return false;
    }
  }
  for (const auto& config : configs) {
    const spinal::BoardInfo* config_board =
      findBoardInfo(boards, config.board_id);
    const spinal::ServoInfo* servo = config_board == nullptr
      ? nullptr : findServoInfo(*config_board, config.servo_index);
    if (servo == nullptr || !servo->configured ||
        servo->bus_type != config.bus_type ||
        (config.mode_set &&
         (servo->mode != config.mode ||
          servo->servo_mode != expectedDynamixelMode(config.mode)))) {
      ROS_ERROR("Final DirectServo readback failed for %s",
                config.name.c_str());
      return false;
    }
    const bool optional_mismatch =
      (config.p_gain.set &&
       config.p_gain.value != servo->p_gain) ||
      (config.i_gain.set &&
       config.i_gain.value != servo->i_gain) ||
      (config.d_gain.set &&
       config.d_gain.value != servo->d_gain) ||
      (config.profile_velocity.set &&
       config.profile_velocity.value != servo->profile_velocity) ||
      (config.current_limit.set &&
       config.current_limit.value != servo->current_limit) ||
      (config.send_data_flag.set &&
       config.send_data_flag.value != servo->send_data_flag) ||
      (config.external_encoder_flag.set &&
       config.external_encoder_flag.value !=
         servo->external_encoder_flag) ||
      (config.joint_resolution.set &&
       config.joint_resolution.value !=
         static_cast<uint16_t>(servo->joint_resolution)) ||
      (config.servo_resolution.set &&
       config.servo_resolution.value !=
         static_cast<uint16_t>(servo->servo_resolution)) ||
      (config.mit_config_set &&
       (config.mit_kp_integer != servo->mit_kp_integer ||
        config.mit_kp_decimal != servo->mit_kp_decimal ||
        config.mit_kd_integer != servo->mit_kd_integer ||
        config.mit_kd_decimal != servo->mit_kd_decimal ||
        config.mit_current_limit != servo->mit_current_limit));
    if (optional_mismatch) {
      ROS_ERROR("Final optional-parameter readback failed for %s",
                config.name.c_str());
      return false;
    }
    if (servo->send_data_flag != 0)
      feedback_servo_ids.insert(config.global_index);
  }
  return true;
}
}

ServoBridge::ServoBridge(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp),
  auto_configure_direct_servo_(false),
  startup_torque_params_valid_(true),
  startup_torque_state_(StartupTorqueState::DISABLED)
{
  nh_.param("/use_sim_time", simulation_mode_, false);
  nhp_.param("use_mujoco", use_mujoco_, false);
  if(use_mujoco_)
    {
      ROS_WARN("use mujoco simulator");
      simulation_mode_ = false;
    }

  if(simulation_mode_)
    {
      if(!ros::service::waitForService(nh_.getNamespace() + std::string("/controller_manager/load_controller")))
        {
          ROS_ERROR("can't find the controller manager: %s", (nh_.getNamespace() + std::string("/controller_manager/load_controller")).c_str());
          return;
        }
    }

  /* get robot URDF model via robot_description*/
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
    ROS_ERROR("Failed to extract urdf model from rosparam");

  /* common param and topics (between servo_bridge and spinal_ros_bridge) */
  ros::NodeHandle nh_controller(nh_, "servo_controller");
  std::string state_sub_topic, pos_pub_topic;
  std::string servo_enable_pub_topic, servo_torque_state_sub_topic;
  nh_controller.param("state_sub_topic", state_sub_topic, std::string("servo/states"));
  nh_controller.param("pos_pub_topic", pos_pub_topic, std::string("servo/target_states"));
  nh_controller.param("servo_enable_pub_topic", servo_enable_pub_topic, std::string("servo/torque_enable"));
  nh_controller.param("servo_torque_state_sub_topic",
                      servo_torque_state_sub_topic,
                      std::string("servo/torque_states"));
  nh_controller.param("auto_configure_direct_servo",
                      auto_configure_direct_servo_, false);
  const bool configure_direct_servo =
    auto_configure_direct_servo_ && !simulation_mode_ && !use_mujoco_;
  bool direct_servo_params_valid = true;

  DirectServoBoardConfig direct_board_config;
  if (!nh_controller.getParam(
        "ServoFeedBackFrequency",
        direct_board_config.feedback_frequency)) {
    nh_controller.param("servo_feedback_frequency",
                        direct_board_config.feedback_frequency, 50);
  }
  double watchdog_seconds = 0.1;
  if (!nh_controller.getParam("ServoTargetWatchdog",
                              watchdog_seconds)) {
    nh_controller.param("servo_target_watchdog",
                        watchdog_seconds, 0.1);
  }
  if (!std::isfinite(watchdog_seconds) ||
      watchdog_seconds < 0.0 || watchdog_seconds > 60.0) {
    ROS_FATAL("ServoTargetWatchdog must be within 0..60 seconds");
    direct_servo_params_valid = false;
  } else {
    direct_board_config.watchdog_ms =
      static_cast<int>(std::lround(watchdog_seconds * 1000.0));
  }
  bool mixed_value = false;
  if (nh_controller.getParam("dynamixel_ttl_rs485_mixed",
                             mixed_value)) {
    direct_board_config.mixed_set = true;
    direct_board_config.mixed = mixed_value ? 1 : 0;
  }
  if (direct_board_config.feedback_frequency <= 0 ||
      direct_board_config.feedback_frequency > 200) {
    ROS_FATAL("ServoFeedBackFrequency must be within 1..200 Hz");
    direct_servo_params_valid = false;
  }
  /* common subsriber: get servo states (i.e. joint angles) from real machine (spinal_ros_bridge) */
  servo_states_subs_.insert(make_pair("common", nh_.subscribe<spinal::ServoStates>(state_sub_topic, 10, boost::bind(&ServoBridge::servoStatesCallback, this, _1, "common"))));
  servo_torque_subs_.insert(make_pair(
    "common", nh_.subscribe<spinal::ServoTorqueStates>(
      servo_torque_state_sub_topic, 1,
      &ServoBridge::servoTorqueStatesCallback, this)));
  /* common publisher: target servo position to real machine (spinal_ros_bridge) */
  servo_target_pos_pubs_.insert(make_pair(
    "common", nh_.advertise<spinal::ServoControlCmd>(pos_pub_topic, 1)));
  mujoco_control_input_pub_ = nh_.advertise<sensor_msgs::JointState>("mujoco/ctrl_input", 1);
  /* common publisher: servo on/off flag to real machine (spinal_ros_bridge) */
  servo_enable_pubs_.insert(make_pair("common", nh_.advertise<spinal::ServoTorqueCmd>(servo_enable_pub_topic, 1)));
  /* common publisher: joint profiles */
  joint_profile_pub_ = nh_.advertise<spinal::JointProfiles>("joint_profiles",1);
  /* subscriber: uav info */
  uav_info_sub_ = nh_.subscribe<spinal::UavInfo>("uav_info", 1, &ServoBridge::uavInfoCallback, this);


  /* get additional config for servos from ros parameters */
  XmlRpc::XmlRpcValue all_servos_params;
  nh_.getParam("servo_controller", all_servos_params);
  std::vector<DirectServoStartupConfig> direct_servo_configs;
  std::set<uint8_t> direct_servo_global_indices;
  spinal::JointProfiles joint_profiles_msg;
  for(auto servo_group_params: all_servos_params)
    {
      if (servo_group_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        continue;

      std::string group_name = servo_group_params.first;

      if(servo_group_params.second.hasMember("startup_torque_enable"))
        {
          XmlRpc::XmlRpcValue& startup_enable =
            servo_group_params.second["startup_torque_enable"];
          if(startup_enable.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
            {
              ROS_ERROR("servo_controller/%s/startup_torque_enable must be a boolean",
                        group_name.c_str());
              startup_torque_params_valid_ = false;
            }
          else
            {
              startup_torque_enable_[group_name] =
                static_cast<bool>(startup_enable);
            }
        }

      /* check whether has feedback state from real machine */
      no_real_state_flags_.insert(make_pair(group_name, false));
      if(servo_group_params.second.hasMember("no_real_state"))
        no_real_state_flags_.at(group_name) = servo_group_params.second["no_real_state"];

      /* pub, sub, and srv */
      /* mandatory subscriber: target servo command (e.g., position, torque) from controller */
      servo_ctrl_subs_.insert(make_pair(group_name, nh_.subscribe<sensor_msgs::JointState>(group_name + string("_ctrl"), 10, boost::bind(&ServoBridge::servoCtrlCallback, this, _1, group_name))));
      /* mandatory subscriber: only target servo troque command from controller */
      servo_torque_ctrl_subs_.insert(make_pair(group_name, nh_.subscribe<sensor_msgs::JointState>(group_name + string("_torque_ctrl"), 10, boost::bind(&ServoBridge::servoTorqueCtrlCallback, this, _1, group_name))));
      /* mandatory service: get torque enalbe/disable flag  */
      servo_enable_srvs_.insert(make_pair(group_name, nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(servo_group_params.first + string("/torque_enable"), boost::bind(&ServoBridge::servoEnableCallback, this, _1, _2, servo_group_params.first))));

      if(servo_group_params.second.hasMember("state_sub_topic"))
        {
          /* option: servo states (i.e. joint angles) from real machine (spinal_ros_bridge) */
          servo_states_subs_.insert(make_pair(group_name, nh_.subscribe<spinal::ServoStates>((string)servo_group_params.second["state_sub_topic"], 10, boost::bind(&ServoBridge::servoStatesCallback, this, _1, servo_group_params.first))));
        }

      if(servo_group_params.second.hasMember("pos_pub_topic"))
        {
          /* option: publish target servo state to real machine (spinal_ros_bridge) */
          servo_target_pos_pubs_.insert(make_pair(
            group_name, nh_.advertise<spinal::ServoControlCmd>(
              servo_group_params.second["pos_pub_topic"], 1)));
        }

      if(servo_group_params.second.hasMember("servo_enable_pub_topic"))
        {
          /* option: torque on/off */
          servo_enable_pubs_.insert(make_pair(group_name, nh_.advertise<spinal::ServoTorqueCmd>((string)servo_group_params.second["servo_enable_pub_topic"], 1)));
        }

      /* servo handler */
      ServoGroupHandler servo_group_handler;
      for(auto servo_params : servo_group_params.second)
        {
          if(servo_params.first.find("controller") != string::npos)
            {
              /* get parameters from urdf file */
              double upper_limit = urdf_model.getJoint(servo_params.second["name"])->limits->upper;
              double lower_limit = urdf_model.getJoint(servo_params.second["name"])->limits->lower;

              /* get parameters from rosparam */
              int servo_id = servo_params.second["id"];
              if (startup_torque_enable_.count(group_name) != 0) {
                if (servo_id < 0 || servo_id > UINT8_MAX) {
                  ROS_ERROR("Servo %s id must fit in uint8",
                            string(servo_params.second["name"]).c_str());
                  startup_torque_params_valid_ = false;
                } else {
                  const uint8_t global_id = static_cast<uint8_t>(servo_id);
                  if (!startup_all_servo_ids_.insert(global_id).second) {
                    ROS_ERROR("Duplicate global servo id %d", servo_id);
                    startup_torque_params_valid_ = false;
                  }
                  bool feedback_enabled =
                    !no_real_state_flags_.at(group_name);
                  XmlRpc::XmlRpcValue send_data_flag;
                  if (feedback_enabled && getInheritedValue(
                        servo_params.second, servo_group_params.second,
                        "send_data_flag", send_data_flag)) {
                    try {
                      feedback_enabled = xmlRpcInteger(
                        send_data_flag, "send_data_flag") != 0;
                    } catch (const std::exception& error) {
                      ROS_ERROR("Invalid send_data_flag for %s: %s",
                                string(servo_params.second["name"]).c_str(),
                                error.what());
                      startup_torque_params_valid_ = false;
                      feedback_enabled = false;
                    }
                  }
                  if (feedback_enabled)
                    startup_real_state_servo_ids_.insert(global_id);
                }
              }
              int angle_sgn = servo_params.second.hasMember("angle_sgn")? 
                servo_params.second["angle_sgn"]:servo_group_params.second["angle_sgn"];
              int zero_point_offset = servo_params.second.hasMember("zero_point_offset")?
                servo_params.second["zero_point_offset"]:servo_group_params.second["zero_point_offset"];
              double angle_scale = servo_params.second.hasMember("angle_scale")?
                servo_params.second["angle_scale"]:servo_group_params.second["angle_scale"];
              double velocity_scale = 0.0;
              if(servo_params.second.hasMember("velocity_scale"))
                velocity_scale = servo_params.second["velocity_scale"];
              else if(servo_group_params.second.hasMember("velocity_scale"))
                velocity_scale = servo_group_params.second["velocity_scale"];
              else
                ROS_WARN("%s: velocity_scale is not set. joint_states.velocity will be published as 0 until configured.",
                         string(servo_params.second["name"]).c_str());

              double torque_scale = servo_params.second.hasMember("torque_scale")?
                servo_params.second["torque_scale"]:(servo_group_params.second.hasMember("torque_scale")?servo_group_params.second["torque_scale"]: XmlRpc::XmlRpcValue(1.0));

              if (configure_direct_servo) {
                try {
                  DirectServoStartupConfig config =
                    makeDirectServoConfig(
                      servo_id, string(servo_params.second["name"]),
                      servo_params.second, servo_group_params.second,
                      angle_scale, velocity_scale,
                      torque_scale);
                  if (!direct_servo_global_indices.insert(
                        config.global_index).second) {
                    throw std::runtime_error(
                      "duplicate global servo id in DirectServo YAML");
                  }
                  direct_servo_configs.push_back(config);
                } catch (const std::exception& error) {
                  ROS_FATAL("Invalid direct servo configuration: %s",
                            error.what());
                  direct_servo_params_valid = false;
                }
              }

              /* for low pass filtering */
              bool filter_flag = servo_group_params.second.hasMember("filter_flag")?
                servo_group_params.second["filter_flag"]:(servo_params.second.hasMember("filter_flag")?servo_params.second["filter_flag"]: XmlRpc::XmlRpcValue(false));
              double sample_freq = servo_group_params.second.hasMember("sample_freq")?
                servo_group_params.second["sample_freq"]:(servo_params.second.hasMember("sample_freq")?servo_params.second["sample_freq"]: XmlRpc::XmlRpcValue(0.0));
              double cutoff_freq = servo_group_params.second.hasMember("cutoff_freq")?
                servo_group_params.second["cutoff_freq"]:(servo_params.second.hasMember("cutoff_freq")?servo_params.second["cutoff_freq"]: XmlRpc::XmlRpcValue(0.0));

              servo_group_handler.push_back(SingleServoHandlePtr(new SingleServoHandle(servo_params.second["name"], servo_params.second["id"], angle_sgn, zero_point_offset, angle_scale, velocity_scale, upper_limit, lower_limit, torque_scale, !no_real_state_flags_.at(group_name), filter_flag, sample_freq, cutoff_freq)));

              /* rosparam and load controller for gazebo */
              if(simulation_mode_)
                {
                  if(!servo_group_params.second.hasMember("simulation") &&
                     !servo_params.second.hasMember("simulation"))
                    {
                      ROS_ERROR("please set gazebo servo parameters for %s, using sub namespace 'simulation:'", string(servo_params.second["name"]).c_str());

                      continue;
                    }

                  vector<string> parameter_names = {"type", "pid", "init_value"};
                  for(auto parameter_name : parameter_names)
                    {
                      if(!servo_params.second.hasMember("simulation") ||
                         (servo_params.second.hasMember("simulation") &&
                          !servo_params.second["simulation"].hasMember(parameter_name)))
                        {
                          if(!servo_group_params.second["simulation"].hasMember(parameter_name))
                            {
                              ROS_ERROR("can not find '%s' gazebo paramter for servo %s", parameter_name.c_str(),  string(servo_params.second["name"]).c_str());
                              return;
                            }
                          nh_.setParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/") + parameter_name, servo_group_params.second["simulation"][parameter_name]);
                        }
                    }
                  nh_.setParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/joint"), servo_params.second["name"]);


                  /* load the controller */
                  ros::ServiceClient controller_loader = nh_.serviceClient<controller_manager_msgs::LoadController>(nh_.getNamespace() + std::string("/controller_manager/load_controller"));
                  controller_manager_msgs::LoadController load_srv;
                  load_srv.request.name = nh_.getNamespace() + string("/servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation");

                  if (controller_loader.call(load_srv))
                    ROS_INFO("load gazebo controller for %s", string(servo_params.second["name"]).c_str());
                  else
                    ROS_ERROR("Failed to call service %s", controller_loader.getService().c_str());

                  /* start the controller */
                  ros::ServiceClient controller_starter = nh_.serviceClient<controller_manager_msgs::SwitchController>(nh_.getNamespace() + std::string("/controller_manager/switch_controller"));
                  controller_manager_msgs::SwitchController switch_srv;
                  switch_srv.request.start_controllers.push_back(load_srv.request.name);
                  switch_srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

                  if (controller_starter.call(switch_srv))
                    ROS_INFO("start servo controller for %s", string(servo_params.second["name"]).c_str());
                  else
                    ROS_ERROR("Failed to call service %s", controller_starter.getService().c_str());

                  /* init the servo command publisher to the controller */
                  servo_target_pos_sim_pubs_[servo_group_params.first].push_back(nh_.advertise<std_msgs::Float64>(load_srv.request.name + string("/command"), 1));
                  // wait for the publisher initialization
                  while(servo_target_pos_sim_pubs_[servo_group_params.first].back().getNumSubscribers() == 0 && ros::ok())
                    ros::Duration(0.1).sleep();

                  /* set the initial angle */
                  std_msgs::Float64 msg;
                  nh_.getParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/init_value"), msg.data);
                  servo_target_pos_sim_pubs_[servo_group_params.first].back().publish(msg);
                }
            }
        }

      servos_handler_.insert(make_pair(servo_group_params.first, servo_group_handler));
    }

  const bool run_startup_torque =
    !startup_torque_enable_.empty() && !simulation_mode_ && !use_mujoco_;
  if (configure_direct_servo && direct_servo_params_valid) {
    if (run_startup_torque && startup_torque_params_valid_)
      startup_torque_state_ = StartupTorqueState::WAITING_FOR_CONFIG;
    direct_servo_configure_ =
      [this, direct_board_config, direct_servo_configs]() {
        std::set<uint8_t> configured_feedback_ids;
        const bool configured = configureDirectServos(
          nh_, direct_board_config, direct_servo_configs,
          DIRECT_SERVO_CONFIG_TIMEOUT, configured_feedback_ids);
        if (configured) {
          startup_expected_feedback_ids_.clear();
          std::set_intersection(
            configured_feedback_ids.begin(),
            configured_feedback_ids.end(),
            startup_real_state_servo_ids_.begin(),
            startup_real_state_servo_ids_.end(),
            std::inserter(startup_expected_feedback_ids_,
                          startup_expected_feedback_ids_.end()));
        }
        return configured;
      };
    direct_servo_config_timer_ = nh_.createWallTimer(
      ros::WallDuration(DIRECT_SERVO_CONFIG_STARTUP_DELAY),
      &ServoBridge::directServoConfigTimerCallback, this, true, true);
  } else if (!configure_direct_servo && run_startup_torque) {
    startup_expected_feedback_ids_ = startup_real_state_servo_ids_;
    if (!startup_torque_params_valid_ ||
        startup_all_servo_ids_.empty() ||
        startup_expected_feedback_ids_.empty()) {
      failStartupTorque(
        "Startup torque parameters contain no valid feedback-enabled servos");
    } else {
      startup_feedback_barrier_stamp_ = ros::Time(0);
      startup_received_feedback_ids_.clear();
      startup_torque_state_ = StartupTorqueState::WAITING_FOR_FEEDBACK;
      startStartupTorqueTimeout(STARTUP_FEEDBACK_TIMEOUT);
    }
  } else if (configure_direct_servo && !direct_servo_params_valid &&
             run_startup_torque) {
    failStartupTorque(
      "DirectServo parameters are invalid; startup torque is disabled");
  }

  if(!simulation_mode_) servo_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void ServoBridge::directServoConfigTimerCallback(
  const ros::WallTimerEvent& event)
{
  (void)event;
  if (!direct_servo_configure_) return;
  direct_servo_config_timer_.stop();
  const bool configured = direct_servo_configure_();
  direct_servo_configure_ = std::function<bool()>();
  if (!configured) {
    failStartupTorque(
      "DirectServo startup configuration or final readback failed");
    return;
  }

  ROS_INFO("DirectServo startup configuration completed");
  if (startup_torque_enable_.empty() || simulation_mode_ || use_mujoco_)
    return;
  if (!startup_torque_params_valid_ ||
      startup_all_servo_ids_.empty() ||
      startup_expected_feedback_ids_.empty()) {
    failStartupTorque(
      "No valid feedback-enabled servos remain after configuration");
    return;
  }

  startup_feedback_barrier_stamp_ = ros::Time::now();
  startup_received_feedback_ids_.clear();
  startup_torque_state_ = StartupTorqueState::WAITING_FOR_FEEDBACK;
  startStartupTorqueTimeout(STARTUP_FEEDBACK_TIMEOUT);
  ROS_INFO("Waiting for post-configuration feedback from %zu servos",
           startup_expected_feedback_ids_.size());
}

void ServoBridge::servoStatesCallback(
  const spinal::ServoStatesConstPtr& state_msg,
  const string& servo_group_name)
{
  /* independent group servo process without state publish */
  if(servo_group_name != std::string("common"))
    {
      for(auto it: state_msg->servos)
        {
          auto servo_handler = find_if(servos_handler_[servo_group_name].begin(),
                                       servos_handler_[servo_group_name].end(),
                                       [&](SingleServoHandlePtr s) {return it.index == s->getId();} );

          if(servo_handler == servos_handler_[servo_group_name].end())
            {
              ROS_ERROR("[servo bridge, servo state callback]: no matching joint handler for servo index %d", it.index);
              return;
            }
          (*servo_handler)->setCurrAngleVal((double)it.angle, ValueType::BIT); // angle (position)
          (*servo_handler)->setCurrVelocityVal((double)it.velocity); // velocity
          (*servo_handler)->setCurrTorqueVal((double)it.load); // torque (effort)
        }

      updateStartupFeedback(*state_msg);
      return;
    }


  /* group of "common", search in all group */
  for(auto it: state_msg->servos)
    {
      for(auto servo_group: servos_handler_)
        {
          // no real state from real machine, skip
          if(no_real_state_flags_.at(servo_group.first))
            {
              ROS_DEBUG_STREAM(servo_group.first << ", no real state, skip");
              continue;
            }

          // has independent servo state process
          if(servo_states_subs_.find(servo_group.first) != servo_states_subs_.end())
            {
              ROS_DEBUG_STREAM(servo_group.first << ", do not process in common group, skip");
              continue;
            }

          auto servo_handler = find_if(servos_handler_[servo_group.first].begin(),
                                       servos_handler_[servo_group.first].end(),
                                       [&](SingleServoHandlePtr s) {return it.index == s->getId();});

          if(servo_handler == servos_handler_[servo_group.first].end())
            {
              // search in next servo group
              continue;
            }

          (*servo_handler)->setCurrAngleVal((double)it.angle, ValueType::BIT); // angle (position)
          (*servo_handler)->setCurrVelocityVal((double)it.velocity); // velocity
          (*servo_handler)->setCurrTorqueVal((double)it.load); // torque (effort)

          ROS_DEBUG("servo index: %d, find in group %s", it.index, servo_group.first.c_str());

          break; // search next servo state from real machine
        }
    }


  sensor_msgs::JointState servo_states_msg;
  servo_states_msg.header.stamp = state_msg->stamp;

  for(auto servo_group : servos_handler_)
    {
      for(auto servo_handler: servo_group.second)
        {
          servo_states_msg.name.push_back(servo_handler->getName());
          servo_states_msg.position.push_back(servo_handler->getCurrAngleVal(ValueType::RADIAN));
          servo_states_msg.velocity.push_back(servo_handler->getCurrVelocityVal());
          servo_states_msg.effort.push_back(servo_handler->getCurrTorqueVal());
        }
    }
  servo_states_pub_.publish(servo_states_msg);
  updateStartupFeedback(*state_msg);
}

void ServoBridge::updateStartupFeedback(
  const spinal::ServoStates& state_msg)
{
  if (startup_torque_state_ !=
      StartupTorqueState::WAITING_FOR_FEEDBACK)
    return;

  if (!startup_feedback_barrier_stamp_.isZero() &&
      (state_msg.stamp.isZero() ||
       state_msg.stamp <= startup_feedback_barrier_stamp_)) {
    ROS_DEBUG("Ignoring servo feedback produced before configuration completed");
    return;
  }

  for (const spinal::ServoState& state : state_msg.servos) {
    if (startup_expected_feedback_ids_.count(state.index) == 0)
      continue;
    if (state.error != 0) {
      failStartupTorque(
        "Servo " + std::to_string(state.index) +
        " reported hardware error " + std::to_string(state.error));
      return;
    }
    startup_received_feedback_ids_.insert(state.index);
  }

  if (startup_received_feedback_ids_.size() ==
      startup_expected_feedback_ids_.size())
    applyStartupTorqueConfig();
}

void ServoBridge::applyStartupTorqueConfig()
{
  if (startup_torque_state_ !=
        StartupTorqueState::WAITING_FOR_FEEDBACK ||
      startup_received_feedback_ids_.size() !=
        startup_expected_feedback_ids_.size())
    return;

  map<uint8_t, bool> torque_enable;
  for (const auto& group_setting : startup_torque_enable_) {
    auto group = servos_handler_.find(group_setting.first);
    if (group == servos_handler_.end()) {
      failStartupTorque(
        "Unknown startup torque servo group " + group_setting.first);
      return;
    }
    for (const auto& servo : group->second) {
      const uint8_t id = static_cast<uint8_t>(servo->getId());
      if (!torque_enable.insert(
            std::make_pair(id, group_setting.second)).second) {
        failStartupTorque(
          "Duplicate global servo id " + std::to_string(id));
        return;
      }
    }
  }

  if (!publishServoTorque(torque_enable))
    return;

  startup_expected_torque_enable_ = torque_enable;
  startup_torque_timeout_timer_.stop();
  startup_torque_state_ =
    StartupTorqueState::WAITING_FOR_TORQUE_CONFIRMATION;
  startStartupTorqueTimeout(STARTUP_TORQUE_CONFIRM_TIMEOUT);
}

bool ServoBridge::publishServoTorque(
  const map<uint8_t, bool>& torque_enable)
{
  auto publisher = servo_enable_pubs_.find("common");
  if (publisher == servo_enable_pubs_.end() ||
      publisher->second.getNumSubscribers() == 0) {
    ROS_WARN_THROTTLE(1.0,
      "Waiting for the common servo torque command subscriber");
    return false;
  }

  spinal::ServoTorqueCmd torque_msg;
  for (const auto& servo_setting : torque_enable) {
    torque_msg.index.push_back(servo_setting.first);
    torque_msg.torque_enable.push_back(servo_setting.second);
  }
  if (torque_msg.index.empty()) {
    ROS_ERROR("Cannot apply startup torque state: no servo IDs");
    return false;
  }

  publisher->second.publish(torque_msg);
  ROS_INFO("Requested grouped startup torque state for %zu servos",
           torque_msg.index.size());
  return true;
}

bool ServoBridge::publishAllServoTorque(bool enable)
{
  map<uint8_t, bool> torque_enable;
  for (uint8_t id : startup_all_servo_ids_)
    torque_enable[id] = enable;
  return publishServoTorque(torque_enable);
}

void ServoBridge::servoTorqueStatesCallback(
  const spinal::ServoTorqueStatesConstPtr& state_msg)
{
  if (startup_torque_state_ !=
      StartupTorqueState::WAITING_FOR_TORQUE_CONFIRMATION)
    return;

  for (const auto& servo_setting : startup_expected_torque_enable_) {
    const uint8_t id = servo_setting.first;
    if (id >= state_msg->torque_enable.size()) {
      ROS_ERROR_THROTTLE(1.0,
        "Servo torque feedback is missing global servo id %u",
        static_cast<unsigned int>(id));
      return;
    }
    const bool enabled = state_msg->torque_enable[id] != 0;
    if (enabled != servo_setting.second)
      return;
  }

  startup_torque_timeout_timer_.stop();
  startup_torque_state_ = StartupTorqueState::READY;
  ROS_INFO("Confirmed grouped startup torque state for %zu servos",
           startup_expected_torque_enable_.size());
}

void ServoBridge::startStartupTorqueTimeout(double timeout)
{
  startup_torque_timeout_timer_.stop();
  startup_torque_timeout_timer_ = nh_.createWallTimer(
    ros::WallDuration(timeout),
    &ServoBridge::startupTorqueTimeoutCallback, this, true, true);
}

void ServoBridge::startupTorqueTimeoutCallback(
  const ros::WallTimerEvent& event)
{
  (void)event;
  if (startup_torque_state_ == StartupTorqueState::WAITING_FOR_FEEDBACK) {
    failStartupTorque(
      "Timed out after " + std::to_string(STARTUP_FEEDBACK_TIMEOUT) +
      " s waiting for valid post-configuration servo feedback (" +
      std::to_string(startup_received_feedback_ids_.size()) + "/" +
      std::to_string(startup_expected_feedback_ids_.size()) + ")");
  } else if (startup_torque_state_ ==
             StartupTorqueState::WAITING_FOR_TORQUE_CONFIRMATION) {
    failStartupTorque(
      "Timed out after " +
      std::to_string(STARTUP_TORQUE_CONFIRM_TIMEOUT) +
      " s waiting for servo torque confirmation");
  }
}

void ServoBridge::failStartupTorque(const std::string& reason)
{
  if (startup_torque_state_ == StartupTorqueState::FAILED)
    return;
  startup_torque_timeout_timer_.stop();
  startup_torque_state_ = StartupTorqueState::FAILED;
  ROS_ERROR("%s; startup torque enable is prohibited", reason.c_str());
  publishAllServoTorque(false);
}

void ServoBridge::servoCtrlCallback(const sensor_msgs::JointStateConstPtr& servo_ctrl_msg, const string& servo_group_name)
{
  auto group_it = servos_handler_.find(servo_group_name);
  if (group_it == servos_handler_.end()) {
    ROS_ERROR("Unknown servo group %s", servo_group_name.c_str());
    return;
  }
  const ServoGroupHandler& group = group_it->second;
  const size_t command_count = servo_ctrl_msg->name.empty()
    ? group.size() : servo_ctrl_msg->name.size();
  const bool has_position = !servo_ctrl_msg->position.empty();
  const bool has_velocity = !servo_ctrl_msg->velocity.empty();
  const bool has_effort = !servo_ctrl_msg->effort.empty();
  auto valid_size = [command_count](size_t size,
                                    const char* field) {
    if (size == 0 || size == command_count) return true;
    ROS_ERROR("%s size %zu does not match servo count %zu",
              field, size, command_count);
    return false;
  };
  if (command_count == 0 ||
      (!has_position && !has_velocity && !has_effort) ||
      !valid_size(servo_ctrl_msg->position.size(), "position") ||
      !valid_size(servo_ctrl_msg->velocity.size(), "velocity") ||
      !valid_size(servo_ctrl_msg->effort.size(), "effort")) {
    return;
  }

  spinal::ServoControlCmd target_state_msg;
  sensor_msgs::JointState mujoco_control_input_msg;
  mujoco_control_input_msg.header = servo_ctrl_msg->header;

  auto append_target = [&](const SingleServoHandlePtr& servo,
                           size_t command_index,
                           size_t group_index) {
    if (has_velocity && !servo->hasVelocityScale()) {
      ROS_ERROR("%s has no velocity_scale",
                servo->getName().c_str());
      return false;
    }
    if (has_effort && !servo->hasTorqueScale()) {
      ROS_ERROR("%s has no torque_scale",
                servo->getName().c_str());
      return false;
    }
    if (has_position)
      servo->setTargetAngleVal(
        servo_ctrl_msg->position[command_index],
        ValueType::RADIAN);
    if (has_velocity)
      servo->setTargetVelocityVal(
        servo_ctrl_msg->velocity[command_index]);
    if (has_effort)
      servo->setTargetTorqueVal(
        servo_ctrl_msg->effort[command_index]);

    target_state_msg.index.push_back(servo->getId());
    if (has_position) {
      const int32_t raw_position = static_cast<int32_t>(
        servo->getTargetAngleVal(ValueType::BIT));
      target_state_msg.position.push_back(raw_position);
    }
    if (has_velocity)
      target_state_msg.velocity.push_back(
        servo->getTargetVelocityRaw());
    if (has_effort) {
      const int16_t raw_current =
        servo->getTargetCurrentRaw();
      target_state_msg.current.push_back(raw_current);
    }

    mujoco_control_input_msg.name.push_back(servo->getName());
    if (has_position)
      mujoco_control_input_msg.position.push_back(
        servo_ctrl_msg->position[command_index]);
    if (has_velocity)
      mujoco_control_input_msg.velocity.push_back(
        servo_ctrl_msg->velocity[command_index]);
    if (has_effort)
      mujoco_control_input_msg.effort.push_back(
        servo_ctrl_msg->effort[command_index]);

    if (simulation_mode_) {
      std_msgs::Float64 msg;
      msg.data = has_position
        ? servo_ctrl_msg->position[command_index]
        : (has_velocity
           ? servo_ctrl_msg->velocity[command_index]
           : servo_ctrl_msg->effort[command_index]);
      servo_target_pos_sim_pubs_[servo_group_name]
        .at(group_index).publish(msg);
    }
    return true;
  };

  if (!servo_ctrl_msg->name.empty()) {
    for (size_t i = 0; i < command_count; ++i) {
      auto servo_it = find_if(
        group.begin(), group.end(),
        [&](const SingleServoHandlePtr& servo) {
          return servo_ctrl_msg->name[i] == servo->getName();
        });
      if (servo_it == group.end()) {
        ROS_ERROR("No servo named %s in group %s",
                  servo_ctrl_msg->name[i].c_str(),
                  servo_group_name.c_str());
        return;
      }
      if (!append_target(
            *servo_it, i, distance(group.begin(), servo_it))) {
        return;
      }
    }
  } else {
    for (size_t i = 0; i < command_count; ++i) {
      if (!append_target(group[i], i, i)) return;
    }
  }

  mujoco_control_input_pub_.publish(mujoco_control_input_msg);

  auto publisher = servo_target_pos_pubs_.find(servo_group_name);
  (publisher != servo_target_pos_pubs_.end()
     ? publisher->second : servo_target_pos_pubs_["common"])
    .publish(target_state_msg);
}

void ServoBridge::servoTorqueCtrlCallback(const sensor_msgs::JointStateConstPtr& servo_ctrl_msg, const string& servo_group_name)
{
  spinal::ServoControlCmd target_torque_msg;

  if(servo_ctrl_msg->name.size() > 0)
    {
      for(int i = 0; i < servo_ctrl_msg->name.size(); i++)
        {/* servo name is assigned */

          if(servo_ctrl_msg->effort.size() !=  servo_ctrl_msg->name.size())
            {
              ROS_ERROR("[servo bridge, servo torque control]: the servo effort (torque) num and name num are different in ros msgs [%d vs %d]",
                        (int)servo_ctrl_msg->effort.size(), (int)servo_ctrl_msg->name.size());
              return;
            }

          // use servo_name to search the servo_handler
          auto servo_handler = find_if(servos_handler_[servo_group_name].begin(), servos_handler_[servo_group_name].end(),
                                       [&](SingleServoHandlePtr s) {return servo_ctrl_msg->name.at(i)  == s->getName();});

          if(servo_handler == servos_handler_[servo_group_name].end())
          {
            ROS_ERROR("[servo bridge, servo control callback]: no matching servo handler for %s", servo_ctrl_msg->name.at(i).c_str());
            return;
          }

          double torque = servo_ctrl_msg->effort.at(i);
          (*servo_handler)->setTargetTorqueVal(torque);
          target_torque_msg.index.push_back((*servo_handler)->getId());
          target_torque_msg.current.push_back((*servo_handler)->getTargetCurrentRaw());
        }
    }
  else
    { /* for fast tranmission: no searching process, in the predefine order */

      if(servo_ctrl_msg->effort.size() != servos_handler_[servo_group_name].size())
        {
          ROS_ERROR("[servo bridge, servo control control]: the joint num from rosparam %d is not equal with ros msgs %d",
                    (int)servos_handler_[servo_group_name].size(), (int)servo_ctrl_msg->effort.size());
          return;
        }

      for(int i = 0; i < servo_ctrl_msg->effort.size(); i++)
        {
          /*  use the kinematics order (e.g. joint1 ~ joint N, gimbal_roll -> gimbal_pitch) */
          SingleServoHandlePtr servo_handler = servos_handler_[servo_group_name].at(i);
          double torque = servo_ctrl_msg->effort.at(i);

          servo_handler->setTargetTorqueVal(torque);
          target_torque_msg.index.push_back(servo_handler->getId());
          target_torque_msg.current.push_back(servo_handler->getTargetCurrentRaw());
        }
    }

  auto publisher = servo_target_pos_pubs_.find(servo_group_name);
  (publisher != servo_target_pos_pubs_.end()
     ? publisher->second : servo_target_pos_pubs_["common"])
    .publish(target_torque_msg);
}

bool ServoBridge::servoEnableCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& servo_group_name)
{
  spinal::ServoTorqueCmd torque_msg;
  for(auto servo_handler: servos_handler_[servo_group_name])
    {
      torque_msg.index.push_back(servo_handler->getId());
      torque_msg.torque_enable.push_back(req.data);
    }

  if (servo_enable_pubs_.find(servo_group_name) != servo_enable_pubs_.end())
    servo_enable_pubs_[servo_group_name].publish(torque_msg);
  else
    servo_enable_pubs_["common"].publish(torque_msg);

  res.success = true;
  res.message = req.data ? "Servo torque enabled" :
    "Servo torque disabled";
  return true;
}

void ServoBridge::uavInfoCallback(const spinal::UavInfoConstPtr& uav_msg)
{
  /* Send servo profiles to Spinal*/
  spinal::JointProfiles joint_profiles_msg;
  for(auto servo_group : servos_handler_){
    for(auto servo : servo_group.second){
      spinal::JointProfile joint_profile;
      if(servo_group.first == "joints"){
        joint_profile.type = spinal::JointProfile::JOINT;
      }
      else if(servo_group.first == "gimbals"){
        joint_profile.type = spinal::JointProfile::GIMBAL;
      }
      else{
        ROS_ERROR("Invalid servo type. Please define 'joints' or 'gimbals'.");
        continue;
      }
      joint_profile.servo_id = servo->getId();
      joint_profile.angle_sgn = servo->getAngleSgn();
      joint_profile.angle_scale = servo->getAngleScale();
      joint_profile.zero_point_offset = servo->getZeroPointOffset();
      joint_profiles_msg.joints.push_back(joint_profile);
    }
  }
  joint_profile_pub_.publish(joint_profiles_msg);
}

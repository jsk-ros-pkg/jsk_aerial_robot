#pragma once

#include <array>
#include <string>
#include <vector>

#include <mujoco_ros/ros_one/plugin_utils.hpp>

namespace mujoco_ros
{

class ThrustVisualizerPlugin : public MujocoPlugin
{
public:
  ThrustVisualizerPlugin() = default;
  ~ThrustVisualizerPlugin() override = default;

  bool Load(const mjModel* model, mjData* data) override;
  void Reset() override;
  void RenderCallback(const mjModel* model, mjData* data, mjvScene* scene) override;

private:
  struct ArrowSource
  {
    int actuator_id;
    int site_id;
    std::string actuator_name;
    std::string site_name;
  };

  void applyVisualDefaults(const mjModel* model);
  bool configureFromParams();
  bool buildConfiguredArrowSources(const mjModel* model);
  bool buildAutomaticArrowSources(const mjModel* model);
  int findSiteForActuator(const mjModel* model, int actuator_id, const std::string& fallback_site_name) const;
  void appendArrow(const mjModel* model, mjData* data, mjvScene* scene, const ArrowSource& arrow) const;

  static bool readDoubleParam(const XmlRpc::XmlRpcValue& config, const std::string& name, double& value);
  static bool readBoolParam(const XmlRpc::XmlRpcValue& config, const std::string& name, bool& value);
  static std::vector<std::string> readStringArrayParam(const XmlRpc::XmlRpcValue& config, const std::string& name);
  static bool readRgbaParam(const XmlRpc::XmlRpcValue& config, const std::string& name, std::array<float, 4>& value);
  static double xmlRpcNumberToDouble(const XmlRpc::XmlRpcValue& value, double default_value);

  std::vector<ArrowSource> arrows_;
  std::vector<std::string> actuator_names_;
  std::vector<std::string> site_names_;

  double scale_ = 0.005;
  double width_ = 0.005;
  double min_force_ = 1.0e-6;
  double max_length_ = 0.0;

  bool use_abs_force_ = false;
  bool direction_from_gear_ = true;
  bool include_all_site_actuators_ = true;

  std::array<float, 4> rgba_ = { 0.7f, 0.9f, 0.9f, 1.0f };
};

}  // namespace mujoco_ros

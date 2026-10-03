#include <aerial_robot_simulation/mujoco/thrust_visualizer_plugin.h>

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.h>

namespace mujoco_ros
{

bool ThrustVisualizerPlugin::Load(const mjModel* model, mjData* /*data*/)
{
  applyVisualDefaults(model);
  if (!configureFromParams())
  {
    return false;
  }

  arrows_.clear();
  const bool success = actuator_names_.empty() ? buildAutomaticArrowSources(model) : buildConfiguredArrowSources(model);
  if (!success || arrows_.empty())
  {
    ROS_ERROR_NAMED("thrust_visualizer", "No actuator-site pairs were found for thrust visualization.");
    return false;
  }

  ROS_INFO_STREAM_NAMED("thrust_visualizer", "Loaded thrust visualizer with " << arrows_.size() << " arrow(s), scale="
                                                                              << scale_ << ", width=" << width_ << ".");
  return true;
}

void ThrustVisualizerPlugin::Reset()
{
}

void ThrustVisualizerPlugin::RenderCallback(const mjModel* model, mjData* data, mjvScene* scene)
{
  for (const auto& arrow : arrows_)
  {
    if (scene->ngeom >= scene->maxgeom)
    {
      mj_warning(data, mjWARN_VGEOMFULL, scene->maxgeom);
      return;
    }

    appendArrow(model, data, scene, arrow);
  }
}

void ThrustVisualizerPlugin::applyVisualDefaults(const mjModel* model)
{
  scale_ = model->vis.map.force / std::max(static_cast<mjtNum>(mjMINVAL), model->stat.meanmass);
  width_ = model->stat.meansize * model->vis.scale.forcewidth;
  if (width_ <= 0.0)
  {
    width_ = model->vis.scale.forcewidth;
  }

  for (int i = 0; i < 4; ++i)
  {
    rgba_[i] = model->vis.rgba.contactforce[i];
  }
}

bool ThrustVisualizerPlugin::configureFromParams()
{
  readDoubleParam(rosparam_config_, "scale", scale_);
  readDoubleParam(rosparam_config_, "width", width_);
  readDoubleParam(rosparam_config_, "min_force", min_force_);
  readDoubleParam(rosparam_config_, "max_length", max_length_);
  readBoolParam(rosparam_config_, "use_abs_force", use_abs_force_);
  readBoolParam(rosparam_config_, "direction_from_gear", direction_from_gear_);
  readBoolParam(rosparam_config_, "include_all_site_actuators", include_all_site_actuators_);
  readRgbaParam(rosparam_config_, "rgba", rgba_);

  actuator_names_ = readStringArrayParam(rosparam_config_, "actuator_names");
  site_names_ = readStringArrayParam(rosparam_config_, "site_names");

  if (!site_names_.empty() && site_names_.size() != actuator_names_.size())
  {
    ROS_ERROR_NAMED("thrust_visualizer", "`site_names` must be omitted or have the same length as `actuator_names`.");
    return false;
  }

  if (scale_ <= 0.0)
  {
    ROS_ERROR_NAMED("thrust_visualizer", "`scale` must be positive.");
    return false;
  }

  if (width_ <= 0.0)
  {
    ROS_ERROR_NAMED("thrust_visualizer", "`width` must be positive.");
    return false;
  }

  min_force_ = std::max(0.0, min_force_);
  max_length_ = std::max(0.0, max_length_);

  return true;
}

bool ThrustVisualizerPlugin::buildConfiguredArrowSources(const mjModel* model)
{
  bool success = true;
  for (std::size_t i = 0; i < actuator_names_.size(); ++i)
  {
    const std::string& actuator_name = actuator_names_[i];
    const std::string site_name = site_names_.empty() ? std::string() : site_names_[i];
    const int actuator_id = mj_name2id(const_cast<mjModel*>(model), mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id < 0)
    {
      ROS_ERROR_STREAM_NAMED("thrust_visualizer", "Actuator `" << actuator_name << "` was not found.");
      success = false;
      continue;
    }

    const int site_id = findSiteForActuator(model, actuator_id, site_name);
    if (site_id < 0)
    {
      ROS_ERROR_STREAM_NAMED("thrust_visualizer", "No site was found for actuator `" << actuator_name << "`.");
      success = false;
      continue;
    }

    const char* resolved_site_name = mj_id2name(const_cast<mjModel*>(model), mjOBJ_SITE, site_id);
    arrows_.push_back({ actuator_id, site_id, actuator_name, resolved_site_name ? resolved_site_name : "" });
  }

  return success;
}

bool ThrustVisualizerPlugin::buildAutomaticArrowSources(const mjModel* model)
{
  if (!include_all_site_actuators_)
  {
    ROS_ERROR_NAMED("thrust_visualizer", "`actuator_names` is empty and `include_all_site_actuators` is false.");
    return false;
  }

  for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id)
  {
    if (model->actuator_trntype[actuator_id] != mjTRN_SITE)
    {
      continue;
    }

    const int site_id = model->actuator_trnid[2 * actuator_id];
    if (site_id < 0 || site_id >= model->nsite)
    {
      continue;
    }

    const char* actuator_name = mj_id2name(const_cast<mjModel*>(model), mjOBJ_ACTUATOR, actuator_id);
    const char* site_name = mj_id2name(const_cast<mjModel*>(model), mjOBJ_SITE, site_id);
    arrows_.push_back({ actuator_id, site_id, actuator_name ? actuator_name : "", site_name ? site_name : "" });
  }

  return !arrows_.empty();
}

int ThrustVisualizerPlugin::findSiteForActuator(const mjModel* model, int actuator_id,
                                                const std::string& fallback_site_name) const
{
  if (!fallback_site_name.empty())
  {
    return mj_name2id(const_cast<mjModel*>(model), mjOBJ_SITE, fallback_site_name.c_str());
  }

  if (model->actuator_trntype[actuator_id] == mjTRN_SITE)
  {
    return model->actuator_trnid[2 * actuator_id];
  }

  const char* actuator_name = mj_id2name(const_cast<mjModel*>(model), mjOBJ_ACTUATOR, actuator_id);
  if (!actuator_name)
  {
    return -1;
  }

  return mj_name2id(const_cast<mjModel*>(model), mjOBJ_SITE, actuator_name);
}

void ThrustVisualizerPlugin::appendArrow(const mjModel* model, mjData* data, mjvScene* scene,
                                         const ArrowSource& arrow) const
{
  double force = data->ctrl[arrow.actuator_id];
  if (use_abs_force_)
  {
    force = std::abs(force);
  }

  if (std::abs(force) < min_force_)
  {
    return;
  }

  mjtNum length = static_cast<mjtNum>(force * scale_);
  if (max_length_ > 0.0)
  {
    length = static_cast<mjtNum>(std::clamp(static_cast<double>(length), -max_length_, max_length_));
  }

  mjtNum local_dir[3] = { 0.0, 0.0, 1.0 };
  if (direction_from_gear_)
  {
    const mjtNum* gear = model->actuator_gear + 6 * arrow.actuator_id;
    if (mju_norm3(gear) > 1.0e-9)
    {
      mju_copy3(local_dir, gear);
      mju_normalize3(local_dir);
    }
  }

  const mjtNum* site_pos = data->site_xpos + 3 * arrow.site_id;
  const mjtNum* site_mat = data->site_xmat + 9 * arrow.site_id;
  mjtNum world_dir[3];
  mju_mulMatVec3(world_dir, site_mat, local_dir);

  mjtNum tip[3] = { site_pos[0] + length * world_dir[0], site_pos[1] + length * world_dir[1],
                    site_pos[2] + length * world_dir[2] };

  mjvGeom* geom = scene->geoms + scene->ngeom++;
  mjv_initGeom(geom, mjGEOM_ARROW, nullptr, nullptr, nullptr, rgba_.data());
  mjv_connector(geom, mjGEOM_ARROW, static_cast<mjtNum>(width_), site_pos, tip);
}

bool ThrustVisualizerPlugin::readDoubleParam(const XmlRpc::XmlRpcValue& config, const std::string& name, double& value)
{
  if (!config.hasMember(name))
  {
    return false;
  }

  value = xmlRpcNumberToDouble(config[name], value);
  return true;
}

bool ThrustVisualizerPlugin::readBoolParam(const XmlRpc::XmlRpcValue& config, const std::string& name, bool& value)
{
  if (!config.hasMember(name) || config[name].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
  {
    return false;
  }

  value = static_cast<bool>(config[name]);
  return true;
}

std::vector<std::string> ThrustVisualizerPlugin::readStringArrayParam(const XmlRpc::XmlRpcValue& config,
                                                                      const std::string& name)
{
  std::vector<std::string> values;
  if (!config.hasMember(name))
  {
    return values;
  }

  const XmlRpc::XmlRpcValue& xml_values = config[name];
  if (xml_values.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN_STREAM_NAMED("thrust_visualizer", "`" << name << "` must be a string array. Ignoring it.");
    return values;
  }

  for (int i = 0; i < xml_values.size(); ++i)
  {
    if (xml_values[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_WARN_STREAM_NAMED("thrust_visualizer", "`" << name << "` contains a non-string value. Ignoring it.");
      values.clear();
      return values;
    }
    values.push_back(static_cast<std::string>(xml_values[i]));
  }

  return values;
}

bool ThrustVisualizerPlugin::readRgbaParam(const XmlRpc::XmlRpcValue& config, const std::string& name,
                                           std::array<float, 4>& value)
{
  if (!config.hasMember(name))
  {
    return false;
  }

  const XmlRpc::XmlRpcValue& rgba = config[name];
  if (rgba.getType() != XmlRpc::XmlRpcValue::TypeArray || rgba.size() != 4)
  {
    ROS_WARN_STREAM_NAMED("thrust_visualizer", "`" << name << "` must be an array with 4 numbers. Ignoring it.");
    return false;
  }

  for (int i = 0; i < 4; ++i)
  {
    value[i] = static_cast<float>(xmlRpcNumberToDouble(rgba[i], value[i]));
  }

  return true;
}

double ThrustVisualizerPlugin::xmlRpcNumberToDouble(const XmlRpc::XmlRpcValue& value, double default_value)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    return static_cast<int>(value);
  }
  return default_value;
}

}  // namespace mujoco_ros

PLUGINLIB_EXPORT_CLASS(mujoco_ros::ThrustVisualizerPlugin, mujoco_ros::MujocoPlugin)

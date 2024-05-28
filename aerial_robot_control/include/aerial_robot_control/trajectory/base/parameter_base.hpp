#pragma once

#include <aerial_robot_control/trajectory/utils/filesystem.hpp>
#include <exception>
#include <sstream>

#include "aerial_robot_control/trajectory/math/types.hpp"
#include "aerial_robot_control/trajectory/utils/filesystem.hpp"
#include "aerial_robot_control/trajectory/utils/yaml.hpp"

namespace agi {


struct ParameterException : public std::exception {
  ParameterException() = default;
  ParameterException(const std::string& msg)
    : msg(std::string("Dodgelib Parameter Exception: ") + msg) {}
  const char* what() const throw() { return msg.c_str(); }

  const std::string msg{"Dodgelib Parameter Exception"};
};

struct ParameterBase {
  virtual ~ParameterBase() = default;
  virtual bool load(const fs::path& filename);
  virtual bool load(const Yaml& yaml);

  virtual bool valid() const;
};

}  // namespace agi

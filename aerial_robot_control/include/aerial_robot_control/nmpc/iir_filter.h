//
// Created by jinjie on 24/08/02.
//

#ifndef AERIAL_ROBOT_CONTROL_IIR_FILTER_H
#define AERIAL_ROBOT_CONTROL_IIR_FILTER_H

#include <iostream>
#include <vector>

namespace aerial_robot_control
{

class IIRFilter
{
public:
  // Initialize the filter with SOS and gain vectors
  void init(const std::vector<std::vector<double>>& sos, const std::vector<double>& g)
  {
    sos_ = sos;
    g_ = g;
    state_.resize(2 * sos.size(), 0.0);  // Initialize state for each section
  }

  // Update the filter with a new input sample and get the output sample
  double update(double input)
  {
    double sample = input;
    for (size_t i = 0; i < sos_.size(); ++i)
    {
      sample = g_[i] * applySOS(sos_[i], sample, state_.data() + 2 * i);
    }
    return sample;
  }

private:
  std::vector<std::vector<double>> sos_;
  std::vector<double> g_;
  std::vector<double> state_;

  // Apply one SOS section to a sample
  double applySOS(const std::vector<double>& section, double input, double* state)
  {
    double output = section[0] * input + state[0];
    state[0] = section[1] * input - section[4] * output + state[1];
    state[1] = section[2] * input - section[5] * output;
    return output;
  }
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_IIR_FILTER_H

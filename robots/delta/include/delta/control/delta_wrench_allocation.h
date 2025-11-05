#pragma once

#include <delta/model/delta_robot_model.h>
#include <delta/control/delta_controller.h>
#include <vector>
#include <Eigen/Core>

double nonlinearWrenchAllocationMinObjective(const std::vector<double>& x, std::vector<double>& grad, void* ptr);
void nonlinearWrenchAllocationEqConstraints(unsigned m, double* result, unsigned n, const double* x, double* grad,
                                            void* ptr);

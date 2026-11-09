# This file is included at the end of the generated aerial_robot_dynamicsConfig.cmake.
#
# include/aerial_robot_dynamics/robot_model.h publicly includes pinocchio and
# OsqpEigen headers, so every dependent package needs them as well.  Those are
# plain CMake packages, hence catkin cannot propagate them by itself: find them
# here and merge them into the variables exported to the dependent packages.
# This is the catkin counterpart of jrl-cmakemodules' add_project_dependency().

include(CMakeFindDependencyMacro)

find_dependency(Eigen3)
find_dependency(pinocchio)
find_dependency(OsqpEigen)
find_dependency(proxsuite)

# linking against the imported targets propagates their include directories,
# compile definitions (PINOCCHIO_WITH_*) and compile features
list(APPEND aerial_robot_dynamics_LIBRARIES
  Eigen3::Eigen
  pinocchio::pinocchio
  OsqpEigen::OsqpEigen
  proxsuite::proxsuite
  )

# For dependent packages which only use ${aerial_robot_dynamics_INCLUDE_DIRS}.
# The variables of each config file are used rather than the INTERFACE_INCLUDE_DIRECTORIES
# of the imported targets, because pinocchio::pinocchio is an INTERFACE target which only
# forwards to pinocchio::pinocchio_{default,parsers} and carries no include directory of
# its own.  Undefined variables simply append nothing.
list(APPEND aerial_robot_dynamics_INCLUDE_DIRS
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${proxsuite_INCLUDE_DIRS}
  )

list(REMOVE_DUPLICATES aerial_robot_dynamics_LIBRARIES)
list(REMOVE_DUPLICATES aerial_robot_dynamics_INCLUDE_DIRS)

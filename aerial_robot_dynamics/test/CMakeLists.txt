include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GTEST_INCLUDE_DIRS}
)

set(algorithm_test_sources
  algorithms/fd_test.cpp
  algorithms/fd_derivative_test.cpp
  algorithms/id_test.cpp
  algorithms/tauext_by_thrust_derivative_q_derivative_test.cpp
  algorithms/tauext_by_thrust_q_derivative_comparison_test.cpp
  algorithms/tauext_by_thrust_q_derivative_test.cpp
)

add_executable(pinocchio_robot_model_gtest
  test_main.cpp
  test_utils.cpp
  ${algorithm_test_sources}
)
target_link_libraries(pinocchio_robot_model_gtest
  robot_model
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_rostest(pinocchio_robot_model.test DEPENDENCIES pinocchio_robot_model_gtest)

install(TARGETS pinocchio_robot_model_gtest
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

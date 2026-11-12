find_package(benchmark REQUIRED)

set(benchmark_targets
  tauext_partial_q_derivative_hessian_benchmark
  tauext_partial_q_derivative_rnea_benchmark
  tauext_partial_q_derivative_static_torque_benchmark
  tauext_partial_q_derivative_numerical_benchmark
  inverse_dynamics_proxqp_benchmark
  )

foreach(benchmark_target ${benchmark_targets})
  add_executable(${benchmark_target} ${benchmark_target}.cpp)
  target_link_libraries(${benchmark_target}
    robot_model
    ${catkin_LIBRARIES}
    benchmark::benchmark
    )
endforeach()

install(TARGETS ${benchmark_targets}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

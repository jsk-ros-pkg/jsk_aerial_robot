add_rostest(hydrus_xi_jacobian.test ARGS headless:=true)
# alternative build command: 
# add_rostest_gtest(hydrus_xi_jacobian_test hydrus_xi_jacobian.test hydrus_xi/jacobian_test.cpp)
# target_link_libraries(hydrus_xi_jacobian_test hydrus_xi_fully_actuated_robot_model ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest(hydrus_xi_hex_branch_control.test ARGS headless:=true)


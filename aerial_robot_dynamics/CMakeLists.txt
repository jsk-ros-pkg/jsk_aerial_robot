cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_dynamics)

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(proxsuite REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(eigenpy REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

option(BUILD_BENCH "Build Benchmark Code" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_model
  CATKIN_DEPENDS roscpp urdf
  CFG_EXTRAS aerial_robot_dynamics-extras.cmake
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  )

add_library(robot_model src/robot_model.cpp)
target_link_libraries(robot_model
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  OsqpEigen::OsqpEigen
  proxsuite::proxsuite
  )

add_library(robot_model_py MODULE src/python_bindings.cpp)
set_target_properties(robot_model_py PROPERTIES
  PREFIX ""
  OUTPUT_NAME "_robot_model"
  SUFFIX ".${Python3_SOABI}.so"
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  )
target_link_libraries(robot_model_py PRIVATE
  robot_model
  eigenpy::eigenpy
  Python3::Module
  )

set(${PROJECT_NAME}_PYTHON_FILES
  src/aerial_robot_dynamics/__init__.py
  src/aerial_robot_dynamics/robot_description.py
  )
foreach(python_file ${${PROJECT_NAME}_PYTHON_FILES})
  get_filename_component(python_file_name ${python_file} NAME)
  configure_file(${python_file}
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/${python_file_name}
    COPYONLY)
endforeach()

install(TARGETS robot_model
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS robot_model_py
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(FILES ${${PROJECT_NAME}_PYTHON_FILES}
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# add dependencies to the PYTHONPATH
catkin_add_env_hooks(25.${PROJECT_NAME}_pythonpath
  SHELLS sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_BENCH)
  add_subdirectory(benchmark)
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()

cmake_minimum_required(VERSION 2.8.3)
project(spinal_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosserial_msgs
  rosserial_server
  std_msgs
  topic_tools
  message_generation
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
)

add_message_files(
  FILES
  SerializedMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rosserial_msgs
    rosserial_server
    std_msgs
    topic_tools
)

include_directories(
  include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(serial_node src/serial_node.cpp)
target_link_libraries(serial_node ${catkin_LIBRARIES})
set_target_properties(serial_node PROPERTIES OUTPUT_NAME serial_node PREFIX "")
add_dependencies(serial_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(udp_socket_node src/udp_socket_node.cpp)
target_link_libraries(udp_socket_node ${catkin_LIBRARIES})
add_dependencies(udp_socket_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


add_executable(multi_thread_test_node src/multi_thread_test.cpp)
target_link_libraries(multi_thread_test_node ${catkin_LIBRARIES})

install(
  TARGETS
  serial_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

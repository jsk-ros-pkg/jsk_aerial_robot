cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

## Declare ROS messages and services
add_message_files(FILES AttGains.msg DynamicReconfigureLevels.msg ITermBias.msg KduinoSimpleImu.msg RPYCtrlOffset.msg VirtualRC.msg AttitudePDGain.msg HydraParam.msg KduinoImu.msg MotorBias.msg RcData.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()




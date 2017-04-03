/**
 * @brief MAVROS Node
 * @file mavros_node.cpp
 * @author Moju Zhao <chou@jsk.imi.i.u-tokyo.ac.jp>
 *
 */
/*
 * mavros
 * Copyright 2017 JSK, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 */

#include <aerial_robot_comm/mavros/mavros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mavros_bridge");

  mavros::MavRos2 mavros_bridge;
  mavros_bridge.spin();

  return 0;
}

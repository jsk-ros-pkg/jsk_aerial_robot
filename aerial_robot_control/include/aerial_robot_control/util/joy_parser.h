// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, DRAGON Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#pragma once

#include <sensor_msgs/Joy.h>

/* general joystick bottons/axes layout */
const int JOY_BUTTON_SIZE              = 17;
const int JOY_BUTTON_STOP            = 0;
const int JOY_BUTTON_STICK_LEFT        = 1;
const int JOY_BUTTON_STICK_RIGHT       = 2;
const int JOY_BUTTON_START             = 3;
const int JOY_BUTTON_CROSS_UP          = 4;
const int JOY_BUTTON_CROSS_RIGHT       = 5;
const int JOY_BUTTON_CROSS_DOWN        = 6;
const int JOY_BUTTON_CROSS_LEFT        = 7;
const int JOY_BUTTON_REAR_LEFT_2       = 8;
const int JOY_BUTTON_REAR_RIGHT_2      = 9;
const int JOY_BUTTON_REAR_LEFT_1       = 10;
const int JOY_BUTTON_REAR_RIGHT_1      = 11;
const int JOY_BUTTON_ACTION_TRIANGLE   = 12;
const int JOY_BUTTON_ACTION_CIRCLE     = 13;
const int JOY_BUTTON_ACTION_CROSS      = 14;
const int JOY_BUTTON_ACTION_SQUARE     = 15;
const int JOY_BUTTON_PAIRING           = 16;
const int JOY_AXIS_SIZE                    = 29;
const int JOY_AXIS_STICK_LEFT_LEFTWARDS    = 0;
const int JOY_AXIS_STICK_LEFT_UPWARDS      = 1;
const int JOY_AXIS_STICK_RIGHT_LEFTWARDS   = 2;
const int JOY_AXIS_STICK_RIGHT_UPWARDS     = 3;
const int JOY_AXIS_BUTTON_CROSS_UP         = 4;
const int JOY_AXIS_BUTTON_CROSS_RIGHT      = 5;
const int JOY_AXIS_BUTTON_CROSS_DOWN       = 6;
const int JOY_AXIS_BUTTON_CROSS_LEFT       = 7;
const int JOY_AXIS_BUTTON_REAR_LEFT_2      = 8;
const int JOY_AXIS_BUTTON_REAR_RIGHT_2     = 9;
const int JOY_AXIS_BUTTON_REAR_LEFT_1      = 10;
const int JOY_AXIS_BUTTON_REAR_RIGHT_1     = 11;
const int JOY_AXIS_BUTTON_ACTION_TRIANGLE  = 12;
const int JOY_AXIS_BUTTON_ACTION_CIRCLE    = 13;
const int JOY_AXIS_BUTTON_ACTION_CROSS     = 14;
const int JOY_AXIS_BUTTON_ACTION_SQUARE    = 15;
const int JOY_AXIS_ACCELEROMETER_LEFT      = 16;
const int JOY_AXIS_ACCELEROMETER_FORWARD   = 17;
const int JOY_AXIS_ACCELEROMETER_UP        = 18;
const int JOY_AXIS_GYRO_YAW                = 19;

/* playstation dualschock 3 joystick: same with general layout */
const int PS3_BUTTON_SIZE                  = 17;
const int PS3_AXIS_SIZE                    = 29;

/* playstation dualschock 4 joystick */
const int PS4_BUTTON_SIZE              = 14;
const int PS4_BUTTON_ACTION_SQUARE     = 0;
const int PS4_BUTTON_ACTION_CROSS      = 1;
const int PS4_BUTTON_ACTION_CIRCLE     = 2;
const int PS4_BUTTON_ACTION_TRIANGLE   = 3;
const int PS4_BUTTON_REAR_LEFT_1       = 4;
const int PS4_BUTTON_REAR_RIGHT_1      = 5;
const int PS4_BUTTON_REAR_LEFT_2       = 6;
const int PS4_BUTTON_REAR_RIGHT_2      = 7;
const int PS4_BUTTON_SHARE             = 8;
const int PS4_BUTTON_OPTIONS           = 9;
const int PS4_BUTTON_STICK_LEFT        = 10;
const int PS4_BUTTON_STICK_RIGHT       = 11;
const int PS4_BUTTON_PAIRING           = 12;
const int PS4_BUTTON_TOUCHPAD          = 13;
const int PS4_AXIS_SIZE                    = 14;
const int PS4_AXIS_STICK_LEFT_LEFTWARDS    = 0;
const int PS4_AXIS_STICK_LEFT_UPWARDS      = 1;
const int PS4_AXIS_STICK_RIGHT_LEFTWARDS   = 2;
const int PS4_AXIS_BUTTON_REAR_LEFT_2      = 3; // neutral=+1, full accel=-1
const int PS4_AXIS_BUTTON_REAR_RIGHT_2     = 4; // neutral=+1, full accel=-1
const int PS4_AXIS_STICK_RIGHT_UPWARDS     = 5;
const int PS4_AXIS_ACCELEROMETER_LEFT      = 6;
const int PS4_AXIS_ACCELEROMETER_FORWARD   = 7;
const int PS4_AXIS_ACCELEROMETER_UP        = 8;
const int PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT = 9; // left = +1, right= -1
const int PS4_AXIS_BUTTON_CROSS_UP_DOWN    = 10; // up = +1, down= -1
const int PS4_AXIS_GYRO_ROLL               = 11;
const int PS4_AXIS_GYRO_YAW                = 12;
const int PS4_AXIS_GYRO_PITCH              = 13;

/* ps4-like bluetooth joystick */
const int BLT_BUTTON_SIZE              = 13;
const int BLT_BUTTON_ACTION_CROSS      = 0;
const int BLT_BUTTON_ACTION_CIRCLE     = 1;
const int BLT_BUTTON_ACTION_TRIANGLE   = 2;
const int BLT_BUTTON_ACTION_SQUARE     = 3;
const int BLT_BUTTON_REAR_LEFT_1       = 4;
const int BLT_BUTTON_REAR_RIGHT_1      = 5;
const int BLT_BUTTON_REAR_LEFT_2       = 6;
const int BLT_BUTTON_REAR_RIGHT_2      = 7;
const int BLT_BUTTON_SHARE             = 8;
const int BLT_BUTTON_OPTIONS           = 9;
const int BLT_BUTTON_PAIRING           = 10;
const int BLT_BUTTON_STICK_LEFT        = 11;
const int BLT_BUTTON_STICK_RIGHT       = 12;
const int BLT_AXIS_SIZE                    = 8;
const int BLT_AXIS_STICK_LEFT_LEFTWARDS    = 0;
const int BLT_AXIS_STICK_LEFT_UPWARDS      = 1;
const int BLT_AXIS_BUTTON_REAR_LEFT_2      = 2; // neutral=+1, full accel=-1
const int BLT_AXIS_STICK_RIGHT_LEFTWARDS   = 3;
const int BLT_AXIS_STICK_RIGHT_UPWARDS     = 4;
const int BLT_AXIS_BUTTON_REAR_RIGHT_2     = 5; // neutral=+1, full accel=-1
const int BLT_AXIS_BUTTON_CROSS_LEFT_RIGHT = 6; // left = +1, right= -1
const int BLT_AXIS_BUTTON_CROSS_UP_DOWN    = 7; // up = +1, down= -1

/* ROG1 embeded joystick */
const int ROG1_BUTTON_SIZE              = 11;
const int ROG1_BUTTON_ACTION_A          = 0;
const int ROG1_BUTTON_ACTION_B          = 1;
const int ROG1_BUTTON_ACTION_X          = 2;
const int ROG1_BUTTON_ACTION_Y          = 3;
const int ROG1_BUTTON_REAR_LEFT_1       = 4;
const int ROG1_BUTTON_REAR_RIGHT_1      = 5;
const int ROG1_BUTTON_DUO_RECT          = 6;
const int ROG1_BUTTON_TRI_LINE          = 7;
const int ROG1_BUTTON_STICK_LEFT        = 9;
const int ROG1_BUTTON_STICK_RIGHT       = 10;
const int ROG1_AXIS_SIZE                    = 8;
const int ROG1_AXIS_STICK_LEFT_LEFTWARDS    = 0;
const int ROG1_AXIS_STICK_LEFT_UPWARDS      = 1;
const int ROG1_AXIS_BUTTON_REAR_LEFT_2      = 2; // neutral=+1, full accel=-1
const int ROG1_AXIS_STICK_RIGHT_LEFTWARDS   = 3;
const int ROG1_AXIS_STICK_RIGHT_UPWARDS     = 4;
const int ROG1_AXIS_BUTTON_REAR_RIGHT_2     = 5; // neutral=+1, full accel=-1
const int ROG1_AXIS_BUTTON_CROSS_LEFT_RIGHT = 6; // left = +1, right= -1
const int ROG1_AXIS_BUTTON_CROSS_UP_DOWN    = 7; // up = +1, down= -1


const sensor_msgs::Joy joyParse(const sensor_msgs::Joy& ps4_joy_msg);


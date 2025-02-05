#include "aerial_robot_control/util/joy_parser.h"

const sensor_msgs::Joy joyParse(const sensor_msgs::Joy& joy_msg)
{
  sensor_msgs::Joy joy_cmd;

  int a_size = joy_msg.axes.size();
  int b_size = joy_msg.buttons.size();

  if (a_size == PS3_AXIS_SIZE && b_size == PS3_BUTTON_SIZE)
    {
      joy_cmd = joy_msg;
    }

  if (a_size == PS4_AXIS_SIZE && b_size == PS4_BUTTON_SIZE)
    {
      joy_cmd.header = joy_msg.header;
      joy_cmd.axes.resize(JOY_AXIS_SIZE, 0);
      joy_cmd.buttons.resize(JOY_BUTTON_SIZE, 0);
      joy_cmd.buttons[JOY_BUTTON_START] = joy_msg.buttons[PS4_BUTTON_OPTIONS];
      joy_cmd.buttons[JOY_BUTTON_STOP] = joy_msg.buttons[PS4_BUTTON_SHARE];
      joy_cmd.buttons[JOY_BUTTON_STICK_LEFT] = joy_msg.buttons[PS4_BUTTON_STICK_LEFT];
      joy_cmd.buttons[JOY_BUTTON_STICK_RIGHT] = joy_msg.buttons[PS4_BUTTON_STICK_RIGHT];
      if(joy_msg.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == 1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_UP] = 1;
      if(joy_msg.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == -1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] = 1;
      if(joy_msg.axes[PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] = 1;
      if(joy_msg.axes[PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] = 1;
      joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] = joy_msg.buttons[PS4_BUTTON_REAR_LEFT_1];
      joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1] = joy_msg.buttons[PS4_BUTTON_REAR_RIGHT_1];
      joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_2] = joy_msg.buttons[PS4_BUTTON_REAR_LEFT_2];
      joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_2] = joy_msg.buttons[PS4_BUTTON_REAR_RIGHT_2];
      joy_cmd.buttons[JOY_BUTTON_ACTION_TRIANGLE] = joy_msg.buttons[PS4_BUTTON_ACTION_TRIANGLE];
      joy_cmd.buttons[JOY_BUTTON_ACTION_CIRCLE] = joy_msg.buttons[PS4_BUTTON_ACTION_CIRCLE];
      joy_cmd.buttons[JOY_BUTTON_ACTION_CROSS] = joy_msg.buttons[PS4_BUTTON_ACTION_CROSS];
      joy_cmd.buttons[JOY_BUTTON_ACTION_SQUARE] = joy_msg.buttons[PS4_BUTTON_ACTION_SQUARE];
      joy_cmd.buttons[JOY_BUTTON_PAIRING] = joy_msg.buttons[PS4_BUTTON_PAIRING];
      joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] = joy_msg.axes[PS4_AXIS_STICK_LEFT_LEFTWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] = joy_msg.axes[PS4_AXIS_STICK_LEFT_UPWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS] = joy_msg.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] = joy_msg.axes[PS4_AXIS_STICK_RIGHT_UPWARDS];
      joy_cmd.axes[JOY_AXIS_ACCELEROMETER_LEFT] = joy_msg.axes[PS4_AXIS_ACCELEROMETER_LEFT];
      joy_cmd.axes[JOY_AXIS_ACCELEROMETER_FORWARD] = joy_msg.axes[PS4_AXIS_ACCELEROMETER_FORWARD];
      joy_cmd.axes[JOY_AXIS_ACCELEROMETER_UP] = joy_msg.axes[PS4_AXIS_ACCELEROMETER_UP];
      joy_cmd.axes[JOY_AXIS_GYRO_YAW] = joy_msg.axes[PS4_AXIS_GYRO_YAW];
    }

  if (a_size == BLT_AXIS_SIZE && b_size == BLT_BUTTON_SIZE)
    {
      joy_cmd.header = joy_msg.header;
      joy_cmd.axes.resize(JOY_AXIS_SIZE, 0);
      joy_cmd.buttons.resize(JOY_BUTTON_SIZE, 0);
      joy_cmd.buttons[JOY_BUTTON_START] = joy_msg.buttons[BLT_BUTTON_OPTIONS];
      joy_cmd.buttons[JOY_BUTTON_STOP] = joy_msg.buttons[BLT_BUTTON_SHARE];
      joy_cmd.buttons[JOY_BUTTON_STICK_LEFT] = joy_msg.buttons[BLT_BUTTON_STICK_LEFT];
      joy_cmd.buttons[JOY_BUTTON_STICK_RIGHT] = joy_msg.buttons[BLT_BUTTON_STICK_RIGHT];
      if(joy_msg.axes[BLT_AXIS_BUTTON_CROSS_UP_DOWN] == 1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_UP] = 1;
      if(joy_msg.axes[BLT_AXIS_BUTTON_CROSS_UP_DOWN] == -1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] = 1;
      if(joy_msg.axes[BLT_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] = 1;
      if(joy_msg.axes[BLT_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] = 1;
      joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] = joy_msg.buttons[BLT_BUTTON_REAR_LEFT_1];
      joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1] = joy_msg.buttons[BLT_BUTTON_REAR_RIGHT_1];
      joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_2] = joy_msg.buttons[BLT_BUTTON_REAR_LEFT_2];
      joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_2] = joy_msg.buttons[BLT_BUTTON_REAR_RIGHT_2];
      joy_cmd.buttons[JOY_BUTTON_ACTION_TRIANGLE] = joy_msg.buttons[BLT_BUTTON_ACTION_TRIANGLE];
      joy_cmd.buttons[JOY_BUTTON_ACTION_CIRCLE] = joy_msg.buttons[BLT_BUTTON_ACTION_CIRCLE];
      joy_cmd.buttons[JOY_BUTTON_ACTION_CROSS] = joy_msg.buttons[BLT_BUTTON_ACTION_CROSS];
      joy_cmd.buttons[JOY_BUTTON_ACTION_SQUARE] = joy_msg.buttons[BLT_BUTTON_ACTION_SQUARE];
      joy_cmd.buttons[JOY_BUTTON_PAIRING] = joy_msg.buttons[BLT_BUTTON_PAIRING];
      joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] = joy_msg.axes[BLT_AXIS_STICK_LEFT_LEFTWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] = joy_msg.axes[BLT_AXIS_STICK_LEFT_UPWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS] = joy_msg.axes[BLT_AXIS_STICK_RIGHT_LEFTWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] = joy_msg.axes[BLT_AXIS_STICK_RIGHT_UPWARDS];
    }

    if (a_size == BLT_AXIS_SIZE && b_size == BLT_BUTTON_SIZE)
    {
      joy_cmd.header = joy_msg.header;
      joy_cmd.axes.resize(JOY_AXIS_SIZE, 0);
      joy_cmd.buttons.resize(JOY_BUTTON_SIZE, 0);
      joy_cmd.buttons[JOY_BUTTON_START] = joy_msg.buttons[BLT_BUTTON_OPTIONS];
      joy_cmd.buttons[JOY_BUTTON_STOP] = joy_msg.buttons[BLT_BUTTON_SHARE];
      joy_cmd.buttons[JOY_BUTTON_STICK_LEFT] = joy_msg.buttons[BLT_BUTTON_STICK_LEFT];
      joy_cmd.buttons[JOY_BUTTON_STICK_RIGHT] = joy_msg.buttons[BLT_BUTTON_STICK_RIGHT];
      if(joy_msg.axes[BLT_AXIS_BUTTON_CROSS_UP_DOWN] == 1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_UP] = 1;
      if(joy_msg.axes[BLT_AXIS_BUTTON_CROSS_UP_DOWN] == -1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] = 1;
      if(joy_msg.axes[BLT_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] = 1;
      if(joy_msg.axes[BLT_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] = 1;
      joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] = joy_msg.buttons[BLT_BUTTON_REAR_LEFT_1];
      joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1] = joy_msg.buttons[BLT_BUTTON_REAR_RIGHT_1];
      joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_2] = joy_msg.buttons[BLT_BUTTON_REAR_LEFT_2];
      joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_2] = joy_msg.buttons[BLT_BUTTON_REAR_RIGHT_2];
      joy_cmd.buttons[JOY_BUTTON_ACTION_TRIANGLE] = joy_msg.buttons[BLT_BUTTON_ACTION_TRIANGLE];
      joy_cmd.buttons[JOY_BUTTON_ACTION_CIRCLE] = joy_msg.buttons[BLT_BUTTON_ACTION_CIRCLE];
      joy_cmd.buttons[JOY_BUTTON_ACTION_CROSS] = joy_msg.buttons[BLT_BUTTON_ACTION_CROSS];
      joy_cmd.buttons[JOY_BUTTON_ACTION_SQUARE] = joy_msg.buttons[BLT_BUTTON_ACTION_SQUARE];
      joy_cmd.buttons[JOY_BUTTON_PAIRING] = joy_msg.buttons[BLT_BUTTON_PAIRING];
      joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] = joy_msg.axes[BLT_AXIS_STICK_LEFT_LEFTWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] = joy_msg.axes[BLT_AXIS_STICK_LEFT_UPWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS] = joy_msg.axes[BLT_AXIS_STICK_RIGHT_LEFTWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] = joy_msg.axes[BLT_AXIS_STICK_RIGHT_UPWARDS];
    }

    if (a_size == ROG1_AXIS_SIZE && b_size == ROG1_BUTTON_SIZE)
    {
      joy_cmd.header = joy_msg.header;
      joy_cmd.axes.resize(JOY_AXIS_SIZE, 0);
      joy_cmd.buttons.resize(JOY_BUTTON_SIZE, 0);
      joy_cmd.buttons[JOY_BUTTON_START] = joy_msg.buttons[ROG1_BUTTON_TRI_LINE];
      joy_cmd.buttons[JOY_BUTTON_STOP] = joy_msg.buttons[ROG1_BUTTON_DUO_RECT];
      joy_cmd.buttons[JOY_BUTTON_STICK_LEFT] = joy_msg.buttons[ROG1_BUTTON_STICK_LEFT];
      joy_cmd.buttons[JOY_BUTTON_STICK_RIGHT] = joy_msg.buttons[ROG1_BUTTON_STICK_RIGHT];
      if(joy_msg.axes[ROG1_AXIS_BUTTON_CROSS_UP_DOWN] == 1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_UP] = 1;
      if(joy_msg.axes[ROG1_AXIS_BUTTON_CROSS_UP_DOWN] == -1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] = 1;
      if(joy_msg.axes[ROG1_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] = 1;
      if(joy_msg.axes[ROG1_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1)
        joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] = 1;
      joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] = joy_msg.buttons[ROG1_BUTTON_REAR_LEFT_1];
      joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1] = joy_msg.buttons[ROG1_BUTTON_REAR_RIGHT_1];
      joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_2] = (joy_msg.axes[ROG1_AXIS_BUTTON_REAR_LEFT_2]==-1)?1:0;
      joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_2] = (joy_msg.axes[ROG1_AXIS_BUTTON_REAR_RIGHT_2]==-1)?1:0;
      joy_cmd.buttons[JOY_BUTTON_ACTION_CROSS] = joy_msg.buttons[ROG1_BUTTON_ACTION_A];
      joy_cmd.buttons[JOY_BUTTON_ACTION_CIRCLE] = joy_msg.buttons[ROG1_BUTTON_ACTION_B];
      joy_cmd.buttons[JOY_BUTTON_ACTION_SQUARE] = joy_msg.buttons[ROG1_BUTTON_ACTION_X];
      joy_cmd.buttons[JOY_BUTTON_ACTION_TRIANGLE] = joy_msg.buttons[ROG1_BUTTON_ACTION_Y];
      joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] = joy_msg.axes[ROG1_AXIS_STICK_LEFT_LEFTWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] = joy_msg.axes[ROG1_AXIS_STICK_LEFT_UPWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS] = joy_msg.axes[ROG1_AXIS_STICK_RIGHT_LEFTWARDS];
      joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] = joy_msg.axes[ROG1_AXIS_STICK_RIGHT_UPWARDS];
    }

  return joy_cmd;
}

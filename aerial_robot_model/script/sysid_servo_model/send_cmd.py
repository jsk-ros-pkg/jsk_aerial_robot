'''
created by Jinjie LI, 2024/02/15
'''

import rospy
import argparse
from spinal.msg import ServoControlCmd


def rad_2_kondo_pos(angle):
    kondo_pos = (11500 - 3500) * (-angle - (-2.36)) / (2.36 - (-2.36)) + 3500
    return int(kondo_pos)


def kondo_pos_2_rad(pos):
    angle = -((2.36 - (-2.36)) * (pos - 3500) / (11500 - 3500) + (-2.36))
    return float(angle)


# use ros timer to send the command
class ServoControlCmdSender:
    def __init__(self, init_set_angle: float):
        rospy.init_node('send_cmd', anonymous=True)
        self.pub = rospy.Publisher('/kondo_servo/states_cmd', ServoControlCmd, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.send_cmd)  # 100Hz
        self.cmd = ServoControlCmd()
        self.cmd.index = [1, 2, 3, 4]
        self.cmd.angles = [rad_2_kondo_pos(init_set_angle), rad_2_kondo_pos(init_set_angle),
                           rad_2_kondo_pos(init_set_angle), rad_2_kondo_pos(init_set_angle)]

    def set_angle(self, angle):
        self.cmd.angles = [rad_2_kondo_pos(angle), rad_2_kondo_pos(angle),
                           rad_2_kondo_pos(angle), rad_2_kondo_pos(angle)]

    def send_cmd(self, event):
        self.pub.publish(self.cmd)


if __name__ == '__main__':
    # add one argument called "set_angle"
    parser = argparse.ArgumentParser(description='Set the angle of the servo', add_help=True)
    parser.add_argument('--init_set_angle', type=float, help='The angle to set the servo to', default=0.0)

    sender = ServoControlCmdSender(parser.parse_args().init_set_angle)

    while True:
        try:
            sender.set_angle(float(input("Enter the angle (rad): ")))
        except rospy.ROSInterruptException:
            break
        except ValueError:
            print("Please enter a valid number. Input Ctrl+Z to exit.")
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            break

#!/usr/bin/env python3

import rospy

from spinal.msg import PwmTest, ServoControlCmd


MOTOR_INDEX = 3  # Fourth motor (zero-based index)
SERVO_INDEX = 7

PWM_VALUES = (0.70, 0.75, 0.80)
SERVO_SPEEDS = (2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0, 32.0, 34.0, 36.0, 38.0, 40.0, 42.0, 44.0, 46.0, 48.0, 50.0, 52.0, 54.0, 56.0, 58.0, 60.0)  # deg/s

START_ANGLE = 90
END_ANGLE = 36
HOLD_DURATION = 5.0
SERVO_RESET_WAIT = 1.0
COMMAND_RATE = 20.0
STOP_PWM = 0.5


def publish_motor(publisher, pwm):
    msg = PwmTest()
    msg.motor_index = [MOTOR_INDEX]
    msg.pwms = [pwm]
    publisher.publish(msg)


def publish_servo(publisher, angle):
    msg = ServoControlCmd()
    msg.index = [SERVO_INDEX]
    msg.angles = [int(round(angle))]
    publisher.publish(msg)


def reset_servo(publisher):
    publish_servo(publisher, START_ANGLE)
    rospy.sleep(SERVO_RESET_WAIT)


def run_fixed_servo_test(motor_publisher, servo_publisher, pwm):
    reset_servo(servo_publisher)
    rospy.loginfo("PWM %.2f: servo fixed at %d deg for %.1f s",
                  pwm, START_ANGLE, HOLD_DURATION)
    publish_motor(motor_publisher, pwm)
    rospy.sleep(HOLD_DURATION)
    publish_motor(motor_publisher, STOP_PWM)


def run_moving_servo_test(motor_publisher, servo_publisher, pwm, speed):
    reset_servo(servo_publisher)
    rospy.loginfo("PWM %.2f: servo %d -> %d deg at %.1f deg/s",
                  pwm, START_ANGLE, END_ANGLE, speed)

    duration = float(START_ANGLE - END_ANGLE) / speed
    steps = int(round(duration * COMMAND_RATE))
    rate = rospy.Rate(COMMAND_RATE)

    publish_motor(motor_publisher, pwm)
    for step in range(steps + 1):
        angle = START_ANGLE + (END_ANGLE - START_ANGLE) * float(step) / steps
        publish_servo(servo_publisher, angle)
        if step < steps:
            rate.sleep()
    publish_motor(motor_publisher, STOP_PWM)


def main():
    rospy.init_node("dc_motor_servo_test")

    motor_publisher = rospy.Publisher("/pwm_test", PwmTest, queue_size=1)
    servo_publisher = rospy.Publisher(
        "/extra_servo_cmd", ServoControlCmd, queue_size=1)
    rospy.sleep(1.0)

    try:
        for pwm in PWM_VALUES:
            run_fixed_servo_test(motor_publisher, servo_publisher, pwm)
            for speed in SERVO_SPEEDS:
                run_moving_servo_test(
                    motor_publisher, servo_publisher, pwm, speed)
    finally:
        publish_motor(motor_publisher, STOP_PWM)
        publish_servo(servo_publisher, START_ANGLE)


if __name__ == "__main__":
    main()

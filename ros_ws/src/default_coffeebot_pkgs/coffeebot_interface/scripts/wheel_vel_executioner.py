#!/usr/bin/env python

import rospy
from utils import *
from nuturtlebot.msg import WheelCommands

TIMEOUT = 0.5   #output PWM will be both set to 0 after 0.5s of idle time
X_VEL_ZERO_TOLERANCE = 0.02
OMEGA_ZERO_TOLERANCE = 0.01
EXECUTION_FREQUENCY = 50
MAX_PWM = 32
MIN_PWM = 10

def almost_equal(a, b, thre):
    return abs(a-b) <= abs(thre)

class WheelVelExecutioner(object):
    def __init__(self):
        general_init()
        motors_init()
        self.wheel_cmd_sub = rospy.Subscriber("/wheel_cmd", WheelCommands, self.wheelCmdCB)
        self.left_PWM = 0
        self.right_PWM = 0

        self.last_execution_time = 0
        while(self.last_execution_time == 0):
            self.last_execution_time = rospy.get_time()

    def __del__(self):
        """Desctructor for cleaning up"""
        motors_cleanup()
        general_cleanup()

    def wheelCmdCB(self, wheel_commands_msg):
        # left_raw_pwm = wheel_commands_msg.left_velocity if abs(wheel_commands_msg.left_velocity) > X_VEL_ZERO_TOLERANCE else 0.0
        # right_raw_pwm = wheel_commands_msg.right_velocity if abs(wheel_commands_msg.right_velocity) > OMEGA_ZERO_TOLERANCE else 0.0
        left_raw_pwm = wheel_commands_msg.left_velocity
        right_raw_pwm = wheel_commands_msg.right_velocity
        self.executePWM(left_raw_pwm, right_raw_pwm)
        self.last_execution_time = rospy.get_time()

    def executePWM(self, left_raw_pwm, right_raw_pwm):
        # +x, forward; -x, backward; x = 0, left or right
        #TODO: safety guard
        if left_raw_pwm > 0:
            if right_raw_pwm > 0:
                forward(left_raw_pwm, right_raw_pwm)
            elif right_raw_pwm == 0:
                half_right(left_raw_pwm, right_raw_pwm)
            else:
                right(left_raw_pwm, -1 * right_raw_pwm)
        elif left_raw_pwm == 0:
            if right_raw_pwm > 0:
                half_left(left_raw_pwm, right_raw_pwm)
            elif right_raw_pwm == 0:
                brake()
            else:
                # there is no function for this... so we have to make an approximation
                right(1, -1 * right_raw_pwm)
        else:
            if right_raw_pwm > 0:
                left(-1 * left_raw_pwm, right_raw_pwm)  
            elif right_raw_pwm == 0:
                # there is no function for this... so we have to make an approximation
                left(-1 * left_raw_pwm, 1)
            else:
                backward(-1 * left_raw_pwm, -1 * right_raw_pwm)


    def timeOut(self):
        time_diff = rospy.get_time() - self.last_execution_time
        return True if time_diff >= TIMEOUT else False

    def brakeNow(self):
        brake()



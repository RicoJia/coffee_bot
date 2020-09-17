#!/usr/bin/env python3
#-*- coding:UTF-8 -*-

import time
from utils import general_init, motors_init, servos_init, LED_init, motors_cleanup, general_cleanup, servos_cleanup
from utils import forward, backward, brake, spin_left, spin_right
from utils import LED_magenta, LED_blue, LED_off
from utils import vertical_servo_control, horizontal_servo_control
from utils import camera_init


# non-blocking keyboard input.
import sys
import select
import tty
import termios

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])



old_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())   #technically, it will be in a try finally block.
try:

    print("************* Hit Ctrl+c to stop the car program. You still need to kill other processes, though. *************")
    print ("forward: w")
    print ("backward: x")
    print ("left: a")
    print ("right: d")
    print ("brake: s")
    print ("speed up (both rotationa dn translation): q")
    print ("slow down (both rotationa dn translation): e")

    print ("Camera up: i")
    print ("Camera down: m")
    print ("Camera left: j")
    print ("Camera right: l")
    print ("Camera right: k")

    print("happy blinks: h")

    print ("Wait for the program to start")

    general_init()
    LED_init()
    motors_init()

    frequency = 10
    max_speed = 32
    min_speed = 10
    pwm_increment = 2
    pwm = 17
    camera_yaw = 110
    camera_pitch = 35
    servos_init(camera_yaw, camera_pitch)

    stop_state = False
    happy_state = False


    camera_init()
    print ("The program has started. Bon Voyage!")
    c = 's'

    while True:

        if isData():
            c = sys.stdin.read(1)
            print(c)

        if c== 's':
            stop_state = True
        elif c == 'h':
            happy_state = not happy_state
            c = ''
        else:           #c == '' or else.
            if c == 'w':
                forward(pwm, pwm)
                stop_state = False
            elif c == 'x':
                backward(pwm, pwm)
                stop_state = False
            elif c == 'a':
                spin_left(pwm, pwm)
                stop_state = False
            elif c == 'd':
                spin_right(pwm, pwm)
                stop_state = False

            elif c == 'q':
                pwm = pwm + pwm_increment if pwm <= max_speed - pwm_increment else pwm
                print ("pwm: ", pwm)
            elif c == 'e':
                pwm = pwm - pwm_increment if pwm >= min_speed + pwm_increment else pwm
                print ("pwm: ", pwm)

            elif c =='i':
                camera_pitch = camera_pitch + 10 if camera_pitch < 170 else camera_pitch
                vertical_servo_control(camera_pitch)
                print("camera pitch: ", camera_pitch, " | camera yaw: ", camera_yaw)

            elif c == 'm':
                camera_pitch = camera_pitch - 10 if camera_pitch > 10 else camera_pitch
                print("camera pitch: ", camera_pitch, " | camera yaw: ", camera_yaw)
                vertical_servo_control(camera_pitch)

            elif c == 'j':
                camera_yaw = camera_yaw + 10 if camera_yaw < 170 else camera_yaw
                print("camera pitch: ", camera_pitch, " | camera yaw: ", camera_yaw)
                horizontal_servo_control(camera_yaw)

            elif c == 'l':
                camera_yaw = camera_yaw - 10 if camera_yaw > 10 else camera_yaw
                print("camera pitch: ", camera_pitch, " | camera yaw: ", camera_yaw)
                horizontal_servo_control(camera_yaw)

            elif c == 'k':
                camera_pitch = 35
                camera_yaw = 110
                print("camera pitch: ", camera_pitch, " | camera yaw: ", camera_yaw)
                horizontal_servo_control(camera_yaw)
                vertical_servo_control(camera_pitch)

            c = ''

        if stop_state == True:
            brake()
            LED_blue()
            time.sleep(0.1)
            if happy_state == True:
                LED_off()
                time.sleep(0.1)
        else:
            LED_magenta()
            time.sleep(1.0/frequency)




except KeyboardInterrupt:
    pass

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)       #you need this line to re-enable keyboard input display on terminal again
motors_cleanup()
general_cleanup()
servos_cleanup()


print("************* Robot control has ended successfully. Wait for a 20 seconds before starting the program again. EnJoy your coffee :) *************")


#!/bin/bash
# This file will copy controls to the robot and execute the file locally on the robot.
    rsync -av --delete /home/ricojia/coffee_bot/controls/ pi@10.0.1.78:Desktop/controls
# ssh pi@10.0.1.78 python3 -u - < /home/ricojia/coffee_bot/controls/teleop.py
#ssh pi@10.0.1.78

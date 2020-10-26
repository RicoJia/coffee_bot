# Coffeebot_Interface

This is the interface node launched on the coffeebot. This package provides: 
1. cmd_vel_executioner, a motion 
executioner for executing /cmd_vel commands with a PD controller.  
2. wheel_command_publisher, a node that counts encoder ticks and publishes wheel speed in rad/s. 

### Usage
You can use the ```coffeebot_interface``` node in a launch file, or separately run it on your Raspberry Pi. 
There is a test script that depends on ``` nuturtle_robot/turtle_interface``` , also it launchs ```rqt_console```
to print messages from the raspberry pi. 

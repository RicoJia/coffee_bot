# The Coffee Bot ROS

###TODO 
1. auto-gmapping, in a custom world, with diff drive
 - copy the auto-gmapping nodes (D)
    1. remove unnecessary files (D)
    2. embed the new world for turtlebot (D)
    3. maje sure mapping is good
 - custom control interface 
    1. come up with list of topics, services. 
        - odom: /gazebo -> /sensor_data -> turtle_interface -> /joint_states 
        (-> /tf -> turtlebot3_slam_gmapping -> /map -> /auto_gmapping) -> ekf_odometer -> /odom 
            -TODO: change frame names to match turtlebot. then test this node alone. (Done) 
        - scan: /gazebo -> /scan? (Done)
        - gazebo_in: /cmd_vel -> turtle_interface -> /wheel_cmd -> /gazebo  (Done)
        - Move_base: /odom, /tf, /scan, /move_base_simple/goal, /map -> move_base -> /cmd_vel
        - turtlebot: /tf, /scan, -> turtlebot -> /map
    2. copy the diff-drive model 
        - change tf: base_link_footprint change it. 
                    
2. amcl and motion planning - using the exisiting map. Tuning
    - change params (keep nuslam)
    - added robot_state publisher
    - Differnt /cmd_vel
    - TODO: need to change turtle_interface and the odom issue!!
        1. check turtlebot3 gazebo plugin, see how /cmd_vel can be transformed there
        2. if no luck, check the slam node. 
        3. check the Slam node, see what sensor info is needed here.  
        
        1. **make gazebo read frequency.** 
         

2. change Gazebo plugin, so it will read pwm properly. 
    - right now: move_base -> turtle_interface -> gazebo_plugin. 
    now, given that the definition of /cmd_vel in move_base is strictly velocity, not "twist", we want: pwm = (speed/max_speed) * max_pwm, 
    then in the plugin, position = (pwm/max_pwm) * max_speed * time. 
    - What you need to change: 
        1. turtle_interface: /cmd_vel call back for pwm, changed it to the non-frequency-related way 
        2. plugin: read max_pwm, timer  (Done), **there's a bug in gazebo plugin about theta_r, theta_l! Also, in the plugin, you're updating the speed, Not the position. Gazebo does not provide position?** 


3. Build IMU node
    - Now we need to add another plugin for IMU. follow tutorial for that
        1. Do we need wheel joint info for /gmapping? no, so you just need IMU data to publish tf in turtle_interface.  
        Add 
            - add the gazebo plugin, 
            - test with gazebo only. - Done, **Gazebo IMU plugin seems to be worse than ROS's IMU plugin!**
    - What we need: 
        2. Incorporate robot_ekf package - needs transform: /map -> /odom -> /base_link. 
            - check if gmapping is printing /map -> /odom? (yep)
            - make sure the robot_pose_ekf prints out /odom -> /base_link.  
            - make sure /base_link -> /scan and /base_link -> /base_footprint tf is statically transformed. and the name matches the scan msg header. (/base_scan)
                But still no updates? embed probes in: imuCB, braodcaster. 
                - k, It needed gps and I added a hack so that it does not need GPS. but still, no imu data?  
                velocity?
                Nope, you need velocity and pose.  
            - Test with keyboard and movevbase. (okay)  
            
                
3. clean up
    - 1. Delete the sensor_msgs in this project (Done, didn't delete it since we might need encoder in the future, but this is gonna be cool!)
    - 
    
###REAL ROBOT STUFF
1. configure IMU, 
    - Download Method 1 (**not recommended**): follow this [example carefully](http://wiki.ros.org/phidgets). 
        1. **Make sure you copy the udev rules to phidget.**
        2. Install sound_play from apt-get 
        3. if you run into issues with opencv2, follow [here](https://answers.ros.org/question/65892/rosmake-error-rospack-error-packagestack-depends-on-non-existent-package-opencv2/)
        4. you also need to download phidgets21 library [here](https://www.phidgets.com/docs/OS_-_Linux)
        5. You may want to delete unncessary packages in CMakeList.txt as they require boost
    - Download Method 2 (**Recommanded**)
        1. come download the IMU driver directly: http://wiki.ros.org/phidgets_imu
        2. configure parameters.  (do not remap topics here)
    - Download imu_madgwick filter. 
        0. test its /tf is good. If so, in launch file disable our own tracker. 
        1. remap topic to from imu_data to /imu_data
        
2. configure Lidar 
    - lidar will be **downward facing** (CW)
    - installation, using SilliconLab CP210x. First, build the driver as shown here http://headstation.com/archives/instructions-installing-cp210x-serial-bridge-driver/
     Then, use a **GOOD USB Cable for connection**.  
    I like to fix my usb connection to ttySLAB0, so I followed [here](https://www.silabs.com/community/interface/knowledge-base.entry.html/2016/06/06/fixed_tty_deviceass-XzTf)
    If there are problems, try adding the VID(vendor), PID(Product ID) yourself: vendor id: 10c4, PID: 80a9, ea60, ea61, ea63, from [this website](http://www.linux-usb.org/usb.ids)
    then follow [here](https://www.raspberrypi.org/forums/viewtopic.php?t=160400)

3. scans are configured CCW or cw? (Now cw)

4. install linux and ubuntu 18.04 server
 - get sd card reader, sd card, raspberry pi 4. If no HDMI during boot up, and the red light is not on, try adjusting the SD card position
 - ssh ricojia@192.168.1.29 
 - Now for some reason, There is very very limited functionalitites in this Ubuntu core 18 server, (no sudo apt, no nothing.)
 - **So I am trying to burn the OS image this again.**
    1.  Download Ubuntu 18.04 from [here](https://ubuntu.com/download/raspberry-pi/thank-you?version=18.04.5&architecture=arm64+raspi4). Then used disk_manager to install it. 
    2. reset ubuntu password: default login name and passwords are both ubuntu.
    3. wifi configuration: 
        go to /etc/netplan, add the wifis part [in this file](https://gist.github.com/dbaldwin/fa1baac11b0ae2f000092b695c3d0b33) to 50-cloud-init.yaml
    4. ssh configuration
        - Set up ssh: 1. delete your old sshkey for the same ip address: ```ssh-keygen -R "you server hostname or ip"```, then just do  ```ssh name@ip```
        - See if you can ssh without ubuntu server being logged in . Yes.
        - change the hostname to "coffeebot@coffeebot001" see [here](https://askubuntu.com/questions/659454/how-to-safely-change-username-and-hostname)
        - make login passwordless: https://www.tecmint.com/ssh-passwordless-login-using-ssh-keygen-in-5-easy-steps/
    5. command search by starting keywords
        
5. Install ROS. [Good video](https://www.youtube.com/watch?v=Irko6xb2qjs)

6. build a /cmd_vel local node. 
    - build minimal node /coffee_bot/cmd_vel_executioner (that takes in /cmd_vel)
        0. Figure out how to build ROS -> GPIO
        1. then source devel/setup.bash in ~/.bashrc
        2. build a pwm publisher on a local ROS node 
            - pinouts are the same for RPi 3B and 4B
            - If you try to use RPi.GPIO but see an error ```RuntimeError: Not running on a RPi!```
                1. make sure you have downloaded rpi.gpio and gpiozero through pip
                2. check you're in the dialout group of ```/dev/gpiomem```, as suggested [here](https://github.com/gpiozero/gpiozero/issues/837)
                to change your user group, do ```$sudo chown username /dev/gpiomem``` 
        3. test the pwm publisher with a simple LED circuit. 
    - build cmd_vel_executioner
        1. build the node, with all directions, 
            - Subscribes to nuturtle msgs (how)
            - Provides a speed limit check, scale it down if overspeed.  
            - why do you need spin left, spin right? (it's just kinda redundant. But your PWM is in [0, 100], so indeed you need to specify wheel rotation)
        2. only keep teleop for testing, then cross compile it. 
        3. launch teleop_keyboard, turtle_interface on your machine. test with LED again
            - does not work. Need cross compilation? 
                1. on multiple machines, you need cross-compilation?
            - Launch on Multi-machine
                1. If you see an error ```RLException: remote roslaunch failed to launch```, follow [this great post](https://answers.ros.org/question/187320/remote-roslaunch-failed-to-launch/)
                 also, change permission using chmod +x
                2. If you see an error ```roslaunch not found```, make sure you source ```devel/setup.bash``` the way in the link above
                3. ```export ROS_MASTER_URI=http://192.168.1.11:11311``` #use hostname -I to check local IP
                4. create a bash file for your local nodes. 
                5. on the turtlebot, ```sudo vim /etc/hosts```, then add ```192.168.something Your_Computer_Hostname``` so the computer can know where your computer is 
                6. **Question** If we start launchfile on computer A, but the launch file runs node1 on computer B. Both computer A and B has a version of node 1, which version of node 1 will be run eventually? 
                    - The computer B version. 
                    - so <output="screen">does not work on remote machines? (No it doesn't. You have to use rqt_console, but I didn't figure out a way to do it yet.)
                
7. Connect IMU to it, run IMU basic node. 
    - Build the phidgets library. 
        1. git clone the https version, I didn't set up ssh for github on that computer
        2. see if you can build the package. Yep
        3. build the imu_madgwick_filter package 
    - test, then change launch file. 
        1. test with turtle_interface, see if you can see imu info. 
        2. if so, change launchfile, test 
    
8. Mount Lidar to it, run with Lidar
    - tune some parameters
    - figure out how to mount it. 
    - create portable udev rules  

10. Final assembly
    - take the circuits apart. 
    - test with the motor circuits        
    
### Question
1. meta-package? nu
2. Why don't I need joint_states for TF? 
3. Do we need to set these in move_base stack? global_costmap/robot_base_frame
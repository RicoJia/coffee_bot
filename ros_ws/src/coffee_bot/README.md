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
        2. Incorporate robot_ekf package
        3. no more odometer? 
    
3. clean up
    - 1. Delete the sensor_msgs in this project
    
4. REAL ROBOT STUFF
    - install linux and ubuntu
        1. get sd card reader, sd card, raspberry pi 4. 
    - Install ROS
    - cross compilation
    - configure IMU, 
    - configure Lidar 
    
### Question
1. meta-package? nu
2. Why don't I need joint_states for TF? 
3. Do we need to set these in move_base stack? global_costmap/robot_base_frame
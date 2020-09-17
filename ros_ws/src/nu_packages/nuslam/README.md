# SLAM for Nuturtle 

This package is the highest level package of the SLAM Simulation of the metapackage Nuturtle_SLAM

### System setup
- Ros Melodic
- Linux Ubuntu 18.04 
- python 2.7 (python 3 above will have trouble launching rqt_plot)

### Usage

1. Set up the project
    ```
    $ mkdir -p /Navigation_Projects/src
    $ cd /Navigation_Projects/src
    $ git clone git@github.com:ME495-Navigation/main-assignment-RicoJia.git
    $ wstool init
    $ cd ..
    $ source devel/setup.bash
    ```
2. To launch the project, with Gazebo: 
``` $ roslaunch nuslam slam.launch robot:=-1 debug:=1```

3. To start the simulation on Gazebo, *
     - Click the triangular start button at the bottom left of the Gazebo console
     - On terminal, **call service**  ```$ rosservice call /real_waypoint/start``` 
     
4. Here are some notes on visualization:
    - The blue cylinders are landmarks, and they are visualized in either /base_link or /map based when reading
from raw landmark observations or filtered landmark positions. 
    - The green cones are waypoints.   
    - The green line is the robot path in /odom
    - The red line is the actual robot path in /map
    - The yellow line is the filtered robot path in /map.  
![Screenshot from 2020-03-23 22-15-45](https://user-images.githubusercontent.com/39393023/77384705-0feb9800-6d54-11ea-8340-158dcb0bef4a.png)

   
### File List
- src/draw_map.cpp    - node for visualizing landmarks
- src/ekf_and_odometer.cpp    - ekf & SLAM node   (in progress, SLAM algorithm needs to be added)
- src/landmarks.cpp   -   Node for return feature matching (In Progress, feature matching is done)
- src/nuslam.cpp      -   Key functions for feature detection
- src/real_world.cpp  -   Provides fake landmark observation  (parameters are in nuslam/config/params.yaml)

- config/params.yaml     -  All parameters of the project
- include/nuslam/nuslam.hpp   -     hpp file for nuslam.cpp 
- launch/control_for_slam.launch    - Sub Launch file for launching control nodes of the project
- launch/slam.launch -  Main launch file for launching the project
- test/landmarks_test.cpp   -   

### Note - Work in Progress: 

1. The actual robot pose from Gazebo is different with different friction parameters and other relevant parameters in [here](../nuturtle_gazebo/urdf/diff_drive.gazebo.xacro)
At the current stage of development, Gazebo wheel positions is synchronized with the wheel position info the EKF gets. However,
it seems that more tuning on the aforementioned parameters is needed so that in a noiseless environment, the EKF filtered output is the same
as the actual robot pose from Gazebo. 

2. Landmark Observation needs to be integrated into the EKF filter.   

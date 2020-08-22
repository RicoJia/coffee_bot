# Coffee Bot 

### Author: Rico Ruotong Jia, Justine Engel

### Introduction
This Project is Rico's birthday present from Justine, his girlfriend. As a base line, they intend to build a robot that can be remotely controlled to deliver coffee in a 12 oz cup. 
Rico and Justine purchased this robot as their mobile platform [Yahboom Raspberry Pi Tank](https://www.amazon.com/gp/product/B07KRVBGQM/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1)


<iframe width="560" height="315" src="https://www.youtube.com/embed/k77LvmTr6P0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

### Overview 
Below are the hardware of the robot. 
- 3D printed cupholder
- Camera (included in Yahboom Robot)
- Raspberry Pi 3B
- Yahboom Raspberry Pi Tank
- BST-4WD servo motor controller 
    
### Usage
1. copy files to the raspberry pi and ssh into it: 
```
$bash run.sh
```
2. 

### Design
The robot design can be decomposed into the following parts: 
- 3D printed cupholder
- Robot & control computer connection 

1. 3D printed cuperholder see [stl file](STL/Cupholder.stl)
2. Robot & control computer connection 

### Raspberry Software Dependency: 
1. mplayer
2. pigpio


### Usage 
1. At the root of the package (~/coffee_bot), copy code from your computer to ```~/Desktop ```pi ```$ bash controls/copy.sh``` 

2. ssh into the pi
```$ssh -X pi@ip_address```

3. run the bash script 
``` bash Desktop/controls/run.sh```
**Note that you might get errors related to pigio if you try to run this bash script right after closing it. That is because in the main program, we invoke pigpio daemon and this will take time to load and close. if you run into trouble, run $sudo killall pigpiod**

4. 5. To see camera livestream, open your browser http://10.0.1.78:8081/  (this is your Pi's local IP address)

[TODO]
    - check Rpi's Ip address
    ``` $ hostname -I```

    - SSH into Rpi's ip address         ```    ssh -X pi@pi_wlan_ip_address      ```    
    note: ssh -X makes the control computer the local host to output images to. 

    - On your laptop, turn on the VLC camera
        ```$vlc sftp://user@host:/path/to/file```
        (note: if you'd like to view an image using [feh](https://taylorhokanson.com/2019/01/31/feh-image-viewer-on-raspberry-pi/), 
        then you need to do ```$export DISPLAY:=0```)
        
3. Alternatively, you can run a bash script to automate this process: 
4. Physical pinouts of the Raspberry Pi can be viewed [here](Media/pinouts.png)

Unsolved problems: 
    1. ssh command for running python properly, especially getch. 
    
    
TODO:
Hardware tasks: 
    1. confirm motor output on both sides; update connection
    2. test two servos. 
    3. test LED
    4. write motor control code. 
    5. write servo control code 
    6. write LED control code.  

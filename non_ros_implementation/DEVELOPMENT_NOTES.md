# Development Notes

### Author Rico Ruotong Jia

##### Camera Notes
1. Raspivid is for raspberry pi camera modules. Since our camera is not a Raspberry Pi module, we do not use it.
2. Pi camera vs external camera. 
    By default, the camera is in /dev/video0. there are different camera driver framework. We are using v4l2 (video for linux two API). To get an output from mplayer, using tv input, output format is yuy2, 
    mplayer tv:// -tv driver=v4l2:device=/dev/video0:width=640:height=440:fps=24:outfmt=yuy2
    1. driver types: V4L (2nd version of realtime video capture driver for linux), V4W(for windows)
    2. output format. [YUV Representation](https://zh.wikipedia.org/wiki/YUV). In short, YUV is an image encoding that reduces bandwidth for chrominance components 
        (U and V are red and blue projections, which make an image colorful). A black-and-white system will be Y (luminance)only. Y' (luma) is gamma corrected Y.  
        Now, YUV system is really used to describe the YCbCr system. 
    3. tv inputs: there are different inputs to mplayer: tv, DVB(a digital tv standard), or radio listening. 

**This works well on the Raspberry pi. However, there is a 2s latency. In order to overcome that, I've decided to use netcat to stream video. This path did not work for me (for some reason, mplayer could not find the remote host through netcat)
Instead, I use adjusted the width and height of mplayer, then latency is gone.**
 
 ```$ mplayer tv:// -tv driver=v4l2:device=/dev/video0:width=320:height=220:fps=24:outfmt=yuy2```

##### Bash Notes 
1. what is background, foreground? 
2. To launch two processes: 1. camera 2. motor control at the same time, I used ```pg &``` in run.sh to run the two processes in background
However, the stdin for python was reading gibberish with an '&' at the end. Then, I took it out to make it run in foreground. This is because foreground processes require user inputs, while background processes do not.         
Also, I used '>/dev/null' on the camera so that its output will be discarded. 
3. I need to run sudo commands, which is associated with a python script. initially, I tried ```$ sudo bash /path/to/bash_script```. However, this messes up mplayer. Instead, an easy way is to invoke the sudo command in 
Python.  

##### Motor Control Notes
1. If motor needs to stop, send pwm=0n to it. Else, the motor will spin at full velocity
2. getch() is a python wrapper of the <conio.h> getch() function. It immediately returns the keyboard output without waiting
for enter. However, it does seem to be waiting for user input. 
3. Servos might jitter. The reason for that is raspberry pi is not a micro-controller - it multitask and the pwm is not stable. 
This is a common known issue. Therefore, use pigio
    - ``` sudo pip3 install pigpio```  
    - To enable pigpio as a daemon,  ``` sudo killall pigpiod```
    - Kill pigpiod when pigpiod is done: ```$sudo killall pigpiod```
4. Right motor seems to have less torque than the left one? Slip in connection?  
    
##### Connection Notes
1. One motor is connected to L-MOB, another is connected to R-MOA
2. Servo motors' brown lines correspond to the black pins on the board. Horizontal servo is connected to J2(the second row of pins close to the board rim), Vertical motor is connected to J3. 
3. My ssh connection was laggy and has lots of freezes... SSH -X is x11 forwarding, which will be slow (what is x11?) turn on ssh -C compression to speed things up. 
  
4. To see the local IP address of a raspberry pi, if its hostname has not been changed, just do ping raspberrypi, then you should be able to see it.   

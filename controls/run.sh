# This file will

#turn on video camera
#mplayer tv:// -tv driver=v4l2:device=/dev/video0:width=320:height=220:fps=5:outfmt=yuy2 >/dev/null &

#mthod 1
    #ssh -C pi@10.0.1.78 ffmpeg -f video4linux2 -s 640x480 -i /dev/video0 -r 10 -b:v 500k -f matroska - | mplayer - -idle -demuxer matroska #works but with a lag

#mothod 2 https://www.raspberrypi.org/documentation/remote-access/ip-address.md



#run the teleop code
python3 Desktop/controls/teleop.py


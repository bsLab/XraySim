#!/bin/bash

c++ -g -DGUI -O2 -m32 -I/opt/opencv/include/opencv4 -c fbp.cpp
g++ -g -DGUI -O2 -m32 -L/usr/lib/gcc/i686-linux-gnu/8  -Wl,-rpath,/opt/opencv/lib -L/opt/opencv/lib -o fbp \
     -lopencv_imgproc -lopencv_video -lopencv_videoio -lopencv_core \
     -lopencv_plot -lopencv_highgui -lopencv_imgcodecs -pthread fbp.o


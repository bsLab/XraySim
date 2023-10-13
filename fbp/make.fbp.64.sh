#!/bin/bash

c++ -g -m64  -O2 -I/opt/opencv/include/opencv4 -c fbp.cpp
g++ -g -m64 -O2 -Wl,-rpath,/opt/opencv64/lib -L/opt/opencv64/lib -o fbp64 \
     -lopencv_imgproc -lopencv_video -lopencv_videoio -lopencv_core \
     -lopencv_imgcodecs -pthread fbp.o


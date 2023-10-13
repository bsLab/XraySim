#!/bin/bash

c++ -g -DGUI -O2 -I/opt/opencv/include/opencv4 -c fbp.cpp
c++ -g -DGUI -O2 -Wl,-rpath,/opt/opencv/lib -L/opt/opencv/lib -o fbp \
     -lopencv_imgproc -lopencv_video -lopencv_videoio -lopencv_core \
     -lopencv_plot -lopencv_highgui -lopencv_imgcodecs fbp.o


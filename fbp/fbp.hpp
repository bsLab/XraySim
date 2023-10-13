/**
    fbp.hpp
    Filtered Back Projection Algorithm

    MIT License
    Copyright (c) 2019 Fran Piernas Diaz
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

    Created using OpenCV 4.0.0

    @author Fran Piernas Diaz (fran.piernas@gmail.com), Stefan Bosse (sbosse@uni-bremen.de)
    @version 1.2

    TODO: Refactor code 
    
    Based on Peter Toft: "The Radon Transform - Theory and Implementation",
    Ph.D. thesis. Department of Mathematical Modelling, Technical University of Denmark, June 1996.



    ************CHANGELOG************
    V1.1:
        Changed the way the point cloud is saved. Values are now floating point normalized
        from 0 to 255, instead of 0 to 255 but integer, as in V1.0.



    Usage to reconstruct a single sinogram, use this code:

        Mat sinogram=imread("sinogram.jpg",IMREAD_COLOR); //load the sinogram image "sinogram.jpg"
        Mat filtered_sinogram=filter_sinogram(sinogram); //filter the sinogram
        Mat reconstruction=iradon(filtered_sinogram,false); //perform back projection. Change false to true if sinogram is a full turn.
        renormalize255_frame(reconstruction); //normalize to 255
        imwrite("Reconstruction.jpg",reconstruction); //save reconstruction

    Usage to reconstuct a CT scan from a video, call this function:

        fbp(video,
            extra_frames,
            pointcloud_threshold,
            normalizeByframe,
            full_turn)

        where:  (string) video is the name of the CT video.
                (int) extra_frames is the number of interpolated frames between pair of frames. Try 1 and
                    increase if your results are noisy due to very few images available.
                (int) pointcloud_threshold is a 0-255 number wich will remove all points whose intensity
                    is less than this number when saving the point cloud.
                (bool) normalizeByframe, if true, all slices will be normalized to the maximum value
                    of that slice. If false, the object is normalized to its absolute maximum.
                (bool) full_turn, if true, the code assumes that the input CT video is a full turn.
                    Set it false if the input is half a turn.

        Follow any instruction on the terminal

    Usage to reconstuct a CT scan from a vector<Mat>, call this function:

        fbp(frames,
            extra_frames,
            pointcloud_threshold,
            normalizeByframe,
            full_turn)

        where frames is a vector<Mat> containing all pictures from 0 to 360 (or 180) degrees. All other
        parameters are the same as explained above.

        Follow any instruction on the terminal

    Special thanks to Nicolas De Francesco (nevermore78@gmail.com) for the help on image processing.
*/

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <fstream>
#include <cmath>
#include <sys/time.h>
#include <thread>

#define K_PLUS 43
#define K_MINUS 45
#define K_ENTER 13
using namespace cv;
using namespace std;

static float FrameMinVal=0.0,
             FrameMaxVal=0.0, 
             FrameScale=1.0,
             FrameRange=255.0,
             Gain=1;

long int millitime() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  return ms;
}

std::string GetMatDepth(const cv::Mat& mat)
{
    const int depth = mat.depth();

    switch (depth)
    {
    case CV_8U:  return "CV_8U";
    case CV_8S:  return "CV_8S";
    case CV_16U: return "CV_16U";
    case CV_16S: return "CV_16S";
    case CV_32S: return "CV_32S";
    case CV_32F: return "CV_32F";
    case CV_64F: return "CV_64F";
    default:
        return "Invalid depth type of matrix!";
    }
}

std::string GetMatType(const cv::Mat& mat)
{
    const int mtype = mat.type();

    switch (mtype)
    {
    case CV_8UC1:  return "CV_8UC1";
    case CV_8UC2:  return "CV_8UC2";
    case CV_8UC3:  return "CV_8UC3";
    case CV_8UC4:  return "CV_8UC4";

    case CV_8SC1:  return "CV_8SC1";
    case CV_8SC2:  return "CV_8SC2";
    case CV_8SC3:  return "CV_8SC3";
    case CV_8SC4:  return "CV_8SC4";

    case CV_16UC1: return "CV_16UC1";
    case CV_16UC2: return "CV_16UC2";
    case CV_16UC3: return "CV_16UC3";
    case CV_16UC4: return "CV_16UC4";

    case CV_16SC1: return "CV_16SC1";
    case CV_16SC2: return "CV_16SC2";
    case CV_16SC3: return "CV_16SC3";
    case CV_16SC4: return "CV_16SC4";

    case CV_32SC1: return "CV_32SC1";
    case CV_32SC2: return "CV_32SC2";
    case CV_32SC3: return "CV_32SC3";
    case CV_32SC4: return "CV_32SC4";

    case CV_32FC1: return "CV_32FC1";
    case CV_32FC2: return "CV_32FC2";
    case CV_32FC3: return "CV_32FC3";
    case CV_32FC4: return "CV_32FC4";

    case CV_64FC1: return "CV_64FC1";
    case CV_64FC2: return "CV_64FC2";
    case CV_64FC3: return "CV_64FC3";
    case CV_64FC4: return "CV_64FC4";

    default:
        return "Invalid type of matrix!";
    }
}

void FrameMinMax(const cv::Mat& frame,float *min, float  *max) {
   const int mtype = frame.type();
   int f,c,k;
   switch (mtype) {
      case CV_8UC3:  
      case CV_8SC3:  
      case CV_16UC3: 
      case CV_16SC3: 
      case CV_32SC3: 
      case CV_32FC3: 
      case CV_64FC3:
        for(f=0;f<frame.size().height;f++)
        {
            for(c=0;c<frame.size().width;c++)
            {
                if (c==0 && f==0) { *min=*max=frame.at<Vec3f>(f,c)[0]; };
                for(k=0;k<3;k++) {
                  if (frame.at<Vec3f>(f,c)[k]<*min) *min=frame.at<Vec3f>(f,c)[k];
                  if (frame.at<Vec3f>(f,c)[k]>*max) *max=frame.at<Vec3f>(f,c)[k];
                }
            }
        }
        break;
      default:
        for(f=0;f<frame.size().height;f++)
        {
            for(c=0;c<frame.size().width;c++)
            {
                if (c==0 && f==0) { *min=*max=frame.at<float>(f,c); };
                if (frame.at<float>(f,c)<*min) *min=frame.at<float>(f,c);
                if (frame.at<float>(f,c)>*max) *max=frame.at<float>(f,c);
            }
        }      
    }
    // printf("%f %f\n",*min,*max);
    return;

}

void FramesMinMax(vector<Mat>& frames,float *_min, float  *_max) {
  float _min1,_max1;
  unsigned int i;
  for(i=0;i<frames.size();i++) {
    FrameMinMax(frames.at(i), &_min1,&_max1);
    if (i==0) { *_min=_min1;*_max=_max1; }
    else      { *_min=min(_min1,*_min);*_max=max(_max1,*_max); } 
  }
  return;  
}

void gamma_correction_frame(Mat& frame, double gamma)
{
    unsigned int f,c;
    const int mtype = frame.type();
    switch (mtype) {
      case CV_8UC3:  
      case CV_8SC3:  
      case CV_16UC3: 
      case CV_16SC3: 
      case CV_32SC3: 
      case CV_32FC3: 
      case CV_64FC3: 
        for(f=0;f<frame.size().height;f++)
        {
            for(c=0;c<frame.size().width;c++)
            {
                frame.at<Vec3f>(f,c)[0]=pow(frame.at<Vec3f>(f,c)[0],gamma);
                frame.at<Vec3f>(f,c)[1]=pow(frame.at<Vec3f>(f,c)[1],gamma);
                frame.at<Vec3f>(f,c)[2]=pow(frame.at<Vec3f>(f,c)[2],gamma);
            }
        }
        break;
      default:
        for(f=0;f<frame.size().height;f++)
        {
            for(c=0;c<frame.size().width;c++)
            {
                frame.at<float>(f,c)=pow(frame.at<float>(f,c),gamma);
            }
        }      
    }
    return;
}

// TODO: should be processed multi-threaded...

void gamma_correction_frames(vector<Mat>& frames, double gamma)
{
    unsigned int i;
    for(i=0;i<frames.size();i++) gamma_correction_frame(frames.at(i), gamma);
    return;
}
/*
   y
   ^
   |          [f1] [f2] [] .. -> z
   |
   +--->x    

*/
// Rotate around x-axis (positive 90°) XYZ -> XZY
// k: new slice z index (old y-axis)
// x: unchanged
// y: old z-axis
Mat rotate_slice_x(vector<Mat>& slices, int w,int h, int k, int cropx, int cropy) {
  // Mat (int rows, int cols, int type)
  Mat slice(h,w,CV_32FC1);
  int i,j,
      X0=slices.at(0).size().width,
      Y0=slices.at(0).size().height,
      Z0=slices.size();
  // printf("Rotate slice %d (%d x %d) from [%d x %d x %d]\n",k,w,h,X0,Y0,Z0);
  for(j=0;j<Z0;j++) {
    for(i=0;i<X0;i++) {
      // .at(row,column) order
      // printf("%d %d\n",i,j);
      slice.at<float>(j,i) = slices.at(j).at<float>(k,i);
    }
  }
  return slice;
}

// Rotation of slice volume (x:1 positive 90° XYZ -> XZY)
vector<Mat> rotate_slices(vector<Mat>& slices, int rx, int ry, int rz, int cropx, int cropy)
{
    unsigned int i,j,k,
             X0=slices.at(0).size().width,
             Y0=slices.at(0).size().height,
             Z0=slices.size();
    vector<Mat> slicesR;
    switch (rx*100+ry*10+rz) {
      case 100: {
        int XR=X0,
            YR=Z0,
            ZR=Y0;
        printf("Rotating slice stack XYZ -> XZY...\n");
        for(k=0;k<ZR;k++) {
          slicesR.push_back(rotate_slice_x(slices,XR,YR,k,cropx,cropy));
        }
        break;
      }
      default:
        printf("Unsupported rotation %d %d %d\n",rx,ry,rz);
    }
    return slicesR;
}

vector<Mat> video_to_array(string video)
{
    printf("Reading video %s...\n",video.c_str());
    VideoCapture video_input(video.c_str());
    Mat frame;
    vector<Mat> input_array;
    int frameIndex=0;
    do
    {
        // printf("reading frame %d\n",frameIndex++);
        video_input>>frame;
        if(frame.empty()) break;
        input_array.push_back(frame.clone());
    } while(!frame.empty());
    printf("Got %d frames.\n",input_array.size());
    printf("Frame size %d x %d pixels x %d (%s) Type %s\n",
      input_array.at(0).size().width,input_array.at(0).size().height,input_array.at(0).depth(),
      GetMatDepth(input_array.at(0)).c_str(),
      GetMatType(input_array.at(0)).c_str());
    return input_array;
}

vector<Mat> imgstack_to_array(string imgfile)
{
    int i;
    vector<Mat> input_array;
    printf("Reading image stack %s...\n",imgfile.c_str());
    imreadmulti(imgfile,input_array,IMREAD_ANYDEPTH);
    printf("Got %d frames.\n",input_array.size());
    printf("Frame size %d x %d pixels x %d (%s) Type %s\n",
      input_array.at(0).size().width,input_array.at(0).size().height,input_array.at(0).depth(),
      GetMatDepth(input_array.at(0)).c_str(),
      GetMatType(input_array.at(0)).c_str());
    double minVal; 
    double maxVal; 
    for(i=0;i<input_array.size();i++) {
      Point minLoc; 
      Point maxLoc;
      double _minVal; 
      double _maxVal; 
      minMaxLoc( input_array.at(i), &_minVal, &_maxVal, &minLoc, &maxLoc );
      if (i==0) { 
        minVal=_minVal; maxVal=_maxVal; 
      } else {
        if (_minVal<minVal) minVal=_minVal;
        if (_maxVal>maxVal) maxVal=_maxVal;
      }
    }
    printf("Image Stack Minimum=%f Maximum=%f\n",(float)minVal,(float)maxVal);
    FrameMinVal=(float)minVal;
    FrameMaxVal=(float)maxVal;
    if (maxVal>255 && maxVal < 4096) {
      // 12 Bit
      FrameScale=16;
      printf("Assuming 12 Bit Frame data => Applying scaling with factor 16\n");
      for(i=0;i<input_array.size();i++) {
        input_array.at(i) *= 16;
      }
    }
    if (maxVal>255) {
      FrameRange=65535;    
    }
    return input_array;
}

Mat convert_frame2U8(Mat& frame, float scale)
{
    Mat converted(frame.size().height,frame.size().width,CV_8UC1);
    frame.convertTo(converted,CV_8UC1,scale);
    return converted;
}


unsigned int begin_frame(vector<Mat>& frame_array)
{
#ifdef GUI
    namedWindow("Select begin frame of CT.",WINDOW_NORMAL);
    cout<<"Select begin frame of CT. Navigate through frames with + or - and press enter to select."<<endl;
    unsigned int frame_index=0, key_pressed;
    do
    {
        imshow("Select begin frame of CT.",frame_array.at(frame_index));
        key_pressed=waitKey(0);
        if(key_pressed==K_PLUS&&frame_index<frame_array.size()-1) frame_index++;
        if(key_pressed==K_MINUS&&frame_index>0) frame_index--;
        cout<<"Moving to frame: "<<frame_index+1<<"/"<<frame_array.size()<<endl;
        if(key_pressed==K_ENTER) break;

    } while(true);
    destroyWindow("Select begin frame of CT.");
    return frame_index;
#else
    return 0;
#endif
}

unsigned int end_frame(vector<Mat>& frame_array, unsigned int initial_frame)
{
#ifdef GUI
    namedWindow("Select end frame of CT.",WINDOW_NORMAL);
    cout<<"Select end frame of CT. Navigate through frames with + or - and press enter to select."<<endl;
    unsigned int frame_index=initial_frame, key_pressed;
    do
    {
        imshow("Select end frame of CT.",frame_array.at(frame_index));
        key_pressed=waitKey(0);
        if(key_pressed==K_PLUS&&frame_index<frame_array.size()-1) frame_index++;
        if(key_pressed==K_MINUS&&frame_index>initial_frame) frame_index--;
        cout<<"Moving to frame: "<<frame_index+1<<"/"<<frame_array.size()<<endl;
        if(key_pressed==K_ENTER) break;
    } while(true);
    destroyWindow("Select end frame of CT.");
    return frame_index;
#else
    return 0;
#endif
}

void cut_area(vector<Mat>& frames,Rect2d r)
{
    if (!r.empty()) {
      int i=0;
      printf("Cropping BBOX [x y w h] %4.0f %4.0f %4.0f %4.0f\n",r.x,r.y,r.width,r.height);
      for(i=0;i<frames.size();i++) frames.at(i)=frames.at(i)(r);
      return;
    }   
#ifdef GUI
    cout<<"Select the object. Align the grid to the axis of rotation."<<endl;
    r=selectROI("Select the object. Align the grid to the axis of rotation.",frames.at(0));
    printf("Cropping BBOX [x y w h] %4.0f %4.0f %4.0f %4.0f\n",r.x,r.y,r.width,r.height);
    destroyAllWindows();
    if(r.empty()) return;
    int i=0;
    for(i=0;i<frames.size();i++) frames.at(i)=frames.at(i)(r);
#endif
    return;
}

Mat make_sinogram(vector<Mat> frames, unsigned int height)
{
    Mat sinogram(frames.at(0).size().width,frames.size(),frames.at(0).type());
    int frame_number=0, j;
    for(frame_number=0;frame_number<frames.size();frame_number++)
    {
        for(j=0;j<frames.at(0).size().width;j++)
        {
            sinogram.at<float>(j,frame_number)=frames.at(frame_number).at<float>(height,j);
        }
    }
    return sinogram;
}

vector<Mat> make_sinograms(vector<Mat>& frames)
{
    unsigned int i;
    vector<Mat> sinograms;
    for(i=0;i<frames.at(0).size().height;i++) sinograms.push_back(make_sinogram(frames,i).clone());
    return sinograms;
}

void remove_frames(vector<Mat>& frames, unsigned int A, unsigned int B)
{
    if(B<frames.size())frames.erase(frames.begin()+B+1, frames.begin()+frames.size());
    if(A>0)frames.erase(frames.begin(), frames.begin()+A);
    return;
}



void convert_frame2RGB8(Mat& frame)
{
    frame.convertTo(frame,CV_8UC3,1);
    cvtColor(frame,frame,COLOR_GRAY2RGB);
    return;
}

void convert_frames2RGB8(vector<Mat>& frames)
{
    unsigned int i;
    for(i=0;i<frames.size();i++) convert_frame2RGB8(frames.at(i));
    return;
}

// TODO TBC
void convert_frame2GRAY(Mat& frame)
{
    frame.convertTo(frame,CV_8UC1,1);
    // cvtColor(frame,frame,COLOR_GRAY2RGB);
    return;
}

void convert_frames2GRAY(vector<Mat>& frames)
{
    unsigned int i;
    for(i=0;i<frames.size();i++) convert_frame2GRAY(frames.at(i));
    return;
}
void convert_frame2U16(Mat& frame)
{

  frame.convertTo(frame,CV_16UC1,256);
  return;
}

void convert_frames2U16(vector<Mat>& frames)
{
  unsigned int i;
  for(i=0;i<frames.size();i++) {
    double minVal; 
    double maxVal; 
    Point minLoc; 
    Point maxLoc;

    minMaxLoc( frames.at(i), &minVal, &maxVal, &minLoc, &maxLoc );

    printf("[%d] minVal %f maxVal %f\n",i,minVal,maxVal);
    convert_frame2U16(frames.at(i));
  }
  return;
}
void renormalize255_frame(Mat& frame)
{
    unsigned int f,c;
    float maxm=0;
    for(f=0;f<frame.size().height;f++)
    {
        for(c=0;c<frame.size().width;c++)
        {
            if(frame.at<float>(f,c)>maxm)maxm=frame.at<float>(f,c);
        }
    }

    for(f=0;f<frame.size().height;f++)
    {
        for(c=0;c<frame.size().width;c++)
        {
            frame.at<float>(f,c)=frame.at<float>(f,c)*255.0/maxm;
        }
    }
    return;
}

void renormalize255_frame(Mat& frame, float maxm)
{
    unsigned int f,c;
    for(f=0;f<frame.size().height;f++)
    {
        for(c=0;c<frame.size().width;c++)
        {
            frame.at<float>(f,c)=frame.at<float>(f,c)*255.0/maxm;
        }
    }
    return;
}

void renormalize255_frame_by_frame(vector<Mat>& frames)
{
    unsigned int i;
    for(i=0;i<frames.size();i++) renormalize255_frame(frames.at(i));
    return;
}

void renormalize255_frames(vector<Mat>& frames)
{
    unsigned int f,c,i;
    float maxm=0;
    for(i=0;i<frames.size();i++)
    {
        for(f=0;f<frames.at(i).size().height;f++)
        {
            for(c=0;c<frames.at(i).size().width;c++)
            {
                if(frames.at(i).at<float>(f,c)>maxm)maxm=frames.at(i).at<float>(f,c);
            }
        }
    }
    printf("renormalize255_frames maxVal=%f\n",maxm);
    for(i=0;i<frames.size();i++)renormalize255_frame(frames.at(i),maxm);
    return;
}

void convert_frame2bw(Mat& frame)
{
  unsigned int f,c;
  Mat converted(frame.size().height,frame.size().width,CV_32FC1);
  for(f=0;f<frame.size().height;f++)
  {
      for(c=0;c<frame.size().width;c++)
      {
          converted.at<float>(f,c)=1.0/3.0*(frame.at<Vec3f>(f,c)[0]+frame.at<Vec3f>(f,c)[1]+frame.at<Vec3f>(f,c)[2]);
          //converted.at<float>(f,c)=frame.at<Vec3f>(f,c)[1];
      }
  }
  frame=converted.clone();
  return;
}


void convert_frames2bw(vector<Mat>& frames)
{
  unsigned int i;
  const int mtype = frames.at(0).type();
  printf("New size %d x %d\n",frames.at(0).size().width,frames.at(0).size().height);
    switch (mtype) {
      case CV_8UC3:  
      case CV_8SC3:  
      case CV_16UC3: 
      case CV_16SC3: 
      case CV_32SC3: 
      case CV_32FC3: 
      case CV_64FC3: 
        for(i=0;i<frames.size();i++) {
          convert_frame2bw(frames.at(i));
        }
        break;
     default:
      printf("Nothing to do.\n");
  }  
  return;
}

void convert_frame2f(Mat& frame)
{
  const int mtype = frame.type();
  switch (mtype) {
    case CV_8UC3:  
    case CV_8SC3:  
    case CV_16UC3: 
    case CV_16SC3: 
    case CV_32SC3: 
    case CV_32FC3: 
    case CV_64FC3: 
      frame.convertTo(frame,CV_32FC3,Gain/FrameRange);
      break;
    default:
      frame.convertTo(frame,CV_32FC1,Gain/FrameRange);
  }
  return;
}

void convert_frames2f(vector<Mat>& frames)
{
    unsigned int i;
    for(i=0;i<frames.size();i++)convert_frame2f(frames.at(i));
    return;
}

Mat filter_sinogram(Mat& sinogram)
{
    Mat filtered_sinogram;
    transpose(sinogram,filtered_sinogram);
    if(filtered_sinogram.type()==CV_8UC3) //Convert to gray scale and 32bit if the input is a 8bit RGB image
    {
        cout<<"Converting to 32bit grayscale..."<<endl;
        convert_frame2f(filtered_sinogram);
        convert_frame2bw(filtered_sinogram);
    }
    Mat dft_sinogram[2]={filtered_sinogram,Mat::zeros(filtered_sinogram.size(),CV_32F)};
    Mat dftReady;
    merge(dft_sinogram,2,dftReady);
    dft(dftReady,dftReady,DFT_ROWS|DFT_COMPLEX_OUTPUT,0);
    split(dftReady,dft_sinogram);
    unsigned int f,c;
    for(f=0;f<dft_sinogram[0].size().height;f++)
    {
        for(c=0;c<dft_sinogram[0].size().width;c++)
        {
            //Sine Filter
            dft_sinogram[0].at<float>(f,c)*=(1.0/(2.0*M_PI))*1.0*abs(sin(1.0*M_PI*(c)/dft_sinogram[0].size().width));
            dft_sinogram[1].at<float>(f,c)*=(1.0/(2.0*M_PI))*1.0*abs(sin(1.0*M_PI*(c)/dft_sinogram[0].size().width));
        }
    }
    merge(dft_sinogram,2,dftReady);
    dft(dftReady,filtered_sinogram,DFT_INVERSE|DFT_ROWS|DFT_REAL_OUTPUT,0);
    transpose(filtered_sinogram,filtered_sinogram);
    return filtered_sinogram;
}

void filter_all_sinograms(vector<Mat>& sinograms)
{
    unsigned int i;
    for(i=0;i<sinograms.size();i++) sinograms.at(i)=filter_sinogram(sinograms.at(i)).clone();
    return;
}

Mat iradon(Mat& sinogram, bool full_turn) //Sinogram must be a 32bit single channel grayscale image normalized 0-1
{
    Mat reconstruction(sinogram.size().height,sinogram.size().height,CV_32FC1);
    float delta_t;
    if(full_turn) delta_t=2.0*M_PI/sinogram.size().width;
    else delta_t=1.0*M_PI/sinogram.size().width;
    unsigned int t,f,c,rho;
    for(f=0;f<reconstruction.size().height;f++)
    {
        for(c=0;c<reconstruction.size().width;c++)
        {
            reconstruction.at<float>(f,c)=0;
            for(t=0;t<sinogram.size().width;t++)
            {
                rho=((f-0.5*sinogram.size().height)*cos(delta_t*t)+(c-0.5*sinogram.size().height)*sin(delta_t*t)+0.5*sinogram.size().height);
                if((rho>0)&&(rho<sinogram.size().height)) reconstruction.at<float>(f,c)+=sinogram.at<float>(rho,t);
            }
            if(reconstruction.at<float>(f,c)<0)reconstruction.at<float>(f,c)=0;
        }
    }
    rotate(reconstruction,reconstruction,ROTATE_90_CLOCKWISE);
    return reconstruction;
}

/*
Parallelization of the most time consuming process of computing the inverse radon transformation (the final fbp reconstruction)
using slice worker threads 
*/

#define MAX_THREADS 24

static vector<Mat> workerSlices (MAX_THREADS);

void processIRadonSlice (int thread_id, int sliceIndex, Mat& sinogramSlice, bool full_turn) {
  // printf("Worker thread %d started for slice %d...\n",thread_id, sliceIndex);
  workerSlices[thread_id]=iradon(sinogramSlice,full_turn).clone();
}

vector<Mat> video_iradon(vector<Mat>& sinograms, bool full_turn, int numthreads)
{
    vector<Mat> slices;
    unsigned int i, t, prev_perc, tn;
    long int last=millitime();
    long int start=last;
    long int avg=0;
    vector<std::thread> threads(MAX_THREADS);
    if (numthreads>0) {
      for(i=0;i<sinograms.size();)
      {
        tn=std::min(numthreads,(int)(sinograms.size()-i));
        for(t=0;t<tn;t++) {
          std::thread th(processIRadonSlice, t, i, std::ref(sinograms.at(i)), full_turn);
          threads[t]=move(th);
          i++;
        }
        for(t=0;t<tn;t++) {
          threads[t].join();
        }
        // printf("adding slices %d\n",i);
        for(t=0;t<tn;t++) {
          slices.push_back(workerSlices[t]);
        }
        if((int)(100.0*i/sinograms.size()-prev_perc)>0) {
          long int now=millitime(),
                   delta=now-last;
          if(avg==0) avg=delta;
          else avg=(avg+delta)/2;
          float current=100.0*i/sinograms.size(),
                remain=100-current,
                scale = current-prev_perc;
          printf("%d%% (%d s / %d min remaining)\n",
              (int)current,
              (int)(remain*avg/(1000*scale)),
              (int)(remain*avg/(1000*60*scale))
          );
          last=now;
        }
        prev_perc=100.0*i/sinograms.size();
      }
    } else {
      for(i=0;i<sinograms.size();i++)
      {
          slices.push_back(iradon(sinograms.at(i),full_turn).clone());
          if((int)(100.0*i/sinograms.size()-prev_perc)>0) {
            long int now=millitime(),
                     delta=now-last;
            if(avg==0) avg=delta;
            else avg=(avg+delta)/2;
            float current=100.0*i/sinograms.size(),
                  remain=100-current;
            printf("%d%% (%d s / %d min remaining)\n",
                (int)current,
                (int)(remain*avg/(1000)),
                (int)(remain*avg/(1000*60))
            );
            last=now;
          }
          prev_perc=100.0*i/sinograms.size();
      }   
    }
    long int now=millitime();
    printf("Total back projection time: %d s (%d min) for %d slices\n",
           (now-start)/1000,
           (now-start)/1000/60,
           slices.size());
    return slices;
}


float frames_get_bg_intensity(Mat& frame, Mat& display_image)
{
  float I0=0.0;
#ifdef GUI
  cout<<"Select a background area."<<endl;
  Rect2d r=selectROI("Select a background area.",display_image);
  printf("Background BBOX [x y w h]: %4.0f %4.0f %4.0f %4.0f\n",r.x,r.y,r.width,r.height);
  destroyAllWindows();
  if(r.empty())
  {
      cout<<"No ROi selected. Background intensity set to 1 (use auto background setting)"<<endl;
      return 1.0;
  }
  Mat selected=frame(r);
  unsigned int f,c;
  for(f=0;f<selected.size().height;f++)
  {
      for(c=0;c<selected.size().width;c++)
      {
          I0+=selected.at<float>(f,c);
      }
  }
  I0*=1.0/(selected.size().width*selected.size().height);
  cout<<"Background intensity set to "<<I0<<endl;
#endif
  return I0;
}

float frames_get_bg_intensity_roi(Mat& frame,Rect2d r)
{
  float I0=0.0;

  printf("Background BBOX [x y w h]: %4.0f %4.0f %4.0f %4.0f\n",r.x,r.y,r.width,r.height);

  Mat selected=frame(r);
  unsigned int f,c;
  for(f=0;f<selected.size().height;f++)
  {
      for(c=0;c<selected.size().width;c++)
      {
          I0+=selected.at<float>(f,c);
      }
  }
  I0*=1.0/(selected.size().width*selected.size().height);


  cout<<"Background intensity set to "<<I0<<endl;

  return I0;
}

float frames_get_bg_intensity_auto(vector<Mat>& frames)
{
  float I0=-1E12;
  unsigned int i,f,c;
  for(i=0;i<frames.size();i++)
  {
    Mat& frame=frames.at(i);
    for(f=0;f<frame.size().height;f++)
    {
        for(c=0;c<frame.size().width;c++)
        {
            I0=max(I0,frame.at<float>(f,c));
        }
    }
  }

  cout<<"Background intensity set to "<<I0<<endl;

  return I0;
}

void frames_get_transmitance(vector<Mat>& frames, float I0)
{
    unsigned int i,f,c;
    float _min=1E9,_max=-1E9;
    for(i=0;i<frames.size();i++)
    {
        for(f=0;f<frames.at(i).size().height;f++)
        {
            for(c=0;c<frames.at(i).size().width;c++)
            {
                if(frames.at(i).at<float>(f,c)<I0&&frames.at(i).at<float>(f,c)>0.0)
                  frames.at(i).at<float>(f,c)=-1.0*log10(frames.at(i).at<float>(f,c)/I0);
                else if(frames.at(i).at<float>(f,c)>I0) 
                  frames.at(i).at<float>(f,c)=0.0;
                else if(frames.at(i).at<float>(f,c)==0.0) 
                  frames.at(i).at<float>(f,c)=-1.0*log10((1.0/255.0)/I0);
                _min=min(_min,frames.at(i).at<float>(f,c));
                _max=max(_max,frames.at(i).at<float>(f,c));
            }
        }
    }
    printf("New transmittance min=%f max=%f I0=%f\n",_min,_max,I0);
    return;
}

Mat create_interpolation_img(Mat& img1, Mat& img2, unsigned int extra_frames, unsigned int extra_frame_number) //expected CV_32FC1 images
{
    unsigned int i,f,c;
    float slope=0;
    Mat interpolated_frame(img1.size().height,img1.size().width,img1.type());
    for(f=0;f<img1.size().height;f++)
    {
        for(c=0;c<img1.size().width;c++)
        {
            slope=(1.0*(img2.at<float>(f,c)-img1.at<float>(f,c)))/(extra_frames+1);
            interpolated_frame.at<float>(f,c)=1.0*extra_frame_number*slope+img1.at<float>(f,c);
        }
    }
    return interpolated_frame;
}

vector<Mat> interpolate(vector<Mat>& frames, unsigned int extra_frames)
{
    unsigned int i, j;
    vector<Mat> interpolated;
    for(i=0;i<frames.size();i++)
    {
        interpolated.push_back(frames.at(i).clone());
        for(j=1;j<=extra_frames;j++)
        {
            if(i!=frames.size()-1)interpolated.push_back(create_interpolation_img(frames.at(i),frames.at(i+1),extra_frames,j).clone());
        }
    }
    return interpolated;
}

void save_video(vector<Mat>& frames, string output)
{
  printf("Saving scan video in file %s (%d x %d x %d slice frames)..\n",output.c_str(),frames.at(0).size().width,frames.at(0).size().height,frames.size());
  VideoWriter video_reconstruction(output.c_str(),VideoWriter::fourcc('M','J','P','G'),25,frames.at(0).size(),true);
  unsigned int i;
  for(i=0;i<frames.size();i++) video_reconstruction.write(frames.at(i));
  video_reconstruction.release();
  return;
}

void save_scan(vector<Mat>& slices, unsigned int i_threshold, string output)
{
  int i;
  vector<int> compression_params;
  compression_params.push_back(IMWRITE_TIFF_COMPRESSION);
  compression_params.push_back(5);

  if (output.empty()) output="slices.tif";
  if (output.find(".tif")!=std::string::npos) {
    printf("Saving slice stack in file %s (%d x %d x %d slices)..\n",output.c_str(),slices.at(0).size().width,slices.at(0).size().height,slices.size());
    imwrite(output,slices,compression_params);
  } else {
    // save single slice files
    printf("Saving slice stack in single files %sNNNN.tif (%d x %d x %d slices)..\n",output.c_str(),slices.at(0).size().width,slices.at(0).size().height,slices.size());
    for(i=0;i<slices.size();i++) {
      char path[1000];
      sprintf(path,"%s%04d.tif",output.c_str(),i);
      imwrite(path,slices.at(i),compression_params);
    }
  }
}

// Apply circular/elliptical mask (radius == width/2 .. of scan image)
void mask_scan(vector<Mat>& slices, int margin)
{
  int i,
      width  = slices.at(0).size().width,
      height = slices.at(0).size().height;
  printf("Performing elliptical masking of slice images (%d x %d x %d) with margin %d...\n",width,height,slices.size(),margin);
  Mat mask(height, width, CV_32FC1, Scalar(0));
  ellipse(mask, Point(width/2,height/2 ), Size(width/2-margin,height/2-margin), 0, 0, 360, Scalar(1), FILLED, 8 );
  for(i=0;i<slices.size();i++) {
    Mat& slice = slices.at(i);
    Mat res;
    res = slice.mul(mask);
    slice=res;
  }
}

#if 0
void fbp(vector<Mat>& frames, unsigned int extra_frames, unsigned int pointcloud_threshold, bool normalizeByframe, bool full_turn, bool auto_size)
{
    Rect2d crop;
    vector<Mat> sinograms, slices;
    cut_area(frames,crop);
    Mat display_image=frames.at(0).clone();
    convert_frames2f(frames);
    gamma_correction_frames(frames,2.2);
    convert_frames2bw(frames);
    if(extra_frames>0)
    {
        cout<<"Performing interpolation..."<<endl;
        frames=interpolate(frames,extra_frames);
        cout<<"Number of frames: "<<frames.size()<<endl;
    }
    frames_get_transmitance(frames,frames_get_bg_intensity(frames.at(0),display_image));

    cout<<"Building sinograms..."<<endl;
    sinograms=make_sinograms(frames);

    cout<<"Filtering sinograms..."<<endl;
    filter_all_sinograms(sinograms);

    cout<<"Performing Filtered Back Projection..."<<endl;
    slices=video_iradon(sinograms,full_turn);

    cout<<"Normalizing..."<<endl;
    if(!normalizeByframe)renormalize255_frames(slices);
    else renormalize255_frame_by_frame(slices);

    cout<<"Saving point cloud..."<<endl;
    save_scan(slices,pointcloud_threshold,"slices.tiff");

    cout<<"Converting to RGB..."<<endl;
    convert_frames2RGB8(slices);

    cout<<"Saving video..."<<endl;
    save_video(slices,"slices.avi");

    cout<<"Finished."<<endl;
    return;
}
#endif

void fbp(string input, string output,
         unsigned int extra_frames, unsigned int pointcloud_threshold, 
         bool normalizeByframe, bool full_turn, bool auto_size, bool auto_background,
         bool preprocessOnly, double gammaVal, float backgroundVal, float gainVal,
         int A_frame, int B_frame,
         Rect2d crop, Rect2d bbackground,
         int mask_margin,
         int numthreads,
         int rotX,
         int rotY,
         int rotZ)
{
    float _min,_max;
    // std::thread t1(ProcessFrame);
    Gain=gainVal;
    printf("Gamma=%f Gain=%f Background=%f\n",gammaVal,gainVal,backgroundVal);
    vector<Mat> frames, sinograms, slices;
    if (input.find(".avi")!=std::string::npos || input.find(".mp4")!=std::string::npos)
      frames=video_to_array(input.c_str());
    else
      frames=imgstack_to_array(input.c_str());
    if (A_frame!=B_frame) {
      printf("Selecting frame range %d .. %d\n",A_frame,B_frame);
      remove_frames(frames,A_frame,B_frame);    
    } else if (auto_size==true) {
      A_frame=0;
      B_frame=frames.size()-1;
      printf("Using full frame range %d .. %d\n",A_frame,B_frame);
    } else { 
      A_frame=begin_frame(frames);
      B_frame=end_frame(frames,A_frame);
      printf("Selecting frame range %d .. %d\n",A_frame,B_frame);
      remove_frames(frames,A_frame,B_frame);
    }
    printf("Number of frames=%d Start=%d End=%d Rot=0..%d°\n",
           frames.size(),A_frame,B_frame,full_turn?360:180);
    if (!auto_size || crop.width!=0) {
      printf("Cropping images ...\n");
      cut_area(frames,crop); 
    } else {
      printf("Using full image size.\n");
    }
    
    Mat display_image=frames.at(0).clone();

    printf("Converting frames...\n");
    convert_frames2f(frames);

    printf("Applying gamma correction %f...\n",gammaVal);
    gamma_correction_frames(frames,gammaVal);

    printf("Converting frames to Float32 Monochrome images (CV_32FC1)...\n");
    convert_frames2bw(frames);

    FramesMinMax(frames,&_min,&_max);

    printf("CV_32FC1 min=%f max=%f\n",_min,_max);

    if(extra_frames>0)
    {
        printf("Performing interpolation...\n");
        frames=interpolate(frames,extra_frames);
        printf("Number of total frames: %d\n",frames.size());
    }
    printf("Extracting background intensity...\n");
    
    if (!bbackground.empty()) {
      frames_get_transmitance(frames,frames_get_bg_intensity_roi(frames.at(0),bbackground));
    } if (auto_background) {
      frames_get_transmitance(frames,frames_get_bg_intensity_auto(frames));
    } if (backgroundVal!=0.0) {
      backgroundVal=pow(backgroundVal*FrameScale*Gain/FrameRange,gammaVal);
      //printf("Using background value %f (scaled by %f, gamma=%f)\n",background,FrameScale,gammaVal);
      printf("Using corrected background value %f\n",backgroundVal);
      frames_get_transmitance(frames,backgroundVal);
    } else {
      frames_get_transmitance(frames,frames_get_bg_intensity(frames.at(0),display_image));
    }
    if (preprocessOnly) return;

    printf("Building sinograms...\n");
    sinograms=make_sinograms(frames);

    printf("Filtering sinograms...\n");
    filter_all_sinograms(sinograms);

    printf("Performing Filtered Back Projection...\n");
    slices=video_iradon(sinograms,full_turn,numthreads);

    printf("Normalizing...\n");
    if(!normalizeByframe) renormalize255_frames(slices);
    else renormalize255_frame_by_frame(slices);

    printf("Masking...\n");
    if (mask_margin) mask_scan(slices, mask_margin);

    // TODO cropping
    if (rotX || rotY || rotZ) {
      slices=rotate_slices(slices, rotX, rotY, rotZ, 0 ,0);
    }
    
    printf("Converting to GRAY...\n");
    // convert_frames2RGB8(slices);
    convert_frames2GRAY(slices);
    printf("Saving image stack... (GRAY)\n");
    // convert_frames2U16(slices);
    save_scan(slices,pointcloud_threshold,output);

    // cout<<"Saving video..."<<endl;
    // save_video(slices,"slices.avi");

    printf("Finished.\n");
    return;
}






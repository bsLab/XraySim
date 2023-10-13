#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>

#include <string>
#include <iostream>
#include <exception>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <math.h>

#include "SimpleGVXR.h"

#include <sys/time.h>
#include <thread>

#include <tiffio.h>

// for synthetic noise 
#include <random>
std::mt19937 generator;
double mean = 0.0;
double stddev  = 1.0;
std::normal_distribution<double> normal(mean, stddev);

void usage () {
  printf("usage: xraysim [-s(ourcePos) x y z] [-d(etPos) x y z] [-D(detUpVec) x y z]\n");
  printf("               [-e(nergy) MeV] [-S(caleIntensity) min max range] [-n addgaussnoiseperc]\n");
  printf("               [-r(otate-z) degree] [-R(otate) x y z] [-ct(-rotate-z) delta degree]\n");
  printf("               [-w pixels]  [-h pixels] [-p pixelsize] [-g openglwinsize]\n");
  printf("               [-show(_scene)] [-soft(ware_cpu_renderer)] [-o out[%%04d].tif/pgm]\n");
  printf("               [-u density g/cm3] file.stl [-u density file2.stl]\n");
}

long int millitime() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  return ms;
}

int savePGM(std::string output,std::vector<std::vector<float>> image,
            float scaleMin, float scaleMax, float scaleRange){
  int iwidth=image.at(0).size(),
      iheight=image.size();
  // Save the image into a image file
  printf("Saving Xray Image to file %s (Scale min=%f max=%f range=%f)\n",
         output.c_str(),scaleMin,scaleMax,scaleRange);
  // normalize to X bits / scaleRange
  FILE *fd = fopen(output.c_str(),"w+");
  fprintf(fd,"P2\n");
  fprintf(fd,"%d %d\n",image.at(0).size(),image.size());
  fprintf(fd,"%d\n",(int)scaleRange);
  int k=0,length=image.size()*image.at(0).size();
  for(int i=0;i<image.size();i++) {
    for(int j=0;j<image.at(0).size();j++) {
      float v=(image.at(i).at(j)-scaleMin)/(scaleMax-scaleMin);
      fprintf(fd,"%d",int(v*scaleRange));
      k++;
      if ((k%16)==0) fprintf(fd,"\n");
      else if (k<length) fprintf(fd," ");
    }
  }
  fclose(fd);
  return 0;
}

int saveTIFF(std::string output,std::vector<std::vector<float>> image,
             float scaleMin, float scaleMax){
  float scaleRange=65535;
  int iwidth=image.at(0).size(),
      iheight=image.size();
  printf("Saving Xray Image to file %s (Scale min=%f max=%f range=%f)\n",
         output.c_str(),scaleMin,scaleMax,scaleRange);
  // Write 16 bit tiff
  // Open the TIFF file
  TIFF *output_image;
  if((output_image = TIFFOpen(output.c_str(), "w")) == NULL){
    std::cerr << "Unable to write tif file: " << output << std::endl;
  }
  TIFFSetField(output_image, TIFFTAG_IMAGEWIDTH, iwidth);
  TIFFSetField(output_image, TIFFTAG_IMAGELENGTH, iheight);
  TIFFSetField(output_image, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(output_image, TIFFTAG_BITSPERSAMPLE, 16);
  TIFFSetField(output_image, TIFFTAG_ROWSPERSTRIP, iheight);
  TIFFSetField(output_image, TIFFTAG_ORIENTATION, (int)ORIENTATION_TOPLEFT);
  TIFFSetField(output_image, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(output_image, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
  TIFFSetField(output_image, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);

  tsize_t image_s;
  short int *data=(short int*)malloc(iwidth*iheight*2);
  int k=0;
  for(int i=0;i<image.size();i++) {
    for(int j=0;j<image.at(0).size();j++) {
      float v=(image.at(i).at(j)-scaleMin)/(scaleMax-scaleMin);
      data[k]=int(v*scaleRange);
      k++;
    }
  }

  if((image_s = TIFFWriteEncodedStrip(output_image, 0, (tdata_t) data, iwidth*iheight*2)) == -1)
  {
        std::cerr << "Unable to write tif file: " << output << std::endl;
  }
   
  TIFFWriteDirectory(output_image);
  TIFFClose(output_image);
  return 0;
}

// usage: xraysim [-s x y z] [-e MeV] [-d x y z] [-w pixels] [-h pixels] [-p pixelsize] file.stl
int main(int argc, char* argv[])
{
    int i;
    

    bool showScene=0;
    bool softRenderer=0;
    float sourcePosX=-40.0;
    float sourcePosY=0.0;
    float sourcePosZ=0.0;
    float detPosX=10.0;
    float detPosY=0.0;
    float detPosZ=0.0;
    float detUpX=0.0;
    float detUpY=0.0;
    float detUpZ=-1.0;
    float hu[10] = {1000.0,1000.0,1000.0,1000.0,1000.0,
                    1000.0,1000.0,1000.0,1000.0,1000.0};
    float density[10] = {1.0,1.0,1.0,1.0,1.0,
                         1.0,1.0,1.0,1.0,1.0};
    
    float energy  = 0.08;
    int width     = 640;
    int height    = 320;
    int glwinsize = 500;
    
    float pixelSize = 0.5;
    
    float angleX   = 0.0;
    float angleY   = 0.0;
    float angleZ   = 0.0;
    
    float angleCT  = 0.0;
    
    float scaleK    = 0.0;
    float scaleRange = 65535;
    float scaleMin  = 0.0;
    float scaleMax  = 0.0;
    
    float noise = 0.0;
    
    int parts   = 0;
    int frameIndex  = 0;
    int frames  = 1;
    
    std::string file[10];
    std::string partLabel[10]={"A","B","C","D","E","F","G","H","I","J"};
    std::string output="xray.tif";
    
    setenv("DISPLAY",":0.0",1);
    
    if (argc<2) { usage(); return EXIT_FAILURE; }
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "-o") {
        if (i + 1 < argc) {
          i++;
          output = argv[i];
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-u") {
        if (i + 1 < argc) { 
          i++;
          density[parts] = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-e") {
        if (i + 1 < argc) { 
          i++;
          energy = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-ct") {
        if (i + 1 < argc) { 
          i++;
          angleCT = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-S") {
        if (i + 3 < argc) { 
          i++;
          scaleMin = std::stof(argv[i++]);
          scaleMax = std::stof(argv[i++]);
          scaleRange = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-r") {
        if (i + 1 < argc) { 
          i++;
          angleZ = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-p") {
        if (i + 1 < argc) { 
          i++;
          pixelSize = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-w") {
        if (i + 1 < argc) { 
          i++;
          width = std::stoi(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-h") {
        if (i + 1 < argc) { 
          i++;
          height = std::stoi(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-g") {
        if (i + 1 < argc) { 
          i++;
          glwinsize = std::stoi(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-n") {
        if (i + 1 < argc) { 
          i++;
          noise = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-s") {
        if (i + 3 < argc) { 
          i++;
          sourcePosX = std::stof(argv[i++]);
          sourcePosY = std::stof(argv[i++]);
          sourcePosZ = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-d") {
        if (i + 3 < argc) { 
          i++;
          detPosX = std::stof(argv[i++]);
          detPosY = std::stof(argv[i++]);
          detPosZ = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-D") {
        if (i + 3 < argc) { 
          i++;
          detUpX = std::stof(argv[i++]);
          detUpY = std::stof(argv[i++]);
          detUpZ = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-R") {
        if (i + 3 < argc) { 
          i++;
          angleX = std::stof(argv[i++]);
          angleY = std::stof(argv[i++]);
          angleZ = std::stof(argv[i]);
          continue;
        } else { usage(); return EXIT_FAILURE; }
      } 
      if (std::string(argv[i]) == "-show") {
        showScene=1;
        continue;
      } 
      if (std::string(argv[i]) == "-soft") {
        softRenderer=1;
        continue;
      } 
      if (argv[i][0]!='-') {
        file[parts++]=argv[i];
      }
    }
    try
    {
        if (softRenderer) setenv("LIBGL_ALWAYS_SOFTWARE","1",1);
        // Print the libraries' version
        std::cout << getVersionOfSimpleGVXR() << std::endl;
        std::cout << getVersionOfCoreGVXR()   << std::endl;

        printf("Creating OpenGL Context and Window\n");
        // Create an OpenGL context
        // createOpenGLContext(-1,3,2);
        // createWindow(-1,1,"VULKAN");
        createWindow();
        
        printf("Setting OpenGL Context Window size %d x %d \n",glwinsize,glwinsize);
        setWindowSize(glwinsize, glwinsize);

        // Set the position of the X-ray source
        printf("Setting source position %f x %f x %f [cm]\n",sourcePosX,  sourcePosY, sourcePosZ);
        setSourcePosition(sourcePosX,  sourcePosY, sourcePosZ, "cm");

        // Set the shape of the X-ray source (here an infinitely small point)
        usePointSource();

        // The spectrum of the X-ray beam
        // (here a monochromatic spectrum)
        printf("Setting monochromatic energy %f MeV\n",energy);
        setMonoChromatic(energy, "MeV", 1000);

        // Set the position of the X-ray detector (film/cassette)
        printf("Setting detector position %f x %f x %f [cm]\n",detPosX, detPosY, detPosZ);
        setDetectorPosition(detPosX, detPosY, detPosZ, "cm");

        // Set the orientation of the X-ray detector (film/cassette)
        printf("Setting detector up vector %f x %f x %f\n",detUpX, detUpY, detUpZ);
        setDetectorUpVector(detUpX, detUpY, detUpZ);

        // Set the number of pixel of the X-ray detector
        printf("Setting detector pixels %d x %d [%f mm]\n",width,height,pixelSize);
        setDetectorNumberOfPixels(width, height);

        // Set the distance between two successive pixels
        setDetectorPixelSize(pixelSize, pixelSize, "mm");

        printf("Reading polygon mesh files...\n");
        // Load a polygon mesh from a STL file as a scenegraph
 //       loadSceneGraph(file, "mm");
        for(int i=0;i<parts;i++) {
          printf("[%d] %s (%s) <= %f (g/cm3)\n",i,file[i].c_str(),partLabel[i].c_str(),density[i]);
          loadMeshFile(partLabel[i], file[i], "mm");
          // moveToCentre(partLabel[i]);
          setHU(partLabel[i], 1000);
          setDensity(partLabel[i], density[i], "g/cm3");
        }
        // Set the material properties of all the nodes within the scenegraph
//        for (unsigned int i = 0; i < getNumberOfChildren("root"); ++i)
//        {
//            std::string label = getChildLabel("root", i);
//            printf("Setting material property for %s <= %f (hu)\n",label.c_str(),hu);
//            moveToCentre(label);
//            setHU(label, hu);
//        }


/*
        loadSceneGraph("second.stl", "mm");

        for (unsigned int i = 0; i < getNumberOfChildren("root"); ++i)
        {
            std::string label = getChildLabel("root", i);
            printf("Setting material property for %s <= %f (hu)\n",label.c_str(),hu);
            moveToCentre(label);
            setHU(label, hu);
        }
*/
        if (angleX!=0.0 || angleY!=0 || angleZ!=0) {
          printf("Rotating scene by %f %f %f\n",angleX,angleY,angleZ);
          if (angleX!=0.0) rotateScene(angleX, 1.0, 0.0, 0.0);
          if (angleY!=0.0) rotateScene(angleY, 0.0, 1.0, 0.0);
          if (angleZ!=0.0) rotateScene(angleZ, 0.0, 0.0, 1.0);
        }

        if (angleCT!=0.0) frames=(int)((float)360.0/angleCT);
        int autoScale=0;
        if (scaleMin==0.0 && scaleMax==0.0) autoScale=1;
        
        for(frameIndex=0;frameIndex<frames;frameIndex++) {
          char path[1000];
          snprintf(path, sizeof(path), output.c_str(), frameIndex);
          std::string pathStr = path;
          
          printf("[%d] Computing Xray Image ...\n",frameIndex);
          // Compute an X-ray image(planar projection/radiograph)
          
          long int start=millitime();
          computeXRayImage();
          long int stop=millitime();

          printf("[%d] Computing Xray Image took %d ms...\n",frameIndex,stop-start);
          
          std::vector<std::vector<float>> image=getLastXRayImage();
          
          if (noise!=0.0) {
            // add relative gaussian noise (rough approximation of xray noise)
            for(int i=0;i<image.size();i++) {
              for(int j=0;j<image.at(0).size();j++) {
                float v = image.at(i).at(j),
                      n = normal(generator)*noise,
                      vn = v+abs(v)*n;
                image.at(i).at(j)=vn;
              }                            
            }
          }
          int iwidth  = image.at(0).size(),
              iheight = image.size();
          if (autoScale) {
            // auto-scale
            scaleMin=scaleMax=image.at(0).at(0);
            for(int i=0;i<image.size();i++) {
              for(int j=0;j<image.at(0).size();j++) {
                float v=image.at(i).at(j);
                scaleMin=std::min(scaleMin,v);
                scaleMax=std::max(scaleMax,v);
              }
            }
          }
              
          float imin=1E30,imax=-1E30;
          for(int i=0;i<image.size();i++) {
            for(int j=0;j<image.at(0).size();j++) {
              imin=std::min(imin,image.at(i).at(j));
              imax=std::max(imax,image.at(i).at(j));
            }
          }
          printf("[%d] Xray Image: %d x %d pixels [%f,%f]\n",frameIndex,image.at(0).size(),image.size(),imin,imax);
          
          if (path[strlen(path)-1]=='f') {
            saveTIFF(pathStr,image,scaleMin,scaleMax);
          }
          if (path[strlen(path)-1]=='m') {
            savePGM(pathStr,image,scaleMin,scaleMax,scaleRange);
          }
          if (angleCT!=0.0) {
            printf("[%d] Rotating scene by 0 0 %f\n",frameIndex,angleCT);
            rotateScene(angleCT, 0.0, 0.0, 1.0);
          }
        }
        // saveLastSinogram();
        // saveLastProjectionSet();
        // saveLastLBuffer();
    }
    catch (const std::exception& err)
    {
        std::cerr << "FATAL ERROR:\t" << err.what() << std::endl;
        return EXIT_FAILURE;
    }

    if (showScene) {
      displayScene();
      std::cout << "press any key to exit...";
      std::cin.get();
    }
        
    return EXIT_SUCCESS;
}

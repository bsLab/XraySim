#include "fbp.hpp"

void usage () {
  printf("usage: fbp [-e(xtra_frames) #] [-f(ull_turn)] [-g(amma) #] [-G(ain) #]\n");
  printf("           [-a(uto_size_and_range)] [-n(ormalizeByframe)] [-ba(ackground_auto)]\n");
  printf("           [-bv(ackground:value) #] [-bb(ackground) x y w h] \n");
  printf("           [-c(rop) x y w h] [-r firstframe last]\n");
  printf("           [-p(reprocess)] [-t(hreads) #]\n");
  printf("           [-m(ask_margin_pixel) #]\n");
  printf("           [-R(otate_additional) x y z] [-C(rop_before_rotate) x y w h]\n");
  printf("           [-o(utput) dir|file.tif(f)]\n");  
  printf("           scan.[avi mp4 tif tiff]\n");
}

int main(int argc, char* argv[]) {
  bool full_turn=false,
       auto_size=false,
       auto_background=false,
       preprocessOnly=false,
       normalizeByframe=false;
  double gammaVal=2.2,gainVal=1.0;
  float  background=0.0;
  
  string input,output;
  int pointcloud_threshold=0,
      extra_frames=0,
      first=0,
      last=0,
      mask_margin=0,
      threads=0,
      rotX=0,
      rotY=0,
      rotZ=0;
      
  Rect2d crop,bbackground;
  
  if (argc<2) { usage(); return EXIT_FAILURE; }
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "-n") {
      normalizeByframe=true;
    }
    if (std::string(argv[i]) == "-f") {
      full_turn=true;
    }
    if (std::string(argv[i]) == "-a") {
      auto_size=true;
    }
    if (std::string(argv[i]) == "-ba") {
      auto_background=true;
    }
    if (std::string(argv[i]) == "-p") {
      preprocessOnly=true;
    }
    if (std::string(argv[i]) == "-e") {
      if (i + 1 < argc) { 
        i++;
        extra_frames = std::stoi(argv[i]);
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (std::string(argv[i]) == "-m") {
      if (i + 1 < argc) { 
        i++;
        mask_margin = std::stoi(argv[i]);
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (std::string(argv[i]) == "-t") {
      if (i + 1 < argc) { 
        i++;
        threads = std::stoi(argv[i]);
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (std::string(argv[i]) == "-r") {
      if (i + 2 < argc) { 
        i++;
        first = std::stoi(argv[i++]);
        last = std::stoi(argv[i]);
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (std::string(argv[i]) == "-c") {
      if (i + 4 < argc) { 
        i++;
        crop.x = std::stof(argv[i++]);
        crop.y = std::stof(argv[i++]);
        crop.width  = std::stof(argv[i++]);
        crop.height = std::stof(argv[i]);
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (std::string(argv[i]) == "-R") {
      if (i + 3 < argc) { 
        i++;
        rotX = std::stoi(argv[i++]);
        rotY = std::stoi(argv[i++]);
        rotZ = std::stoi(argv[i]);
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (std::string(argv[i]) == "-bb") {
      if (i + 2 < argc) { 
        i++;
        bbackground.x = std::stof(argv[i++]);
        bbackground.y = std::stof(argv[i++]);
        bbackground.width  = std::stof(argv[i++]);
        bbackground.height = std::stof(argv[i]);
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (std::string(argv[i]) == "-bv") {
      if (i + 1 < argc) { 
        i++;
        background = (double)std::stof(argv[i]);
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (std::string(argv[i]) == "-g") {
      if (i + 1 < argc) { 
        i++;
        gammaVal = (double)std::stof(argv[i]);
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (std::string(argv[i]) == "-G") {
      if (i + 1 < argc) { 
        i++;
        gainVal = (double)std::stof(argv[i]);
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (std::string(argv[i]) == "-o") {
      if (i + 1 < argc) { 
        i++;
        output = argv[i];
        continue;
      } else { usage(); return EXIT_FAILURE; }
    } 
    if (argv[i][0]!='-') {
      input=argv[i];
    }
  }
  
  fbp(input,
      output,
      extra_frames,
      pointcloud_threshold,
      normalizeByframe,
      full_turn,
      auto_size,
      auto_background,
      preprocessOnly,
      gammaVal,
      background,
      gainVal,
      first,
      last,
      crop,
      bbackground,
      mask_margin,
      threads,
      rotX,
      rotY,
      rotZ);
  return 0;
}

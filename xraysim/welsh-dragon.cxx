#include <cstdlib>

#include <string>
#include <iostream>
#include <exception>

#include "SimpleGVXR.h"

int main()
{
    try
    {
        // Print the libraries' version
        std::cout << getVersionOfSimpleGVXR() << std::endl;
        std::cout << getVersionOfCoreGVXR()   << std::endl;

        // Create an OpenGL context
        createWindow();
        setWindowSize(512, 512);

        // Set the position of the X-ray source
        setSourcePosition(-40.0,  0.0, 0.0, "cm");

        // Set the shape of the X-ray source (here an infinitely small point)
        usePointSource();

        // The spectrum of the X-ray beam
        // (here a monochromatic spectrum)
        setMonoChromatic(0.08, "MeV", 1000);

        // Set the position of the X-ray detector (film/cassette)
        setDetectorPosition(10.0, 0.0, 0.0, "cm");

        // Set the orientation of the X-ray detector (film/cassette)
        setDetectorUpVector(0, 0, -1);

        // Set the number of pixel of the X-ray detector
        setDetectorNumberOfPixels(640, 320);

        // Set the distance between two successive pixels
        setDetectorPixelSize(0.5, 0.5, "mm");

        // Load a polygon mesh from a STL file as a scenegraph
        loadSceneGraph("welsh-dragon-small.stl", "mm");

        // Set the material properties of all the nodes within the scenegraph
        for (unsigned int i = 0; i < getNumberOfChildren("root"); ++i)
        {
            std::string label = getChildLabel("root", i);
            moveToCentre(label);
            setHU(label, 1000);
        }

        // Compute an X-ray image(planar projection/radiograph)
        computeXRayImage();

        // Save the image into a image file
        saveLastXRayImage("welsh-dragon-xray_cpp.tif");
    }
    catch (const std::exception& err)
    {
        std::cerr << "FATAL ERROR:\t" << err.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

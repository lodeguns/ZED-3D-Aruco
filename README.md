# ZED-3D-Aruco

# Stereolabs ZED - Depth Sensing + OpenCV ARUCO markers detection.

This sample captures a 3D point cloud and display it in an OpenGL window, intercepting ARUCO markers

It shows how to:
- Get a 3D point cloud with the API.
- Display point cloud in OpenGL.
- Use a thread to capture the point cloud and update the GL window simultaneously.
- Open a parallel OpenCV window detecting ARUCO markers see [OpenCV ARUCO Markers](https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html).

Note the markers are created through the [Online ArUco markers generator](http://chev.me/arucogen/)

To retrieve a depth map of the scene, see [Depth Sensing](https://github.com/stereolabs/zed-examples/tree/master/tutorials) tutorial.

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com).
- For more information, read the ZED [API documentation](https://www.stereolabs.com/developers/documentation/API/).

### Prerequisites

- Windows 7 64bits or later, Ubuntu 16.04
- [ZED SDK](https://www.stereolabs.com/developers/) and its dependencies ([CUDA](https://developer.nvidia.com/cuda-downloads))

## Build the program

#### Build for Windows

- Create a "build" folder in the source folder
- Open cmake-gui and select the source and build folders
- Generate the Visual Studio `Win64` solution
- Open the resulting solution and change configuration to `Release`
- Build solution

#### Build for Linux

Open a terminal in the sample directory and execute the following command:

    mkdir build
    cd build
    cmake ..
    make

## Run the program

- Navigate to the build directory and launch the executable file
- Or open a terminal in the build directory and run the sample :

        ./ZED_Depth_Sensing


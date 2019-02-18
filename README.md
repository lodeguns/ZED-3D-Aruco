# ZED-3D-Aruco

### Stereolabs ZED - Depth Sensing + OpenCV ARUCO markers detection +  GLViewer click handler.

This sample captures a 3D point cloud and display it in an OpenGL window, intercepting ARUCO markers

It shows how to:
- Get a 3D point cloud with the API.
- Display point cloud in OpenGL.
- Use a thread to capture the point cloud and update the GL window simultaneously.
- Open a parallel OpenCV window detecting ARUCO markers see [OpenCV ARUCO Markers](https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html).
- GLViewer is modified in order to obtain a pixel click handler.

Note the markers are created through the [Online ArUco markers generator](http://chev.me/arucogen/)

To retrieve a depth map of the scene, see [Depth Sensing](https://github.com/stereolabs/zed-examples/tree/master/tutorials) tutorial.

### Here a working example on 3 AruCo Markers
![alt text](https://raw.githubusercontent.com/lodeguns/ZED-3D-Aruco/master/arucom.png)


### Code Mod. 
For those impatient, the origial ZED SDK run() function is modified in this way:
```
void run() {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    printf("OpenCV: %s", cv::getBuildInformation().c_str());
    char key = ' ';
    while (key != 'q') {
        if (zed.grab() == SUCCESS) {
            // Retrieve a colored RGBA point cloud in GPU memory and update GL viewing window
            // width and height specify the total number of columns and rows for the point cloud dataset
            // Retrieve the IDs of printed ArUco markers.
            // Get 3D coordinates by clicking on the GL Viewer.

            // superficie
            zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_GPU, width, height);
            viewer.updatePointCloud(point_cloud);

           
            //std::cout << " Click handler: " << std::endl;
            //std::cout << viewer.getClickedPoint().x << std::endl;
            //std::cout << viewer.getClickedPoint().y << std::endl;

            //Surface
            //Standard cod: MAT_TYPE_8U_C4
            Mat image_zed(width, height, MAT_TYPE_8U_C4);
            Mat depth_image_zed(width, height, MAT_TYPE_8U_C4);

            // sl::Mat to cv::Mat conversion
            cv::Mat image_ocv = slMat2cvMat(image_zed);
            cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);

            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, width, height);
            zed.retrieveImage(depth_image_zed, VIEW_DEPTH, MEM_CPU, width, height);


           // Display image and depth using cv:Mat which share sl:Mat data
           //cv::imshow("Image", image_ocv);
           // cv::imshow("Depth", depth_image_ocv);


           //OpenCV conv.
           cv::Mat image_ocv_2, imageCopy;
           cv::Mat mask;
           image_ocv.convertTo(image_ocv, CV_8UC4);
           cv::cvtColor(image_ocv, mask, CV_BGR2GRAY);


           mask.copyTo(imageCopy);
           std::vector<int> ids;
           std::vector<std::vector<cv::Point2f> > corners;
           cv::aruco::detectMarkers(mask, dictionary, corners, ids);
              // if at least one marker detected
           std::vector<int> v1 = { 3, 2, 1 };

           if (ids.size() > 0)
           {        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
                   if (v1 != ids) {
                       std::cout << "Visible markes:" << std::endl;
                       for (auto i : ids) std::cout << i << std::endl;
                   }
             }



            cv::imshow("Image", imageCopy);



            //Handle key event
            key = cv::waitKey(10);
            processKeyEvent(zed, key);


        } else sl::sleep_ms(1);
    }
}

```


## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com).
- For more information, read the ZED [API documentation](https://www.stereolabs.com/developers/documentation/API/).

### Prerequisites

- Ubuntu 16.04
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

## Lincense
The SDK example is implemented by ZED and follow its lincence.
The modifications are developed in the NeuRoNe Lab - University of Salerno, IT
and supported by the ValueBiotech - Milano, IT

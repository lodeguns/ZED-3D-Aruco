///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/*************************************************************************
 ** This sample demonstrates how to capture images and 3D point cloud   **
 ** with the ZED SDK and display the result in an OpenGL window. 		    **
 *************************************************************************/

 // Standard includes
#include <stdio.h>
#include <string.h>
#include <vector>

// OpenCV
#include <opencv2/aruco.hpp>

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "GLViewer.hpp"
#include <SaveDepth.hpp>

// Using std and sl namespaces
using namespace std;
using namespace sl;

// Create ZED objects (camera, callback, images)
sl::Camera zed;
sl::Mat point_cloud;
std::thread zed_callback;
cv::Mat slMat2cvMat(Mat& input);

Mat image_zed;
Mat depth_image_zed;
cv::Mat depth_image_ocv;
cv::Mat image_ocv;


void printHelp();


int width, height;
bool quit;



// Point cloud viewer
GLViewer viewer;

// Sample functions
void startZED();
void run();
void close();








int main(int argc, char **argv) {

    // Set configuration parameters for the ZED
    InitParameters initParameters;
    if (argc == 2) initParameters.svo_input_filename = argv[1];
    initParameters.camera_resolution = RESOLUTION_HD2K;
    initParameters.depth_mode = DEPTH_MODE_QUALITY;
    initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    initParameters.coordinate_units = UNIT_CENTIMETER;

    // Open the camera
    ERROR_CODE err = zed.open(initParameters);
    if (err != SUCCESS) {
        cout << toString(err) << endl;
        zed.close();
        viewer.exit();
        return 1; // Quit if an error occurred
    }

    // Initialize point cloud viewer in half-size
    width = (int) zed.getResolution().width / 2;
    height = (int) zed.getResolution().height / 2;
    viewer.init(width, height);




    // Start the camera thread
    startZED();

    // Set the display callback
    glutCloseFunc(close);
    glutMainLoop();
    return 0;
}


cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}

/**
    Launch ZED thread. Using a thread here allows to capture a point cloud and update the GL window concurrently.
 **/
void startZED() {
    quit = false;
    zed_callback = std::thread(run);
}



/**
    This function loops to get image and motion data from the ZED. It is similar to a callback.
    Add your own code here.
 **/
void run() {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    printf("OpenCV: %s", cv::getBuildInformation().c_str());
    char key = ' ';
    while (key != 'q') {
   // while (!quit) {
        if (zed.grab() == SUCCESS) {
            // Retrieve a colored RGBA point cloud in GPU memory and update GL viewing window
            // width and height specify the total number of columns and rows for the point cloud dataset
            // In this example, we retrieve and display a half size point cloud using width and height parameters

            // superficie
            zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_GPU, width, height);
            viewer.updatePointCloud(point_cloud);


            //Segui handler nella classe GLViewer, questa funzione va in loop qui.

            //Nota la struct è definita dentro GLViewer.h
            //std::cout << " Click handler 2 " << std::endl;
            //std::cout << viewer.getClickedPoint().x << std::endl;
            //std::cout << viewer.getClickedPoint().y << std::endl;

            //Superfice
            //Nota standard è MAT_TYPE_8U_C4.. ma ho messo MAT_TYPE_8U_C4
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
             //not need to sort since it already sorted

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


            //old piece of code.
/*           // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
            // Only the headers and pointer to the sl::Mat are copied, not the data itself


            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, width, height);
            zed.retrieveImage(depth_image_zed, VIEW_DEPTH, MEM_CPU, width, height);

            // Retrieve the RGBA point cloud in half-resolution
            // To learn how to manipulate and display point clouds, see Depth Sensing sample
            zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_CPU, width, height);
            viewer.updatePointCloud(point_cloud);
            // Display image and depth using cv:Mat which share sl:Mat data
            //cv::imshow("Image", image_ocv);
            //cv::imshow("Depth", depth_image_ocv);

            //Handle key event
            key = cv::waitKey(10);
            processKeyEvent(zed, key);


*/






        } else sl::sleep_ms(1);
    }
}

/**
    This function closes the ZED camera, its callback (thread) and the GL viewer
 **/
void close() {
    quit = true;

    // Stop callback
    zed_callback.join();

    // Exit point cloud viewer
    viewer.exit();

    // Free buffer and close the ZED
    point_cloud.free(MEM_GPU);
    zed.close();
}

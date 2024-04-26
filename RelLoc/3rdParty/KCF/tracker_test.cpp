#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

#include <dirent.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>

using namespace std;
using namespace cv;

/*
int main( int argc, char** argv ){
    // declares all required variables
    cv::Rect2d roi;
    // create a tracker object
    cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();
    // set input video

    // Frame readed
    cv::Mat frame;

    // Read Images
    std::ifstream listFramesFile;
    std::string listFrames = "images.txt";
    listFramesFile.open(listFrames);
    std::string frameName;

    int nFrames = 0;

    while ( std::getline(listFramesFile, frameName) ){
        // Read each frame from the list
        frame = cv::imread(frameName, CV_LOAD_IMAGE_COLOR);

        if(frame.rows==0 || frame.cols==0)
            break;

        if (nFrames == 0) {
            roi = selectROI("tracker",frame);
            //quit if ROI was not selected
            if(roi.width==0 || roi.height==0)  break;
            tracker->init(frame,roi);
        }
        else {
            // update the tracking result
            tracker->update(frame,roi);
            // draw the tracked object
            rectangle( frame, roi, Scalar( 255, 0, 0 ), 2, 1 );
            // show image with the tracked object
            imshow("tracker",frame);
            //quit on ESC button
            if(waitKey(1)==27)break;
        }

        nFrames++;
    }
    listFramesFile.close();
}
*/


int main(int argc, char* argv[]){

    if (argc > 5) return -1;

    bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool SILENT = false;
    bool LAB = false;

    for(int i = 0; i < argc; i++){
        if ( strcmp (argv[i], "hog") == 0 )
            HOG = true;
        if ( strcmp (argv[i], "fixed_window") == 0 )
            FIXEDWINDOW = true;
        if ( strcmp (argv[i], "singlescale") == 0 )
            MULTISCALE = false;
        if ( strcmp (argv[i], "show") == 0 )
            SILENT = false;
        if ( strcmp (argv[i], "lab") == 0 ){
            LAB = true;
            HOG = true;
        }
        if ( strcmp (argv[i], "gray") == 0 )
            HOG = false;
    }

    // Create KCFTracker object
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

    // Frame readed
    cv::Mat frame;

    // Tracker results
    cv::Rect result;

    // Path to list.txt
    std::ifstream listFile;
    std::string fileName = "images.txt";
    listFile.open(fileName);

    // Read Images
    std::ifstream listFramesFile;
    std::string listFrames = "images.txt";
    listFramesFile.open(listFrames);
    std::string frameName;


    // Write Results
    std::ofstream resultsFile;
    std::string resultsPath = "output.txt";
    resultsFile.open(resultsPath);

    // Frame counter
    int nFrames = 0;

    cv::namedWindow( "Image View", 1 );
    while ( std::getline(listFramesFile, frameName) ){
        frameName = frameName;
        // Read each frame from the list
        frame = cv::imread(frameName, CV_LOAD_IMAGE_COLOR);

        // First frame, give the groundtruth to the tracker
        if (nFrames == 0) {
            result = cv::selectROI("tracker",frame);
            //quit if ROI was not selected

            std::cout << result << std::endl;
            if(result.width==0 || result.height==0)  break;
            tracker.init( result, frame );
        }
        // Update
        else{
            result = tracker.update(frame);
            rectangle( frame, cv::Point( result.x, result.y ), cv::Point( result.x+result.width, result.y+result.height), cv::Scalar( 0, 255, 255 ), 1, 8 );
            resultsFile << result.x << "," << result.y << "," << result.width << "," << result.height << std::endl;

            std::cout << "track: " << result << std::endl;
        }

        nFrames++;

        if (!SILENT){
            cv::imshow("Image", frame);
            cv::waitKey(500);
        }
    }
    resultsFile.close();
    listFile.close();
}

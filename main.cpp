//Best one.
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif
#define CAPTURE_MODE 0
#define VIEW_MODE 1
#define PAUSE_MODE 2
using namespace cv;
using namespace std;
// Function to generate chessboard corners
void create3DChessboardCorners1( vector<Point3f>& corners);
int main(int argc, char *argv[]) {
    // Check number of arguments given.
//    if ( argc != 5 ) {
//        cout << "Wrong number of input arguments." << endl;
//        cout << "#column corners, #row corners, patch size in mm, #views to capture" << endl;
//        return -1;
//    }
//    
    // Try to open the video capture device
    VideoCapture cap("/Users/u7/Library/Developer/Xcode/DerivedData/cam4/intrinsics.avi");
    if ( !cap.isOpened() ) {
        cout << "Could not open the video capture device." << endl;
        return -1;
    }
    
    Mat frame;
    cout << "Frame size: " << cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x" << cap.get(CV_CAP_PROP_FRAME_HEIGHT)<< endl;
    
    cout << "Starting camera calibration." << endl;
    Size boardSize(8,6); // (cols, rows)cc
    int nBoards = 25;                 // Number off board views to capture
    int successes = 0;							 // Counter for the number of succesful views captured
    vector< vector<Point2f> > image_points;
    vector< vector<Point3f> > object_points;
    vector< Point3f> cors;
    
    int mode = VIEW_MODE; // Start in view mode
    
    namedWindow("Frame", 1);
    while (successes < nBoards) {
        cap >> frame; // Grab a frame from the video stream
        
        // Does frame contain data ?
        if ( !frame.data ) {
            cout << "Frame does not contain data." << endl;
            continue;
        }
        
        imshow("Frame", frame); // Show the captured frame
        
        int c = waitKey(15);
        if (c == 'p') mode = PAUSE_MODE;
        if (c == 'c') mode = CAPTURE_MODE;
        if (c == 'v') mode = VIEW_MODE;
        
        if (mode == VIEW_MODE && c != 27) continue;
        
        if (mode == CAPTURE_MODE) {
            vector<Point2f> corners;

            bool patt_found = findChessboardCorners(frame,
                                                    boardSize,
                                                    corners,
                                                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            
            // Did we find a chessboard pattern ?
            if ( !patt_found ) {
                cout << "No pattern found in image." << endl;
                continue;
            }
            
            // Have all corners been detected ?
            if( corners.size() != boardSize.area()) {
                cout << corners.size() << " out of " << boardSize.area() << " detected. " << endl;
                continue;
            }
            
            // Pattern present and all corners have been detected, continue
            cout << "[" << (successes+1) << "/" << nBoards << "] Good frame :D" << endl;
            
            // Get subpixel accuracy on corners
            Mat frame_grey;
            cvtColor(frame, frame_grey, CV_BGR2GRAY); // Convert frame to grey colour
            cornerSubPix(frame_grey,
                         corners,
                         Size(11, 11),
                         Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            
            // Show image with detected corners
            drawChessboardCorners(frame, boardSize, Mat(corners), patt_found);
            imshow("Frame", frame);
            
            mode = PAUSE_MODE;
            image_points.push_back(corners); // Append corners of this frame to the list
            successes++;
        } // if( mode == CAPTURE_MODE )
        
        // Pause mode
        while (mode == PAUSE_MODE && c != 27) {
            c = waitKey(15);
            // Save current frame ?
            if (c == 's') {
                imwrite("frame.png", frame);
            }
            if (c == 'v') {
                mode = VIEW_MODE;
            }
            if (c == 'c') {
                mode = CAPTURE_MODE;
            }
        }
        
        if (c == 27) return 0; // Exit if escape key is pressed
    }
    
    // Assign object points, these are the same for every view
    //create3DChessboardCorners(boardSize, squareSize,cors);
    create3DChessboardCorners1(cors);
    object_points.assign(nBoards, cors);
    Mat cameraMatrix;
    Mat distCoeffs;
    vector<Mat> rotVectors;
    vector<Mat> tranVectors;
    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(object_points, image_points, frame.size(), cameraMatrix,
                                 distCoeffs, rotVectors, tranVectors, CV_CALIB_FIX_ASPECT_RATIO);
    
    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;
     cout<<"This is cameraMatrix "<<cameraMatrix<<endl;
    // The actual calibration
    // Save the intrinsics
    FileStorage fs_int("/Users/u7/Library/Developer/Xcode/DerivedData/cam4/intrinsics.xml", FileStorage::WRITE);
    if(!fs_int.isOpened())
    {
        return -1;
    }
    fs_int << "CameraMatrix" << cameraMatrix;
   
    fs_int << "DistortionCoeffs" << distCoeffs;
    cout<<"This is dist "<<distCoeffs<<endl;
    fs_int.release(); // Close file
    cout<<"done saving"<<endl;
    return 0;
}
void create3DChessboardCorners1( vector<Point3f>& corners) {
    for( int i = 0; i < 6; ++i )
        for( int j = 0; j < 8; ++j )
            corners.push_back(Point3f(float( j*115), float( i*115 ), 0));
    cout<<"This is corners"<<corners<<endl;
}

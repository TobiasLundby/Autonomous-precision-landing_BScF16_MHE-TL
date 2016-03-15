/*****************************************************************************
* University of Southern Denmark
* UAS Center
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: drone_tracking.cpp
* PROJECT....: Autonomous precision landing ground system
* DESCRIPTION: Includes functionality to track a drone
*
*****************************************************************************/

#pragma once

/***************************** Include files *******************************/
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <string>

#include "opencv2/features2d.hpp"


using namespace cv;
using namespace std;
/*****************************    Defines    *******************************/
# define M_PI           3.14159265358979323846  /* pi */

/*****************************   Class   *******************************/
class drone_tracking
{
public: // Methods
  drone_tracking();
  drone_tracking(string);
private: // Methods
  void show_frame(string, Mat);
  void frame_analysis();
  void diode_detection();

private: // Variables
  string filename;
  Mat frame_bgr;
  VideoCapture capture;
  string video_window_text = "Drone tracking";

  // Diode detection Variables
  // HSV limits for color seperation
  int hsv_h_red_base        = 160; //60 is for green
  int hsv_h_red_sensitivity = 30;
  int hsv_h_red_low         = hsv_h_red_base - hsv_h_red_sensitivity;
  int hsv_h_red_upper       = hsv_h_red_base + hsv_h_red_sensitivity;
  int hsv_s_red_low         = 100;
  int hsv_s_red_upper       = 255;
  int hsv_v_red_low         = 100;
  int hsv_v_red_upper       = 255;
  int gaussian_blur         = 11; //Must be positive and odd
  Mat frame_hsv;
  Mat frame_red;
  Mat frame_gray;
  Mat frame_gray_with_Gblur;
  Mat mask_red;

  double m, M;
  Point p_min, p_max;
};

drone_tracking::drone_tracking()
/*****************************************************************************
*   Input    :
*   Output   :
*   Function : Default constructor
******************************************************************************/
{}

drone_tracking::drone_tracking(string filenameIn)
/*****************************************************************************
*   Input    : (int) number of commandline arguments
*            : (char**) commandline arguments
*   Output   : None
*   Function : Overload constructor
******************************************************************************/
{
  filename = filenameIn;
  // Open specified video file or webcam
  if(filename=="") { // If filename is webcam
    cout << "Opening external webcam"<< endl;
    capture.open(1);                                 // Open external webcam
  } else if(filename=="webcam") {
    cout << "Opening external webcam"<< endl;
    capture.open(0);                                // Open internal webcam
  } else {
    cout << "Opening video" << endl;
    capture.open(filename);                         // Open file
  }

  if(capture.isOpened()) { // Test if capture is opened
    cout << "Capture is opened" << endl;
    for(;;) { // Processing
      capture >> frame_bgr;
      if(frame_bgr.empty())
        break;
      frame_analysis(); // Master method for analysis
      if(waitKey(10) >= 0)
        break;
    }
  } else { // Error capture is not opened
    cout << "No capture to open" << endl;
    frame_bgr = Mat::zeros(480, 640, CV_8UC1);
    show_frame("Failed to open", frame_bgr);
    waitKey(0);
  }
}

void drone_tracking::show_frame(string window_text, Mat in_frame)
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Displays video
******************************************************************************/
{
  imshow(window_text, in_frame);
}

void drone_tracking::frame_analysis()
/*****************************************************************************
*   Input    : None (the frame is a part of the class)
*   Output   : None (the frame is a part of the class)
*   Function : Analyses the frame
******************************************************************************/
{
  // ALL THE ANALYSIS METHODS SHOULD BE CALLED HERE - THIS IS THE MASTER
  // ALL THE ANALYSIS METHODS SHOULD BE CALLED HERE - THIS IS THE MASTER
  show_frame(video_window_text, frame_bgr);
  diode_detection();
}

void drone_tracking::diode_detection()
/*****************************************************************************
*   Input    : None (the frames are a part of the class)
*   Output   : None (the frames are a part of the class)
*   Function : Finds the diode
******************************************************************************/
{
  cvtColor(frame_bgr, frame_hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  cvtColor(frame_bgr, frame_gray, COLOR_BGR2GRAY); //Convert the captured frame from BGR to HSV
  show_frame("Gray frame", frame_gray);
  GaussianBlur(frame_gray, frame_gray_with_Gblur, Size(gaussian_blur, gaussian_blur), 0);

  //minMaxLoc(frame_gray_with_Gblur, &m, &M, &p_min, &p_max);
  //cout << "min: " << m << " at " << p_min << " max: " << M << " at " << p_max << endl;
  //circle(frame_gray_with_Gblur, p_max, 40, Scalar(255, 0, 0), 5);

  // Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

  params.blobColor = 255;
  // Change thresholds
	//params.minThreshold = 0;
	//params.maxThreshold = 100;
	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 10;
  params.maxArea = 100;
	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.1;
	// Filter by Convexity
	//params.filterByConvexity = true;
	//params.minConvexity = 0.87;
	// Filter by Inertia
	//params.filterByInertia = true;
	//params.minInertiaRatio = 0.01;
	// Storage for blobs
	vector<KeyPoint> keypoints;

  // Set up detector with params
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	// Detect blobs
	detector->detect( frame_gray_with_Gblur, keypoints);
  Mat im_with_keypoints;
	drawKeypoints( frame_gray_with_Gblur, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

	// Show blobs
	imshow("keypoints", im_with_keypoints );

  //show_frame("Gray frame with blur", frame_gray_with_Gblur);

  inRange(frame_hsv, Scalar(hsv_h_red_low,hsv_s_red_low,hsv_v_red_low), Scalar(hsv_h_red_upper, hsv_s_red_upper, hsv_v_red_upper), mask_red);
  frame_red = Scalar(0);
  frame_bgr.copyTo(frame_red, mask_red);
  //show_frame("Red frame", frame_red);
}


// int drone_tracking::dummy_function()
// /*****************************************************************************
// *   Input    :
// *   Output   :
// *   Function :
// ******************************************************************************/
// {
//
// }

/****************************** End Of Module *******************************/

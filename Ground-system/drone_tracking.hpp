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
#include "opencv2/core.hpp"               // OpenCV includes
#include "opencv2/imgproc.hpp"            // OpenCV includes
#include "opencv2/highgui.hpp"            // OpenCV includes
#include "opencv2/videoio.hpp"            // OpenCV includes
#include <string>                         // Nessecary for constructor and other
#include "opencv2/features2d.hpp"         // Used for blob detection


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
  int  midpoint_circle_algorithm(Mat&, int, int, int);

private: // Variables
  string filename;
  Mat frame_bgr;
  VideoCapture capture;
  string video_window_text = "Drone tracking";

  // Diode detection Variables
  // HSV limits for color seperation
  int hsv_h_red_base        = 160; //60 is for green
  int hsv_h_red_sensitivity = 40;
  int hsv_h_red_low         = hsv_h_red_base - hsv_h_red_sensitivity;
  int hsv_h_red_upper       = hsv_h_red_base + hsv_h_red_sensitivity;
  int hsv_s_red_low         = 100;
  int hsv_s_red_upper       = 255;
  int hsv_v_red_low         = 100;
  int hsv_v_red_upper       = 255;
  int gaussian_blur         = 3; //Must be positive and odd
  Mat frame_hsv;
  Mat frame_red;
  Mat frame_red_hsv;
  Mat frame_red_split[3];
  Mat frame_gray;
  Mat frame_gray_with_Gblur;
  Mat mask_red;

  bool enable_wait = true;
  int wait_time_ms = 200;

	// Storage for blobs
	vector<KeyPoint> keypoints;

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
  //show_frame(video_window_text, frame_bgr);
  diode_detection();
}

void drone_tracking::diode_detection()
/*****************************************************************************
*   Input    : None (the frames are a part of the class)
*   Output   : None (the frames are a part of the class)
*   Function : Finds the diode
******************************************************************************/
{
  if (enable_wait)
    waitKey(wait_time_ms);
  //resize(frame_bgr, frame_bgr, Size(600, 400)); // The webcam is 1280x720
  cvtColor(frame_bgr, frame_hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  cvtColor(frame_bgr, frame_gray, COLOR_BGR2GRAY); //Convert the captured frame from BGR to HSV
  GaussianBlur(frame_gray, frame_gray_with_Gblur, Size(gaussian_blur, gaussian_blur), 0); // Gaussian blur on gray frame

  // Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;
  // Change thresholds
	//params.minThreshold = 0;
	//params.maxThreshold = 100;
  params.blobColor = 255;
	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 10;
  params.maxArea = 300;
	// Filter by Circularity
	params.filterByCircularity = false;
	params.minCircularity = 0.5;
	// Filter by Convexity
	params.filterByConvexity = false;
	params.minConvexity = 0.87;
	// Filter by Inertia
	params.filterByInertia = false;
	params.minInertiaRatio = 0.01;

  // Set up detector with params
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	// Detect blobs
	detector->detect(frame_gray_with_Gblur, keypoints);
  Mat im_with_keypoints;
	drawKeypoints( frame_gray_with_Gblur, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  //cout << "There are " << keypoints.size() << " keypoints" << endl;

  inRange(frame_hsv, Scalar(hsv_h_red_low,hsv_s_red_low,hsv_v_red_low), Scalar(hsv_h_red_upper, hsv_s_red_upper, hsv_v_red_upper), mask_red);
  frame_red = Scalar(0);
  dilate(mask_red, mask_red, cv::Mat(), cv::Point(-1,-1)); // Enhance the red areas in the image
  frame_bgr.copyTo(frame_red, mask_red);

  split(frame_red,frame_red_split);

  cvtColor(frame_red, frame_red_hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

  int total_avg_hue_value;
  if (keypoints.size()) {
    for (size_t i = 0; i < keypoints.size(); i++) {
      int iterations = 0;
      int avg_hue_value = 0;
      for (int j = 1; j < keypoints[i].size; j++) {
        iterations++;
        avg_hue_value += midpoint_circle_algorithm(mask_red, keypoints[i].pt.x, keypoints[i].pt.y, j);
        circle(im_with_keypoints, keypoints[i].pt, j, Scalar(0, 255, 0), 1);
      }
      total_avg_hue_value = avg_hue_value / iterations;
      if (total_avg_hue_value > 10) {
      //  cout << "Fucking red diode" << endl;
        circle(im_with_keypoints, keypoints[i].pt, keypoints[i].size, Scalar(255-(i*10), 0, 0), keypoints[i].size *1.5);
      }
      cout << "Index " << i << " has avg value " << total_avg_hue_value << endl;
      //circle(im_with_keypoints, keypoints[i].pt, keypoints[i].size, Scalar(255-(i*10), 0, 0), keypoints[i].size);
    }
  }

  // Show blobs
	imshow("keypoints", im_with_keypoints );

  //show_frame("Channel frame", frame_red_split[0]);
  show_frame("Red frame", mask_red);
}

int drone_tracking::midpoint_circle_algorithm(Mat& image, int x0, int y0, int radius)
/*****************************************************************************
*   Input    : x
*   Output   :
*   Function :
*   Ref      : https://en.wikipedia.org/wiki/Midpoint_circle_algorithm
******************************************************************************/
{
  int x = radius;
  int y = 0;
  int decisionOver2 = 1 - x;   // Decision criterion divided by 2 evaluated at x=r, y=0
  int sum = 0;
  int pixel_count = 0;

  while( y <= x )
  {
    pixel_count += 8;
    sum += image.at<uchar>(x + x0, y + y0); // Octant 1
    sum += image.at<uchar>(y + x0, x + y0); // Octant 2
    sum += image.at<uchar>(-x + x0, y + y0); // Octant 4
    sum += image.at<uchar>(-y + x0, x + y0); // Octant 3
    sum += image.at<uchar>(-y + x0, -x + y0); // Octant 6
    sum += image.at<uchar>(-x + x0, -y + y0); // Octant 5
    sum += image.at<uchar>(x + x0, -y + y0); // Octant 7
    sum += image.at<uchar>(y + x0, -x + y0); // Octant 8

    y++;
    if (decisionOver2<=0)
    {
      decisionOver2 += 2 * y + 1;   // Change in decision criterion for y -> y+1
    }
    else
    {
      x--;
      decisionOver2 += 2 * (y - x) + 1;   // Change for y -> y+1, x -> x-1
    }
  }
  if (pixel_count)
    return sum / pixel_count;
  return 1;
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

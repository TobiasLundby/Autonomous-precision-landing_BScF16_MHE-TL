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
#include <vector>



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

  // Drone shape detection methods
  void find_black_mask();

private: // Variables
  string filename;
  Mat frame_bgr;
  VideoCapture capture;
  string video_window_text = "Drone tracking";

  // Drone shape detection variables
  Mat shape_frame;
  Mat shape_frame_hsv;
  Mat black_mask_bgr;
  Mat black_mask_hsv;
  // HSV values for black
  int bgr_b_black_low = 0;
  int bgr_g_black_low = 0;
  int bgr_r_black_low = 0;
  int bgr_b_black_upper = 30;
  int bgr_g_black_upper = 30;
  int bgr_r_black_upper = 30;

  int hsv_h_black_base        = 0; //60 is for green
  int hsv_h_black_sensitivity = 30;
  int hsv_h_black_low         = 0;//hsv_h_black_base //- hsv_h_black_sensitivity;
  int hsv_h_black_upper       = 0;//hsv_h_black_base + hsv_h_black_sensitivity;
  int hsv_s_black_low         = 0;
  int hsv_s_black_upper       = 0;
  int hsv_v_black_low         = 0;
  int hsv_v_black_upper       = 10;


  //http://docs.opencv.org/3.1.0/d1/dc5/tutorial_background_subtraction.html#gsc.tab=0


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
  find_black_mask();
  show_frame(video_window_text, frame_bgr);
}


// Drone shape tracking methods
void drone_tracking::find_black_mask()
{
  frame_bgr.copyTo(shape_frame);  // Copy original frame
  cvtColor(shape_frame, shape_frame_hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  inRange(shape_frame, Scalar(bgr_b_black_low,bgr_g_black_low,bgr_r_black_low), Scalar(bgr_b_black_upper, bgr_g_black_upper, bgr_r_black_upper),black_mask_bgr);
  inRange(shape_frame_hsv, Scalar(hsv_h_black_low,hsv_s_black_low,hsv_v_black_low), Scalar(hsv_h_black_upper, hsv_s_black_upper, hsv_v_black_upper),black_mask_hsv);
  //inRange(shape_frame,cv::Scalar(black_lowerb_b,black_lowerb_g,black_lowerb_r),cv::Scalar(black_upperb_b,black_upperb_g,black_upperb_r),black_mask);
  imshow("RGB Black mask",black_mask_bgr);
  imshow("HSV Black mask ",black_mask_hsv);
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

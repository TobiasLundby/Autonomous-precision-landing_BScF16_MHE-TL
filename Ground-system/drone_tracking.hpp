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
  void frame_save(Mat&);
  void frame_save(Mat&, string);
  void diode_detection();

private: // Variables
  string filename;
  Mat frame_bgr;
  VideoCapture capture;
  string video_window_text = "Drone tracking";

  int global_frame_counter = 0;

  // frame_save
  int frame_save_counter = 1;
  string frame_save_type = "png";

  // Diode detection Variables
  // HSV limits for color seperation
  int hsv_h_red_base        = 160; //60 is for green
  int hsv_h_red_sensitivity = 25;
  int hsv_h_red_low         = hsv_h_red_base - hsv_h_red_sensitivity;
  int hsv_h_red_upper       = hsv_h_red_base + hsv_h_red_sensitivity;
  int hsv_s_red_low         = 100;
  int hsv_s_red_upper       = 255;
  int hsv_v_red_low         = 100;
  int hsv_v_red_upper       = 255;
  int gaussian_blur         = 3; //Must be positive and odd
  int dilate_color_iterations = 3;
  Mat frame_hsv;
  Mat frame_red;
  Mat frame_red_hsv;
  Mat frame_red_split[3];
  Mat frame_gray;
  Mat frame_gray_with_Gblur;
  Mat mask_red;
  Mat mask_circles;
  Mat frame_temp;

  int mean_multiply_factor = 100; //Effects the one below linear
  double color_threashold = 0.010;
  int hue_radius = 20; // [%]

  bool enable_wait = false;
  int wait_time_ms = 100;

  bool test_bool = true;

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
  global_frame_counter++;
  cout << endl << "Frame: " << global_frame_counter << endl;
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
	params.minArea = 5;
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
  for (size_t i = 0; i < dilate_color_iterations; i++)
    dilate(mask_red, mask_red, Mat(), Point(-1,-1)); // Enhance the red areas in the image
  frame_bgr.copyTo(frame_red, mask_red);

  split(frame_red,frame_red_split);

  cvtColor(frame_red, frame_red_hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

  // int total_avg_hue_value;
  // if (keypoints.size()) {
  //   for (size_t i = 0; i < keypoints.size(); i++) {
  //     int iterations = 0;
  //     int avg_hue_value = 0;
  //     for (int j = 1; j < keypoints[i].size; j++) {
  //       iterations++;
  //       avg_hue_value += midpoint_circle_algorithm(mask_red, keypoints[i].pt.x, keypoints[i].pt.y, j);
  //       circle(im_with_keypoints, keypoints[i].pt, j, Scalar(0, 255, 0), 1);
  //     }
  //     total_avg_hue_value = avg_hue_value / iterations;
  //     if (total_avg_hue_value > 10) {
  //     //  cout << "Fucking red diode" << endl;
  //       circle(im_with_keypoints, keypoints[i].pt, keypoints[i].size, Scalar(255-(i*10), 0, 0), keypoints[i].size *1.5);
  //     }
  //     //cout << "Index " << i << " has avg value " << total_avg_hue_value << endl;
  //     //circle(im_with_keypoints, keypoints[i].pt, keypoints[i].size, Scalar(255-(i*10), 0, 0), keypoints[i].size);
  //   }
  // }

  //mask_circles
  double mean_of_frame = 0;
  if (keypoints.size())
  {
    for (size_t i = 0; i < keypoints.size(); i++) {
      //for (size_t i = 3; i < 4; i++) {
      putText(im_with_keypoints, to_string(i), keypoints[i].pt, FONT_HERSHEY_PLAIN, 2, Scalar(0,255,0));
      mask_circles = mask_red.clone(); // Just to get the proper size
      mask_circles = Scalar(0); // Image data clear
      circle(mask_circles, keypoints[i].pt, keypoints[i].size * 1+(hue_radius/100), Scalar(255), -1); // Draw a circle
      frame_temp = Scalar(0);
      mask_red.copyTo(frame_temp, mask_circles);
      //imshow("Temp", frame_temp );
      mean_of_frame = mean(frame_temp)[0]/(pow(keypoints[i].size,2)*M_PI)*mean_multiply_factor;
      //cout << i << ": " << mean_of_frame << "\t";
      if (mean_of_frame > color_threashold) {
        circle(im_with_keypoints, keypoints[i].pt, keypoints[i].size, Scalar(255-(i*10), 0, 0), keypoints[i].size+(hue_radius/100));
        cout << "Red LED at position: " << keypoints[i].pt << endl;
      }
    }
    cout << endl;
  }
  // Show blobs
  show_frame("Recognized red LEDs", im_with_keypoints);
  frame_save(im_with_keypoints);

  //show_frame("Channel frame", frame_red_split[0]);
  //show_frame("Red frame", frame_red);
}

void drone_tracking::frame_save(Mat& frame_in)
/*****************************************************************************
*   Input    : The actual frame
*   Output   : None (void)
*   Function : Saves the frame with an automatically generated name. NOTE the folders 'output', 'images', and 'videos' must exist
******************************************************************************/
{
  string name;
  name = string("frame-") + to_string(frame_save_counter);
  frame_save_counter++;
  frame_save(frame_in, name);
}

void drone_tracking::frame_save(Mat& frame_in, string name_in)
/*****************************************************************************
*   Input    : The actual frame and a name WITHOUT file extension
*   Output   : None (void)
*   Function : Saves the frame. NOTE the folders 'output', 'images', and 'videos' must exist
******************************************************************************/
{
  name_in += "." + frame_save_type;
  imwrite( "./output/images/"+name_in, frame_in );
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

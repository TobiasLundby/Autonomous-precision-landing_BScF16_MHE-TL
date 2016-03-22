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
  void show_frame(string, Mat); //Use window_names to point to the name
  void frame_analysis();
  void create_windows();
  void frame_save(Mat&);
  void frame_save(Mat&, string);
  vector<KeyPoint> diode_detection();
  vector<KeyPoint> keypoint_detection(Mat, Mat, Mat);
  vector<KeyPoint> keypoint_filtering(vector<KeyPoint>, bool, Mat);
  void window_taskbar_create(int);

private: // Variables
  string filename;
  Mat frame_bgr;
  VideoCapture capture;

 // General variables
  bool enable_wait = false;
  int wait_time_ms = 500;
  bool debug = false;
  int global_frame_counter = 0;
  int start_skip_frames = 0;

 // show_frame variables
  bool window_enable = true;
  vector<string> window_names; // Holds the window names but no values can be added here, must be added in the method.
  bool custom_window_size = true;
  int custom_window_width = 600; // Custom but ensure the proper aspect ratio for the camera
  int custom_window_height = 400;
  int screen_dimension_width = 1280; //HD: 1080; FULL-HD: 1920; Other: 1280
  int screen_dimension_height = 800; //HD: 800 (or 720); FULL-HD: 1200 (or 1080); Other: 800
  bool enable_trackbars = false;

 // frame_save variables
  int frame_save_counter = 1;
  string frame_save_type = "png"; // File extension for saving frames

 // Diode detection Variables
  // HSV limits for color seperation (presumably changable)
  int hsv_h_red_base        = 160; //60 is for green, 160 is for red
  int hsv_h_red_sensitivity = 24;
  int hsv_h_red_low         = hsv_h_red_base - hsv_h_red_sensitivity;
  int hsv_h_red_upper       = hsv_h_red_base + hsv_h_red_sensitivity;
  int hsv_s_red_low         = 100;
  int hsv_s_red_upper       = 255;
  int hsv_v_red_low         = 100;
  int hsv_v_red_upper       = 255;
  int gaussian_blur         = 3; //Must be positive and odd. Higher, more blur
  int dilate_color_iterations = 3;

  // Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;
  bool second_detection_run = false;

  int mean_multiply_factor = 1000000; //Effects the one below linear
  int color_threashold_1 = 110;
  int color_threashold_2 = 80;
  int hue_radius = 20; // [%]

  vector<KeyPoint> leds; // Stores the red LED keypoints

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
    create_windows();
    for(;;) { // Processing
      capture >> frame_bgr;
      if(frame_bgr.empty())
        break;
      if (!frame_bgr.empty())
        if (start_skip_frames < global_frame_counter)
          frame_analysis(); // Master method for analysis
      global_frame_counter++;
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

void drone_tracking::create_windows()
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Creates the windows specified in the vector.
******************************************************************************/
{
  window_names.push_back("Input stream"); //Window 1
  window_names.push_back("Recognized red LEDs"); //Window 2
  window_names.push_back("Red mask"); //Window 3
  window_names.push_back("Red frame"); //Window 4
  //window_names.push_back("Other2"); //Window 5
  //window_names.push_back("Other3"); //Window 6
  //window_names.push_back("Window N"); //Window N
  if (window_enable)
  {
    int j = 0; // Secondary row position counter
    for (size_t i = 0; i < window_names.size(); i++) { // Loop through all the windows
      if (custom_window_size)
      {
        namedWindow(window_names[i],WINDOW_NORMAL); // WINDOW_NORMAL allows for window resize
        resizeWindow(window_names[i], custom_window_width, custom_window_height); // Resize the window
        if ((i+1)*custom_window_width < screen_dimension_width) { // First screen row
          moveWindow(window_names[i], i*custom_window_width, 0);
        } else { // Secondary screen row
          moveWindow(window_names[i], j*custom_window_width, custom_window_height+40);
          j++;
        }
      } else
        namedWindow(window_names[i],WINDOW_AUTOSIZE); // WINDOW_AUTOSIZE does not allow for resize
      if (enable_trackbars)
        window_taskbar_create(i); // Create appropiate task bars
    }
  }
}

void drone_tracking::window_taskbar_create(int window_number)
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Creates the specified task bars
******************************************************************************/
{
  if (window_number==1) // Test which window window_taskbar_create is called with
     createTrackbar("Threashold", window_names[window_number], &color_threashold_1, 1000); // 1st aug: name; 2nd aug: window; 3rd aug: pointer to the variabel (must be int); 4th aug: max value
  if (window_number==3) // Test which window window_taskbar_create is called with
     createTrackbar("Dilate iterations", window_names[window_number], &dilate_color_iterations, 10); // 1st aug: name; 2nd aug: window; 3rd aug: pointer to the variabel (must be int); 4th aug: max value
}

void drone_tracking::show_frame(string window_text, Mat in_frame)
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Displays the given frame
******************************************************************************/
{
  if (window_enable)
    if (!in_frame.empty())
      imshow(window_text, in_frame); // 1st aug: window name; 2nd aug: frame to show
}

void drone_tracking::frame_analysis()
/*****************************************************************************
*   Input    : None (the frame is a part of the class)
*   Output   : None (the frame is a part of the class)
*   Function : Analyses the frame
******************************************************************************/
{
  if (enable_wait)
    waitKey(wait_time_ms); // Wait if enabled, typically used for manual debug
  // ALL THE ANALYSIS METHODS SHOULD BE CALLED HERE - THIS IS THE MASTER
  // ALL THE ANALYSIS METHODS SHOULD BE CALLED HERE - THIS IS THE MASTER
  show_frame(window_names[0], frame_bgr); // Show original frame
  if (debug)
    cout << endl << "Frame: " << global_frame_counter << endl;
  leds = diode_detection();
}

void drone_tracking::frame_save(Mat& frame_in)
/*****************************************************************************
*   Input    : The actual frame
*   Output   : None (void)
*   Function : Saves the frame with an automatically generated name. NOTE the folders 'output', 'images', and 'videos' must exist
******************************************************************************/
{
  string name;
  name = string("frame-") + to_string(frame_save_counter); // Generate name
  frame_save_counter++;
  frame_save(frame_in, name); // 1st aug: frame to save; 2nd aug: file name
}

void drone_tracking::frame_save(Mat& frame_in, string name_in)
/*****************************************************************************
*   Input    : The actual frame and a name WITHOUT file extension
*   Output   : None (void)
*   Function : Saves the frame. NOTE the folders 'output', 'images', and 'videos' must exist
******************************************************************************/
{
  name_in += "." + frame_save_type; // Add the file extension
  if (!frame_in.empty()) // Only save a frame with content
    imwrite( "./output/images/"+name_in, frame_in ); // 1st aug: path and name; 2nd aug: frame to save
}

vector<KeyPoint> drone_tracking::diode_detection()
/*****************************************************************************
*   Input    : None (the frames are a part of the class)
*   Output   : None (the frames are a part of the class)
*   Function : Finds the diode
******************************************************************************/
{
  Mat frame_hsv;
  Mat frame_red;
  Mat frame_gray;
  Mat frame_gray_with_Gblur;
  Mat mask_red;

  cvtColor(frame_bgr, frame_hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  cvtColor(frame_bgr, frame_gray, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY
  GaussianBlur(frame_gray, frame_gray_with_Gblur, Size(gaussian_blur, gaussian_blur), 0); // Gaussian blur on gray frame, 1st aug: input frame; 2nd aug: output frame; 3rd aug: defines the blur radius; 4th aug: Gaussian kernel standard deviation in X direction, when this is 0 it is computed from the 3rd aug. Gaussaian blur is used since an example used this.

  inRange(frame_hsv, Scalar(hsv_h_red_low,hsv_s_red_low,hsv_v_red_low), Scalar(hsv_h_red_upper, hsv_s_red_upper, hsv_v_red_upper), mask_red); // Find the areas which contain red color. 1st aug: inpur frame; 2nd aug: the lower HSV limits; 3rd aug: the upper HSV limits; 4th aug: the output mask.
  frame_red = Scalar(0); // Clear the red frame.
  for (size_t i = 0; i < dilate_color_iterations; i++)
    dilate(mask_red, mask_red, Mat(), Point(-1,-1)); // Enhance the red areas in the image
  frame_bgr.copyTo(frame_red, mask_red); // Copy frame_bgr to frame_red but only the area marked in the red_mask

  vector<KeyPoint> detected_leds;
  detected_leds = keypoint_detection(frame_gray, frame_gray_with_Gblur, mask_red); // Detect the red LEDs

  if (debug) {
    cout << "There are " << detected_leds.size() << " red LEDs" << endl;
    if (second_detection_run)
      cout << "** outdoor **" << endl;
  }

  show_frame(window_names[2], frame_red); // Show the frame only containing the red color
  show_frame(window_names[3], mask_red); // Show the red masking frame

  return detected_leds;
}

vector<KeyPoint> drone_tracking::keypoint_detection(Mat in_frame_gray, Mat in_frame_gray_with_Gblur, Mat in_mask_red)
/*****************************************************************************
*   Input    : None (the frames are a part of the class). Uses frame_bgr ,im_with_keypoints, and mask_red
*   Output   : None (the frames are a part of the class)
*   Function : Finds brighest points
******************************************************************************/
{
  Mat im_with_keypoints;
 // All the parameters below are used for the blob detector - only some of them are used because motion blur introduces different shapes etc.
  // Change thresholds
  //params.minThreshold = 0;
  //params.maxThreshold = 100;
  params.blobColor = 255; // This parameter is used instead of the threadholds to detect the white color
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 3;
  params.maxArea = 300;
  // Filter by Circularity - we do not do this parameter due to motion blur
  params.filterByCircularity = false;
  params.minCircularity = 0.5;
  // Filter by Convexity - we do not use this parameter to ensure detection
  params.filterByConvexity = false;
  params.minConvexity = 0.87;
  // Filter by Inertia - not used but it means "the inertial resistance of the blob to rotation about its principal axes"
  params.filterByInertia = false;
  params.minInertiaRatio = 0.01;
  params.maxInertiaRatio = 0.5;

	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params); // Set up detector with params
  vector<KeyPoint> keypoints; // Used for holding all the keypoints
  vector<KeyPoint> temp_keypoints; // Used to hold the filtered keypoints (red keypoints)
	detector->detect(in_frame_gray_with_Gblur, keypoints); // Detect the keyframes from the frame
  temp_keypoints = keypoint_filtering(keypoints, second_detection_run, in_mask_red); // Filter the detected keypoints

  if (!temp_keypoints.size()) { // See if any keypoints matched a red LED. If not then detect using different frame (mask_red)
    keypoints.clear(); // Delete the old keypoints to hold the new ones from the detector.
    detector->detect(in_mask_red, keypoints); // Detect the keyframes from the frame
    second_detection_run = true; // Tell the other functions that the keypoints were found from the secondary frame
    temp_keypoints = keypoint_filtering(keypoints, second_detection_run, in_mask_red); // Filter the detected keypoints
  } else
    second_detection_run = false; // Tell the other functions that the keypoints were found from the primary frame

  im_with_keypoints = Scalar(0,0,0); // Clear all the channels in the image
  drawKeypoints( frame_bgr, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS ); // Show the keypoints on a frame. 1st aug: input frame; 2nd aug: vector with keypoints to be drawn; 3rd aug: output frame; 4th aug: color of the drawn keypoints (red here); 5th aug: For each keypoint, the circle around keypoint with keypoint size and orientation will be drawn.
  for (size_t i = 0; i < temp_keypoints.size(); i++) // Run through all the red LED keypoints.
    circle(im_with_keypoints, temp_keypoints[i].pt, temp_keypoints[i].size, Scalar(255, i*40, i*20), temp_keypoints[i].size+(hue_radius/100)); // Draw the right keypoints (the red LEDs). 1st aug: in frame; 2nd aug: the centrum of the circle; 3rd aug: the circle radius; 4th aug: the color of the circle; 5th aug: the width of the circle border.

  show_frame(window_names[1], im_with_keypoints); // Show the detected keypoints and the RED leds

  return temp_keypoints;
}

vector<KeyPoint> drone_tracking::keypoint_filtering(vector<KeyPoint> in_keypoints, bool outdoor, Mat in_mask_red)
/*****************************************************************************
*   Input    : The 1st aug is the passed keypoints, the 2nd aug is whether or not the keypoints are from a secondary run / outdoor (some params change). Uses mask_circles
*   Output   : None (the frames are a part of the class)
*   Function : Filters the found keypoints
******************************************************************************/
{
  Mat mask_circles;
  Mat frame_temp;
  vector<KeyPoint> out_keypoints; // Vector to hold the sorted keypoints
  double mean_of_frame = 0; // Used to minimize calculations
  int red_diodes = 0; // Number of red LEDs
  if (in_keypoints.size()) // Test if there are any detected keypoints
  {
    for (size_t i = 0; i < in_keypoints.size(); i++) { // Run through all the keypoints
      mask_circles = in_mask_red.clone(); // Just to get the proper size for the new frame
      mask_circles = Scalar(0); // Clear the only channel on the circle mask frame
      circle(mask_circles, in_keypoints[i].pt, in_keypoints[i].size * 1+(hue_radius/100), Scalar(255), -1); // Draw a circle for the mask.
      //1st aug: in frame; 2nd aug: the centrum of the circle; 3rd aug: the circle radius; 4th aug: the color of the circle; 5th aug: the width of the circle border, when -1 it fills the cirle instead.
      frame_temp = Scalar(0); // Clear the only channel on the circle mask frame
      in_mask_red.copyTo(frame_temp, mask_circles); // Copy in_mask_red to frame_temp but only the area marked in the mask_circles
      mean_of_frame = (mean(frame_temp)[0]/(pow(in_keypoints[i].size,2)*M_PI))*mean_multiply_factor; // Calculate the mean of the frame using an openCV function. Calculation is relative since it takes the size into account.

      if (debug)
        cout << "Keypoint " << i << " has a mean red value of " << mean_of_frame << endl;

      int color_threashold; // Used to hold the right color_threashold. Used to (almost) avoid the same code
      if (!outdoor)
        color_threashold = color_threashold_1;
      else
        color_threashold = color_threashold_2;
      if (mean_of_frame > color_threashold) { // Test if it is a red LED; mean above threashold
        red_diodes++;
        out_keypoints.push_back(in_keypoints[i]); // Add the keypoint to the output list since it is a red LED
      }
    }
    if (debug)
      cout << endl; // Makes debug reading easier
  }
  return out_keypoints;
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

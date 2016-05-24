/*****************************************************************************
* University of Southern Denmark
* UAS Center
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: drone_tracking.hpp
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
#include "opencv2/photo.hpp"              // Højgaard does not know if he uses this - NOTE
#include <opencv2/video/background_segm.hpp> // Højgaard does not know if he uses this - NOTE
#include <vector>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <pthread.h>

#include "interthread_communication.hpp"

using namespace cv;
using namespace std;
/*****************************    Defines    *******************************/
// Diode  tracking
# define M_PI           3.14159265358979323846  /* pi */
# define HUE_ORANGE     11                      /* 0-22 */
# define HUE_YELLOW     30                      /* 22-38 */
# define HUE_GREEN      60                      /* 38-75 */
# define HUE_GREEN_LED  55                      /* 38-75 */
# define HUE_BLUE       100                      /* 75-130 */
# define HUE_VIOLET     145                      /* 130-160 */
# define HUE_RED        160                      /* 160-179; since the red color appears darker, more towards blue, 160 is choosen */

    // Diode relations (all realtions are caculated in regards to the distance between LED 1 and 2 which is 21.4 cm)
# define DIODE_1_2      1.00000000000
# define DIODE_1_3      1.58728971963
# define DIODE_1_4      1.08878504673

# define DIODE_2_1      DIODE_1_2
# define DIODE_2_3      DIODE_1_4
# define DIODE_2_4      DIODE_1_3

# define DIODE_3_1      DIODE_1_3
# define DIODE_3_2      DIODE_1_4
# define DIODE_3_4      1.29439252336

# define DIODE_4_1      DIODE_1_4
# define DIODE_4_2      DIODE_1_3
# define DIODE_4_3      DIODE_3_4

# define DIODE_3_4_SECONDARY  1.18884120172 // used if diode 1 or 2 is not visible
# define DIODE_4_3_SECONDARY  DIODE_3_4_SECONDARY

    // Center relation (all realtions are caculated in regards to the distance between LED 1 and 2 which is 21.4 cm)
# define DIODE_1_CENTER 0.67289719626
# define DIODE_2_CENTER DIODE_1_CENTER
# define DIODE_3_CENTER 0.91588785046
# define DIODE_4_CENTER DIODE_3_CENTER

    // Tolerances
# define DIODE_TOLERANCE  0.05 // Added to relation
# define CENTER_TOLERANCE 0.025 // Added to relation



// Drone shape tracking
  // Load shape
#define DATA_FILE_PATH      "shape_test_data.csv"
#define SHAPE_IM_PATH       "src/shape_dummy.jpg" // Path to dummy shape template
// #define SHAPE_IM_PATH       "src/shape_other_no_bg.png"   // Path to plywood shape template
#define SHAPE_CONTOUR_INDEX 0               // Index for shape contour in shape_contours. 0 for dummy. 1 for plywood.

  // Erosion and dilation
#define EROSION_TYPE        MORPH_ELLIPSE // From example in link in erode/dilate function: A filled ellipse
#define EROSION_SIZE        1             // From example in link in erode/dilate function: Based on examples online and Stigs code
#define ERODE_ITERATIONS    2             // From example in link in erode/dilate function: Based on examples online and Stigs code
#define DILATION_TYPE       MORPH_ELLIPSE // From example in link in erode/dilate function: A filled ellipse
#define DILATION_SIZE       EROSION_SIZE  // From example in link in erode/dilate function
#define DILATE_ITERATIONS   2

  // Thresholding image
#define THRESH_THRESH       60
#define THRESH_MAXVAL       255
#define THRESH_TYPE         THRESH_BINARY_INV // If src(x,y)>TRESH_TRESH: src(x,y)=0 else THRESH_MAXVAL // Chosen because it is used in Stig Turner's code.

#define SHAPE_FOUND_THRESH  45             // Value below is a match
#define MINIMUM_DRONE_SIZE  500             // For discarding too small contours            // Value below is a match
#define MAXIMUM_DRONE_SIZE  10000
 // For test
 #define SAVE_FRAME_NUM     150           // Frame that is saved in frame_analysis

typedef struct xy_position{   // Struct for xy-position of drone
   double x;
   double y;
   double orientation;        // Might not be used. Not stable yet.
 } xy_position;

 typedef struct xyz_position{   // Struct for xy-position of drone
    double x;                   //Horizontal plane is x and y
    double y;                   //  ---""---
    double z;                   // Distance above ground station
    double orientation;        // Might not be used. Not stable yet.
    double orientation_old;
    double orientation_old_old;
    double orientation_length_factor;
    int orientation_run;
  } xyz_position;


/*****************************   Class   *******************************/
class drone_tracking
{
public: // Methods
  drone_tracking();
  drone_tracking(string);
private: // Methods
  void show_frame(string, bool, Mat); //Use window_names to point to the name
  void frame_analysis();
  void create_windows();
  void frame_save(Mat&);
  void frame_save(Mat&, string);
  vector<KeyPoint> diode_detection();
  bool find_position(vector<KeyPoint>, xyz_position&);
  double calc_dist(KeyPoint, KeyPoint);
  //double get_orientation(Mat);
  vector<KeyPoint> keypoint_detection(Mat, Mat, Mat);
  vector<KeyPoint> keypoint_filtering(vector<KeyPoint>, bool, Mat);
  void window_taskbar_create(int);

  // Drone shape detection methods
  void simple_shape_tracking();

  // Match shape methods
  //xy_position get_drone_position(Mat);
  bool get_drone_position(Mat, xy_position *, Mat *);
  void load_shape_im();
  vector<vector<Point>> get_contours(Mat);
  Mat local_erode(Mat);
  Mat local_dilate(Mat);
  void get_position(vector<Point> contour, xy_position *);
  //void handle_trackbars();

private: // Variables
  string filename;
  Mat frame_bgr;
  VideoCapture capture;

 // General variables
  bool enable_wait = false;
  int wait_time_ms = 50;
  bool debug = false;
  bool debug_frame_num = true;
  int global_frame_counter = 0;
  int start_skip_frames = 0;

 // show_frame variables
  bool window_enable = true;
  vector<string> window_names; // Holds the window names but no values can be added here, must be added in the method.
  vector<bool> window_show; // Holds a flag for showning (true) or not showing each window. Assign value in create_window and use in show_frame.
  bool custom_window_size = true;
  int custom_window_width = 400; // Custom but ensure the proper aspect ratio for the camera
  int custom_window_height = 300;
  int screen_dimension_width = 1280; //HD: 1080; FULL-HD: 1920; Other: 1280
  int screen_dimension_height = 800; //HD: 800 (or 720); FULL-HD: 1200 (or 1080); Other: 800
  bool enable_trackbars = true;

 // frame_save variables
  int frame_save_counter = 1;
  string frame_save_type = "png"; // File extension for saving frames

 // Diode detection Variables
  // HSV limits for color seperation (presumably changable)
  int hsv_h_base        = HUE_GREEN_LED; //60 is for green, 160 is for red
  int hsv_h_sensitivity = 15; // 5 for big green led and 24 for most other
  int hsv_h_low         = 40;//hsv_h_base - hsv_h_sensitivity;
  int hsv_h_upper       = 100;//hsv_h_base + hsv_h_sensitivity;
  int hsv_s_low         = 50;//100;
  int hsv_s_upper       = 255;
  int hsv_v_low         = 50;//100;
  int hsv_v_upper       = 255;
  int gaussian_blur         = 3; //Must be positive and odd. Higher, more blur
  bool enable_gaussian_blur = false;
  int dilate_color_iterations = 4;

  // Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;
  bool second_detection_run = false;

  int mean_multiply_factor = 1000000; //Effects the one below linear
  int color_threashold_1 = 60;
  int color_threashold_2 = 40;
  int hue_radius = 10; // [%]

  vector<KeyPoint> leds; // Stores the red LED keypoints
  xyz_position diode_drone;
  int orientation_max_change = 20;
  string diode_tracking_filename = "data_diode_tracking.csv";
  ofstream data_diode_tracking;
  bool log_diode_tracking = false;
  int detected_keypoints;
  bool diode_save_frame = false;
  int diode_save_frame_num = 0; //664

  // MatchShape variables
  bool shape_loaded = false;        // True if shape is loaded
  vector<vector<Point>> shape_contours, frame_contours; // Contours
  vector<Vec4i> shape_hierarchy;    // Hierarchy
  int frame_number = 0;             // Frame number for debug

  int erosion_type = EROSION_TYPE;
  int erosion_size = EROSION_SIZE;
  int erode_iterations = ERODE_ITERATIONS;
  int dilation_type = DILATION_TYPE;
  int dilation_size = DILATION_SIZE;
  int dilate_iterations = DILATE_ITERATIONS;
  int thresh_tresh = THRESH_THRESH;
  int shape_found_thresh = SHAPE_FOUND_THRESH;
  int minimum_drone_size = MINIMUM_DRONE_SIZE;


  bool wait_enable = false;
  int wait_time = 0;

  // Match shape constants
  Scalar color_green = Scalar(0,255,0), color_red = Scalar(0,0,255);
  Scalar color_white = Scalar(255,255,255), color_black = Scalar(0,0,0);

  ofstream shape_data;

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
*   Input    : Filename
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
    shape_data.open(DATA_FILE_PATH);
    if(!shape_data.is_open())
    {
      cout << "Unable to open data file" << endl;
      exit(1);
    }
    capture.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    cout << "Capture is opened" << endl;
    create_windows();
    if (log_diode_tracking) {
      data_diode_tracking.open (diode_tracking_filename);
      data_diode_tracking << "frame,drone_detected,detected_keypoints,diodes,orientation[deg],orientation_length_factor[pixel],orientation_run[1=top,2=bottom]\n";
    }
    for(;;) { // Processing
      capture >> frame_bgr;
      if(frame_bgr.empty())
        break;
      if (!frame_bgr.empty())
        if (start_skip_frames < global_frame_counter)
          frame_analysis(); // Master method for analysis
      global_frame_counter++;
      if (debug) {
        cout << "The frame dimensions are:" << frame_bgr.cols << "x" << frame_bgr.rows << endl;
      }
      if(waitKey(10) >= 0)
        break;
    }
    if (log_diode_tracking)
      data_diode_tracking.close();
    shape_data.close();
  } else { // Error capture is not opened
    cout << "No capture to open" << endl;
    frame_bgr = Mat::zeros(480, 640, CV_8UC1);
    show_frame("Failed to open", true, frame_bgr);
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
  // Input frame
  window_names.push_back("Input stream"); window_show.push_back(true); //Window 0 *

  // Diode detection
  window_names.push_back("Recognized LEDs"); window_show.push_back(true); //Window 1 *
  window_names.push_back("Color mask"); window_show.push_back(true); //Window 2 *
  window_names.push_back("Color seperation frame"); window_show.push_back(true); //Window 3 *
  //window_names.push_back("Other2"); window_show.push_back(true);//Window 4
  //window_names.push_back("Other3"); window_show.push_back(true);//Window 5

  // Shape detection
  window_names.push_back("Drone shape"); window_show.push_back(false); //Window 4 *
  window_names.push_back("Frame contours"); window_show.push_back(false); //Window 5
  window_names.push_back("Tracking"); window_show.push_back(false); //Window 6 *
  window_names.push_back("Drone masked out, inside"); window_show.push_back(false); //Window 7 *
  window_names.push_back("Shape frame"); window_show.push_back(false); //Window 8
  window_names.push_back("Contours on shape frame"); window_show.push_back(false); //Window 9
  window_names.push_back("Contour0"); window_show.push_back(false); //Window 10
  window_names.push_back("Contour1"); window_show.push_back(false); //Window 11
  window_names.push_back("Thresholded frame"); window_show.push_back(false); //Window 12
  window_names.push_back("Erode"); window_show.push_back(false); //Window 13
  window_names.push_back("Dilate"); window_show.push_back(false); //Window 14
  window_names.push_back("Settings"); window_show.push_back(false); //Window 15
  window_names.push_back("Contourx"); window_show.push_back(false); // 16
  //window_names.push_back("Window N"); window_show.push_back(true); //Window N
  if (window_enable)
  {
    int j = 0; // Secondary row position counter
    for (size_t i = 0; i < window_names.size(); i++) { // Loop through all the windows
      if (window_show[i])
      {
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
}

void drone_tracking::window_taskbar_create(int window_number)
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Creates the specified task bars
******************************************************************************/
{
  if (window_number==1){ // Test which window window_taskbar_create is called with
     createTrackbar("Threashold", window_names[window_number], &color_threashold_1, 1000); // 1st arg: name; 2nd arg: window; 3rd arg: pointer to the variabel (must be int); 4th arg: max value
     createTrackbar("Detection radius", window_names[window_number], &hue_radius, 255);
  }
  if (window_number==2){ // Test which window window_taskbar_create is called with
      createTrackbar("HUE GREEN LOW", window_names[window_number], &hsv_h_low, 255);
      createTrackbar("HUE GREEN UPPER", window_names[window_number], &hsv_h_upper, 255);
      createTrackbar("SATURATION LOW", window_names[window_number], &hsv_s_low, 255);
      createTrackbar("SATURATION UPPER", window_names[window_number], &hsv_s_upper, 255);
      createTrackbar("VIBRANCE LOW", window_names[window_number], &hsv_v_low, 255);
      createTrackbar("VIBRANCE UPPER", window_names[window_number], &hsv_v_upper, 255);
  }
  if (window_number==3) // Test which window window_taskbar_create is called with
     createTrackbar("Dilate iterations", window_names[window_number], &dilate_color_iterations, 10); // 1st arg: name; 2nd arg: window; 3rd arg: pointer to the variabel (must be int); 4th arg: max value
  if (window_number==15)
  {
    createTrackbar("erosion_type", window_names[window_number], &erosion_type, 2);
    createTrackbar("erosion_size", window_names[window_number], &erosion_size, 20);
    createTrackbar("erode_iterations", window_names[window_number], &erode_iterations, 20);
    createTrackbar("dilation_type", window_names[window_number], &dilation_type, 2);
    createTrackbar("dilation_size", window_names[window_number], &dilation_size, 20);
    createTrackbar("dilate_iterations", window_names[window_number], &dilate_iterations, 20);
    createTrackbar("thresh_thresh", window_names[window_number], &thresh_tresh, 255);
    createTrackbar("match treshold",window_names[window_number], &shape_found_thresh,200);
    createTrackbar("drone minimum size",window_names[window_number], &minimum_drone_size,2000);
  }

/*
  int erosion_type = EROSION_TYPE;
  int erosion_size = EROSION_SIZE;
  int erode_iterations = ERODE_ITERATIONS;
  int dilation_type = DILATION_TYPE;
  int dilation_size = DILATION_SIZE;
  int dilate_iterations = DILATE_ITERATIONS;
  int thresh_tresh = THRESH_THRESH;
*/
}

void drone_tracking::show_frame(string window_text, bool show_this_frame, Mat in_frame)
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Displays the given frame
******************************************************************************/
{
  if (window_enable)
    if (show_this_frame)
      if (!in_frame.empty())
        imshow(window_text, in_frame); // 1st arg: window name; 2nd arg: frame to show
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
  // SHOWING THE INPUT FRAME BELOW
  if (debug_frame_num)
    cout << endl << "Frame: " << global_frame_counter << endl;

  leds = diode_detection();
  cout << "Found diodes: " << leds.size() << endl;
  // Calculate orientation
  bool force_down = false;
  double scale_factor;
  double temp_scale_factor = 0;
  int drone_detected2 = 0;
  int diodes_inside = 0;
  double rect_p_gain = 1.10;
  int x_mid = 1280/2;
  int y_mid = 720/2;
  int x_base = 400;
  int y_base = 300;
  if (leds.size() > 0) {
    for (size_t i = 0; i < leds.size() - 1; i++) {
      for (size_t j = i + 1; j < leds.size(); j++) {
        temp_scale_factor += calc_dist(leds[i], leds[j]);
      }
    }
    temp_scale_factor /= leds.size();
    cout << temp_scale_factor << endl;
    scale_factor = temp_scale_factor/4000;
    cout << scale_factor << endl;

    for (size_t i = 0; i < leds.size(); i++) {
      if ((leds[i].pt.x > x_mid-((x_base*(1+scale_factor)*rect_p_gain)/2) and leds[i].pt.x < x_mid+((x_base*(1+scale_factor)*rect_p_gain)/2)) and
      (leds[i].pt.y > y_mid-((y_base*(1+scale_factor)*rect_p_gain)/2) and leds[i].pt.y < y_mid+((y_base*(1+scale_factor)*rect_p_gain)/2))) {
        cout << "led inside: " << i << endl;
        diodes_inside++;
      }
    }

    if (find_position(leds, diode_drone)) {
       if (diode_drone.orientation < diode_drone.orientation - orientation_max_change and diode_drone.orientation > diode_drone.orientation + orientation_max_change) {
         diode_drone.orientation = diode_drone.orientation_old;
       } else { // Drone has been found and an angle has been calculated!
         drone_detected2 = 1;
       }
    }

    if(false)
      cout << "Drone orientation is "<< diode_drone.orientation << endl;

  }
  if (diodes_inside == leds.size() and leds.size() >= 3) {
    rectangle( frame_bgr, Point( x_mid-((x_base*(1+scale_factor)*rect_p_gain)/2), y_mid-((y_base*(1+scale_factor)*rect_p_gain)/2) ), Point( x_mid+((x_base*(1+scale_factor)*rect_p_gain)/2), y_mid+((y_base*(1+scale_factor)*rect_p_gain)/2)), Scalar( 0, 255, 0 ), +1, 4 );
    force_down = true;
  } else {
    rectangle( frame_bgr, Point( x_mid-((x_base*(1+scale_factor)*rect_p_gain)/2), y_mid-((y_base*(1+scale_factor)*rect_p_gain)/2) ), Point( x_mid+((x_base*(1+scale_factor)*rect_p_gain)/2), y_mid+((y_base*(1+scale_factor)*rect_p_gain)/2)), Scalar( 0, 0, 255 ), +1, 4 );
  }

  if (log_diode_tracking) {
    if (leds.size() > 0) {
      data_diode_tracking << global_frame_counter << "," << drone_detected2 << "," << detected_keypoints << "," << leds.size() << "," << diode_drone.orientation << "," << diode_drone.orientation_length_factor << "," << diode_drone.orientation_run << "\n" ;
    } else {
      data_diode_tracking << global_frame_counter << ",0,0,0,0,0\n";
    }
  }

  show_frame(window_names[0], window_show[0], frame_bgr); // Show original frame
  if (global_frame_counter == diode_save_frame_num and diode_save_frame)
    frame_save(frame_bgr, "frame_bgr_"+to_string(diode_save_frame_num));


  // Hejgaard analysis
  //locate_uav(frame_bgr);
  //simple_shape_tracking();
  xy_position position_from_shape;    // For position returned
  Mat shape_frame;                    // Frame with only drone masked out
  shape_frame = Mat::zeros( frame_bgr.size(), CV_8UC3 );

  bool drone_detected = get_drone_position(frame_bgr,&position_from_shape, &shape_frame); // Get the position
  show_frame(window_names[4], window_show[4], shape_frame);

  shape_data << frame_number << "," << drone_detected << "," << position_from_shape.x << "," << position_from_shape.y << endl;

  // Put someting in the socket packet;
  pthread_mutex_lock(&mutex_sock_pack_out);
  sock_pack_out.field0 = force_down;
  sock_pack_out.field1 = 1;
  sock_pack_out.field2 = 2;
  sock_pack_out.field3 = 3;
  sock_pack_out.field4 = 4;
  sock_pack_out.field5 = 5;
  sock_pack_out.field6 = 6;
  sock_pack_out.field7 = 7;
  sock_pack_out.field8 = 8;
  sock_pack_out.field9 = 9;
  sock_pack_out.field10 = 10;
  sock_pack_out.field11 = 11;
  sock_pack_out.field12 = 12;
  sock_pack_out.field13 = 13;
  sock_pack_out.field14 = 14;
  sock_pack_out.field15 = 15;
  sock_pack_out.field16 = 16;
  sock_pack_out.field17 = 17;
  sock_pack_out.field18 = 18;
  sock_pack_out.field19 = 19;
  pthread_mutex_unlock(&mutex_sock_pack_out);

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
  frame_save(frame_in, name); // 1st arg: frame to save; 2nd arg: file name
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
    imwrite( "./output/images/"+name_in, frame_in ); // 1st arg: path and name; 2nd arg: frame to save
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
  if (enable_gaussian_blur)
    GaussianBlur(frame_gray, frame_gray_with_Gblur, Size(gaussian_blur, gaussian_blur), 0); // Gaussian blur on gray frame, 1st arg: input frame; 2nd arg: output frame; 3rd arg: defines the blur radius; 4th arg: Gaussian kernel standard deviation in X direction, when this is 0 it is computed from the 3rd arg. Gaussaian blur is used since an example used this.
  else
    frame_gray.copyTo(frame_gray_with_Gblur);

  inRange(frame_hsv, Scalar(hsv_h_low,hsv_s_low,hsv_v_low), Scalar(hsv_h_upper, hsv_s_upper, hsv_v_upper), mask_red); // Find the areas which contain red color. 1st arg: inpur frame; 2nd arg: the lower HSV limits; 3rd arg: the upper HSV limits; 4th arg: the output mask.
  frame_red = Scalar(0); // Clear the red frame.
  for (size_t i = 0; i < dilate_color_iterations; i++)
    dilate(mask_red, mask_red, Mat(), Point(-1,-1)); // Enhance the red areas in the image
  frame_bgr.copyTo(frame_red, mask_red); // Copy frame_bgr to frame_red but only the area marked in the red_mask
  //cout << "Orientation: " << get_orientation(mask_red) << endl;

  vector<KeyPoint> detected_leds;
  detected_leds = keypoint_detection(frame_gray, frame_gray_with_Gblur, mask_red); // Detect the red LEDs

  if (debug) {
    cout << "There are " << detected_leds.size() << " LEDs" << endl;
    if (second_detection_run)
      cout << "** secondary_detection_run **" << endl;
  }

  show_frame(window_names[2], window_show[2], frame_red); // Show the frame only containing the red color
  show_frame(window_names[3], window_show[3], mask_red); // Show the red masking frame
  if (global_frame_counter == diode_save_frame_num and diode_save_frame) {
    frame_save(frame_red, "frame_red_"+to_string(diode_save_frame_num));
    frame_save(mask_red, "frame_mask_"+to_string(diode_save_frame_num));
  }

  return detected_leds;
}

vector<KeyPoint> drone_tracking::keypoint_detection(Mat in_frame_gray, Mat in_frame_gray_with_Gblur, Mat in_mask_red)
/*****************************************************************************
*   Input    : The 3 frame arguments are frames nessecary for the frame analysis.
*   Output   : Vector containing keypoints of the desired LED color.
*   Function : Finds brighest points and thereby LEDs (if they are bright enough compared to the rest of the frame)
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
  params.minArea = 5;
  params.maxArea = 600;
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
  detected_keypoints = keypoints.size(); // Used for logging
  temp_keypoints = keypoint_filtering(keypoints, second_detection_run, in_mask_red); // Filter the detected keypoints

  if (!temp_keypoints.size()) { // See if any keypoints matched a red LED. If not then detect using different frame (mask_red)
    keypoints.clear(); // Delete the old keypoints to hold the new ones from the detector.
    detector->detect(in_mask_red, keypoints); // Detect the keyframes from the frame
    second_detection_run = true; // Tell the other functions that the keypoints were found from the secondary frame
    temp_keypoints = keypoint_filtering(keypoints, second_detection_run, in_mask_red); // Filter the detected keypoints
  } else
    second_detection_run = false; // Tell the other functions that the keypoints were found from the primary frame

  im_with_keypoints = Scalar(0,0,0); // Clear all the channels in the image
  drawKeypoints( frame_bgr, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS ); // Show the keypoints on a frame. 1st arg: input frame; 2nd arg: vector with keypoints to be drawn; 3rd arg: output frame; 4th arg: color of the drawn keypoints (red here); 5th arg: For each keypoint, the circle around keypoint with keypoint size and orientation will be drawn.
  for (size_t i = 0; i < temp_keypoints.size(); i++) // Run through all the red LED keypoints.
    circle(im_with_keypoints, temp_keypoints[i].pt, temp_keypoints[i].size, Scalar(255, i*40, i*20), temp_keypoints[i].size*(1+(hue_radius/100))); // Draw the right keypoints (the red LEDs). 1st arg: in frame; 2nd arg: the centrum of the circle; 3rd arg: the circle radius; 4th arg: the color of the circle; 5th arg: the width of the circle border.
  for (size_t i = 0; i < temp_keypoints.size(); i++) {
    putText(im_with_keypoints, to_string(i), temp_keypoints[i].pt,
    FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,0,255), 1, CV_AA);
  }
  show_frame(window_names[1], window_show[1], im_with_keypoints); // Show the detected keypoints and the RED leds

  if (global_frame_counter == diode_save_frame_num and diode_save_frame)
    frame_save(im_with_keypoints, "frame_keypoints_"+to_string(diode_save_frame_num));

  return temp_keypoints;
}

/*
double drone_tracking::get_orientation(Mat red_mask_local)
{
  Mat mask_grey;
  red_mask_local.copyTo(mask_grey);
  Moments moment = moments(mask_grey,false);  // Calculate moments. Arguments - 1st: a contour, 2nd: binary image, if true all nonzero pixels are treated as 1's
  //pos->x = moment.m10/moment.m00;            // Calculate x position
  //pos->y = moment.m01/moment.m00;            // Calculate y position
  // Calculate orientation (may not be correct). Based on link above
  double orientation = 0.5 * atan(2 * moment.m11 / (moment.m20 - moment.m02));
  double orientation_out = (orientation / M_PI) * 180;

  return orientation_out;
}
*/

vector<KeyPoint> drone_tracking::keypoint_filtering(vector<KeyPoint> in_keypoints, bool in_secondary_detection_run, Mat in_mask_red)
/*****************************************************************************
*   Input    : The 1st arg is the passed keypoints, the 2nd arg is whether or not the keypoints are from a secondary run / secondary_detection_run (some params change); 3rd arg: the red mask frame.
*   Output   : Vector containing keypoints of the desired LED color
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
      circle(mask_circles, in_keypoints[i].pt, in_keypoints[i].size * (1+(hue_radius/100)), Scalar(255), -1); // Draw a circle for the mask.
      //1st arg: in frame; 2nd arg: the centrum of the circle; 3rd arg: the circle radius; 4th arg: the color of the circle; 5th arg: the width of the circle border, when -1 it fills the cirle instead.
      frame_temp = Scalar(0); // Clear the only channel on the circle mask frame
      in_mask_red.copyTo(frame_temp, mask_circles); // Copy in_mask_red to frame_temp but only the area marked in the mask_circles
      mean_of_frame = (mean(frame_temp)[0]/(pow(in_keypoints[i].size,2)*M_PI))*mean_multiply_factor; // Calculate the mean of the frame using an openCV function. Calculation is relative since it takes the size into account.

      if (debug or true)
        cout << "Keypoint " << i << " has a mean value of " << mean_of_frame << endl;

      int color_threashold; // Used to hold the right color_threashold. Used to (almost) avoid the same code
      if (!in_secondary_detection_run)
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

  bool drone_tracking::find_position(vector<KeyPoint> diode_keypoints, xyz_position& drone_position)
/*****************************************************************************
*   Input    :
*   Output   :
*   Function :
******************************************************************************/
{
  if (diode_keypoints.size() < 3) {
    return false;
  } else if (diode_keypoints.size() >= 3) {
    // Find the smallest relation
    int min_dist1_diode1, min_dist1_diode2;
    double min_dist1 = pow(10,10); // Just defined as a very high number for the first test
    double temp_dist;
    for (size_t i = 0; i < diode_keypoints.size() - 1; i++) {
      for (size_t j = i + 1; j < diode_keypoints.size(); j++) {
        temp_dist = calc_dist(diode_keypoints[i], diode_keypoints[j]);
        if (temp_dist < min_dist1) {
          min_dist1 = temp_dist;
          min_dist1_diode1 = i;
          min_dist1_diode2 = j;
        }
      }
    }
    /*
    cout << "Smallest relation test" << endl;
    cout << "Min diode 1: " << min_dist1_diode1 << endl;
    cout << "Min diode 2: " << min_dist1_diode2 << endl;
    cout << "Min dist is: " << min_dist1 << endl;
    */
    // Determine diode 3 or 4
    int diode1_3or4 = diode_keypoints.size()+1; // The diode from the smallest relation and related to diode 3 or 4. Set to that value so we can check if it is found
    int diode2_3or4 = diode_keypoints.size()+1; // Actual diode 3 or 4. Set to that value so we can check if it is found
    for (size_t i = 0; i < diode_keypoints.size(); i++) {
      if (i != min_dist1_diode1 and i != min_dist1_diode2) {
        temp_dist = calc_dist(diode_keypoints[min_dist1_diode1], diode_keypoints[i]);
        if (temp_dist/min_dist1 > DIODE_1_4-DIODE_TOLERANCE and temp_dist/min_dist1 < DIODE_1_4+DIODE_TOLERANCE) {
          diode1_3or4 = min_dist1_diode1;
          diode2_3or4 = i;
          //cout << "Relation is: " << temp_dist/min_dist1 << endl;
        }
      }
    }
    /*
    if (diode1_3or4 < diode_keypoints.size() and diode1_3or4 < diode_keypoints.size()) {
      cout << "Bottom relation" << endl;
      cout << "Ref diode: " << diode1_3or4 << endl;
      cout << "Diode 3 or 4: " << diode2_3or4 << endl;
    } else {
      cout << " --- no bottom relation found ---" << endl;
    }
    */
    if (diode2_3or4 > diode_keypoints.size() ) { // If the diode is not found it looks through the other diodes relations
      for (size_t i = 0; i < diode_keypoints.size(); i++) {
        if (i != min_dist1_diode1 and i != min_dist1_diode2) {
          temp_dist = calc_dist(diode_keypoints[min_dist1_diode2], diode_keypoints[i]);
          if (temp_dist/min_dist1 > DIODE_2_3-DIODE_TOLERANCE and temp_dist/min_dist1 < DIODE_2_3+DIODE_TOLERANCE) {
            diode1_3or4 = min_dist1_diode2;
            diode2_3or4 = i;
          }
        }
      }
      if (diode2_3or4 > diode_keypoints.size()) { // Diode 1 or 2 has not been found so the relation is another
        for (size_t i = 0; i < diode_keypoints.size(); i++) {
          if (i != min_dist1_diode1 and i != min_dist1_diode2) {
            temp_dist = calc_dist(diode_keypoints[min_dist1_diode1], diode_keypoints[i]);
            if (temp_dist/min_dist1 > DIODE_3_4_SECONDARY-DIODE_TOLERANCE and temp_dist/min_dist1 < DIODE_3_4_SECONDARY+DIODE_TOLERANCE) {
              diode1_3or4 = min_dist1_diode1;
              diode2_3or4 = i;
            }
          }
        }
        if (diode2_3or4 > diode_keypoints.size()) { // Continue on the relation test
          for (size_t i = 0; i < diode_keypoints.size(); i++) {
            if (i != min_dist1_diode1 and i != min_dist1_diode2) {
              temp_dist = calc_dist(diode_keypoints[min_dist1_diode2], diode_keypoints[i]);
              if (temp_dist/min_dist1 > DIODE_3_4_SECONDARY-DIODE_TOLERANCE and temp_dist/min_dist1 < DIODE_3_4_SECONDARY+DIODE_TOLERANCE) {
                diode1_3or4 = min_dist1_diode2;
                diode2_3or4 = i;
              }
            }
          }
        }
        if (diode2_3or4 > diode_keypoints.size()) { // no relation was found during secondary analysis, retuning false
          return false;
        } else {
          // Find rotation for 3 diodes
          double delta_x, delta_y;
          if (diode_keypoints[diode1_3or4].pt.x > diode_keypoints[diode2_3or4].pt.x) {
            delta_x = diode_keypoints[diode1_3or4].pt.x - diode_keypoints[diode2_3or4].pt.x;
            delta_y = diode_keypoints[diode1_3or4].pt.y - diode_keypoints[diode2_3or4].pt.y;
          } else {
            delta_x = diode_keypoints[diode2_3or4].pt.x - diode_keypoints[diode1_3or4].pt.x;
            delta_y = diode_keypoints[diode2_3or4].pt.y - diode_keypoints[diode1_3or4].pt.y;
          }
          drone_position.orientation_old_old = drone_position.orientation_old;
          drone_position.orientation_old = drone_position.orientation;
          drone_position.orientation = (atan2(delta_y, delta_x) * 180 / M_PI);// - 98.945;
          drone_position.orientation_run = 2;
          drone_position.orientation_length_factor = temp_dist;
          line( frame_bgr, diode_keypoints[diode1_3or4].pt, diode_keypoints[diode2_3or4].pt, Scalar( 102, 0, 204 ));
          /*
          bool rot_cor = false; // rotation correction
          for (size_t i = 0; i < diode_keypoints.size(); i++)
            if ((i != diode1_3or4 or i != diode2_3or4) and !rot_cor) {
              if (diode_keypoints[i].pt.y < diode_keypoints[diode1_3or4].pt.y and
                diode_keypoints[i].pt.y < diode_keypoints[diode2_3or4].pt.y) {
                rot_cor = true;
              }
            }
          if (rot_cor)
            drone_position.orientation += 90;
          else
            drone_position.orientation += 270;
          */

          //cout << "[2] Drone has orientation: " << drone_position.orientation << " (calculated from point " << diode1_3or4 << " and " << diode2_3or4 << ")" << endl;
          //cout << " [2] Diode1: " << diode1_3or4 << ": " << diode_keypoints[diode1_3or4].pt.x << ", " << diode_keypoints[diode1_3or4].pt.y << endl;
          //cout << " [2] Diode2: " << diode2_3or4 << ": " << diode_keypoints[diode2_3or4].pt.x << ", " << diode_keypoints[diode2_3or4].pt.y << endl;

          // Find position for 3 diodes
          return true;
        }
      }
    } else {
      // Find rotation for 4 diodes
      double delta_x, delta_y;
      if (diode_keypoints[min_dist1_diode1].pt.x > diode_keypoints[min_dist1_diode2].pt.x) {
        delta_x = diode_keypoints[min_dist1_diode1].pt.x - diode_keypoints[min_dist1_diode2].pt.x;
        delta_y = diode_keypoints[min_dist1_diode1].pt.y - diode_keypoints[min_dist1_diode2].pt.y;
      } else {
        delta_x = diode_keypoints[min_dist1_diode2].pt.x - diode_keypoints[min_dist1_diode1].pt.x;
        delta_y = diode_keypoints[min_dist1_diode2].pt.y - diode_keypoints[min_dist1_diode1].pt.y;
      }
      drone_position.orientation_old_old = drone_position.orientation_old;
      drone_position.orientation_old = drone_position.orientation;
      drone_position.orientation = atan2(delta_y, delta_x) * 180 / M_PI;
      drone_position.orientation_run = 1;
      drone_position.orientation_length_factor = temp_dist;
      line( frame_bgr, diode_keypoints[min_dist1_diode1].pt, diode_keypoints[min_dist1_diode2].pt, Scalar( 102, 0, 204 ));

      /*
      bool rot_cor = false; // rotation correction
      for (size_t i = 0; i < diode_keypoints.size(); i++)
        if ((i != min_dist1_diode1 or i != min_dist1_diode2) and !rot_cor) {
          if (diode_keypoints[i].pt.y < diode_keypoints[min_dist1_diode1].pt.y and
            diode_keypoints[i].pt.y < diode_keypoints[min_dist1_diode2].pt.y) {
            rot_cor = true;
          }
        }
      if (rot_cor)
        drone_position.orientation += 270;
      else
        drone_position.orientation += 90;
      */

      /*
      double temp_y = 0;
      for (size_t i = 0; i < diode_keypoints.size(); i++)
        if (i != min_dist1_diode1 or i != min_dist1_diode2)
          temp_y += diode_keypoints[i].pt.y;
      temp_y /= (diode_keypoints.size()-2);
      if (temp_y < ((diode_keypoints[min_dist1_diode1].pt.y + diode_keypoints[min_dist1_diode2].pt.y)/2))
        drone_position.orientation += 180;
      */

      //cout << "[1] Drone has orientation: " << drone_position.orientation << " (calculated from point " << min_dist1_diode1 << " and " << min_dist1_diode2 << ")" << endl;
      //cout << " [1] Diode1: " << min_dist1_diode1 << ": " << diode_keypoints[min_dist1_diode1].pt.x << ", " << diode_keypoints[min_dist1_diode1].pt.y << endl;
      //cout << " [1] Diode2: " << min_dist1_diode2 << ": " << diode_keypoints[min_dist1_diode2].pt.x << ", " << diode_keypoints[min_dist1_diode2].pt.y << endl;

      // Find position for 4 diodes
      //Calculate position
      return true;
    }
    // CODE
  } /*else if (diode_keypoints >= 4) {

  }*/

  /*
  // Tests - start
  drone_position.x = 0;
  cout << "Y: " << drone_position.y << endl << endl;
  if (diode_keypoints.size()) {
    cout << "Y: " << diode_keypoints[0].pt.x << endl << endl;
  }
  */
  // Tests - end
  return true;
}

double drone_tracking::calc_dist(KeyPoint point1, KeyPoint point2)
/*****************************************************************************
*   Input    :
*   Output   :
*   Function :
******************************************************************************/
{
  return sqrt(pow((point2.pt.x-point1.pt.x),2)+pow((point2.pt.y-point1.pt.y),2));
}

// Drone shape tracking methods


void drone_tracking::simple_shape_tracking()
{
  Mat frame_gray, black_mask1, black_mask1_blurred;
  int black_low = 0;
  int black_high = 40;



  //cvtColor(frame_bgr, frame_gray, CV_BGR2GRAY);
  //findContours(frame_gray,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

  inRange(frame_bgr, Scalar(black_low,black_low,black_low), Scalar(black_high,black_high,black_high),black_mask1);
  //show_frame("black_mask",black_mask1);
  //blur(black_mask1, black_mask1_blurred,Size(25,25));
  //show_frame("black_mask blurred",black_mask1_blurred);
  //inRange(black_mask1_blurred, Scalar(black_low,black_low,black_low), Scalar(black_high,black_high,black_high),black_mask1);
  //show_frame("new black_mask ",black_mask1_blurred);

  Mat frame_local, frame_local2;
  frame_bgr.copyTo(frame_local);
  frame_bgr.copyTo(frame_local2);


  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;

  //cvtColor(frame_bgr,frame_gray,CV_BGR2GRAY);

  int thresh = 150;
  Mat canny_result;
  Canny(frame_bgr,canny_result, thresh,thresh*2);
  //show_frame("Canny result",canny_result);

  findContours(canny_result,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
  //drawContours(frame_local, contours, int contourIdx, const Scalar &color{#, int thickness{#, int lineType{#, InputArray hierarchy{#, int maxLevel{#, Point offset#}#}#}#}#})

  Scalar color=Scalar(0,255,0);
  for(int i;i<contours.size();i++)
  {

    drawContours(frame_local, contours, i, color, 5, 8, hierarchy, 0, Point(0,0));
  }

  bool foundOne = false;
  int biggest_index1 = -1;
  double area, biggest_area = 0;
  for(int i=0;i<contours.size();i++)
  {
    area = contourArea(contours[i]);
    if( area > biggest_area)
    {
      biggest_area = area;
      biggest_index1 = i;
      foundOne = true;
    }
  }

  drawContours(frame_local2, contours, biggest_index1, color, 5, 8, hierarchy, 0, Point(0,0));

  cout << contours.size() << endl;

  namedWindow("Contours",CV_WINDOW_FREERATIO);
  show_frame("Contours", true,frame_local);

  namedWindow("Biggest contour",CV_WINDOW_FREERATIO);
  show_frame("Biggest contour", true, frame_local2);

  // Moments and mass centers
  // Inspiration: http://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html
  vector<Moments> moment(contours.size() );
  for(int i = 0; i < contours.size(); i++)
  {
    moment[i] = moments(contours[i],false);
  }

  color = Scalar(0,0,255);
  vector<Point2f> mass_center(contours.size());
  for(int i = 0; i < contours.size(); i++)
  {
    mass_center[i] = Point2f(moment[i].m10/moment[i].m00,moment[i].m01/moment[i].m00);
  }

  circle( frame_local2, mass_center[biggest_index1], 4, color, -1, 8, 0 );

  //namedWindow("Center",CV_WINDOW_FREERATIO);
  show_frame("Center", true, frame_local2);

}


//xy_position drone_tracking::get_drone_position(Mat src_frame_in)
bool drone_tracking::get_drone_position(Mat src_frame_in, xy_position *position_out, Mat *frame_out)
/*****************************************************************************
*   Input    : The current frame as a Mat
*   Output   : Struct with position
*   Function : Finds position of the drone in the frame by matching contours
*              in the frame with the contour of the drone from a template image.
*              Written with inspiration from/based on code written by Stig
*              Halfdan Juhl Turner. This also holds for the functions that are
*              called from get_drone_position.
******************************************************************************/
{
  // Program control
  if(wait_enable)
    waitKey(wait_time);                         // Wait so visual debugging is possible

  bool debug = false;
  bool show_result = false;

  // Prepare the frame for tracking
  Mat src_frame_color, src_frame_gray, shape_masked;  // A frame for color and gray

  src_frame_in.copyTo(src_frame_color); // Make sure not to alter original frame
  cvtColor(src_frame_color,src_frame_gray,COLOR_BGR2GRAY);  // Convert to gray

  // Variables
  xy_position position;                   // Return struct
  vector<double> match_results;           // Values from match
  double lowest_match_result = INT_MAX;   // No match at the beginning
  int best_match_index;                   // Best match (if any)
  bool match_found = false;

  // Zero initialize position
  position_out->x = 0;
  position_out->y = 0;

  // Load shape
  if(!shape_loaded)   // Shape must be loaded first
    load_shape_im();

  //Find contours in frame
  frame_contours = get_contours(src_frame_gray);  // Find all contours in frame

  /*************** DEBUG ******************************************************/
  if(debug)
  {
    // Print number of contours
    cout << "Frame contours: " << frame_contours.size() << endl;
    // Show all contours
    drawContours(src_frame_color,shape_contours,-1,color_green,1,8,shape_hierarchy,
      0,Point(0,0));    // 1st aug: mat to be drawed upon, 2nd aug: contours to be drawed, 3rd aug: index for contour (-1 is all), 4th aug: color as a Scalar, 5th aug: thickness, 6th aug: line type (8 standard), 7th aug: hierarchy, 8th aug: maxlevel of hierarchy (0 is only the specified one), 9th aug: offset to shift contours (standard: don't shift Point(0,0))
    //namedWindow("Frame contours", WINDOW_FREERATIO);
    show_frame(window_names[5], window_show[5], src_frame_color);
  }
  /*********** END DEBUG ******************************************************/

  // Match shape contour with found contours
  for(int i = 0; i < frame_contours.size(); i++)  // For all found contours
  {
    match_results.push_back(matchShapes(shape_contours[SHAPE_CONTOUR_INDEX],
       frame_contours[i], CV_CONTOURS_MATCH_I1, 0));  // Match them             // Arguments in matchShapes. 1st aug: first contour, 2nd aug: second contour, 3rd aug: match type (I1 selected because it worked fine and was used various examples online), 4th aug: parameter that is not supported yet.
  }

  // Find match with lovest value (0 is two identical contours)
  for(int i = 0; i < static_cast<int>(match_results.size()); i++)
  {
    if(match_results[i] < lowest_match_result  // Current value less than lowest
      && match_results[i] < (shape_found_thresh*0.01) // & it is a match
      && contourArea(frame_contours[i])> minimum_drone_size)  // Make sure the contour is large enough to be a drone
    {
      lowest_match_result = match_results[i];   // Update lowest
      best_match_index = i;                     // Save the index
    }
  }

  // Process the match
  if(lowest_match_result != INT_MAX)            // If a match is found
  {
    match_found = true;
    get_position(frame_contours[best_match_index], &position); //Get its position
    if(show_result)
    {
      drawContours(src_frame_color,frame_contours,best_match_index,
        color_green,4,8,shape_hierarchy,0,Point(0,0));  // Draw the shape (drone) // 1st aug: mat to be drawed upon, 2nd aug: contours to be drawed, 3rd aug: index for contour (-1 is all), 4th aug: color as a Scalar, 5th aug: thickness, 6th aug: line type (8 standard), 7th aug: hierarchy, 8th aug: maxlevel of hierarchy (0 is only the specified one), 9th aug: offset to shift contours (standard: don't shift Point(0,0))
      circle(src_frame_color, Point2f(position.x,position.y), 4,
        color_red, -1, 8, 0); // Draw center of shape (mass_center). Arguments - 1st aug: mat to draw in, 2nd aug: center point, 3rd aug: radius, 4th aug: color as a scalar, 5th aug: thickness (negative is filled circle), 6th aug: linetype (8 standard), 7th aug: number of fractional bits in the center coordinates and the radius value
    }

    drawContours(*frame_out,frame_contours,best_match_index,
      color_white,-1,8,shape_hierarchy,0,Point(0,0));   // Make a mask with the drone // Draw the shape (drone) // 1st aug: mat to be drawed upon, 2nd aug: contours to be drawed, 3rd aug: index for contour (-1 is all), 4th aug: color as a Scalar, 5th aug: thickness (negative = filled out), 6th aug: line type (8 standard), 7th aug: hierarchy, 8th aug: maxlevel of hierarchy (0 is only the specified one), 9th aug: offset to shift contours (standard: don't shift Point(0,0))


    position_out->x=position.x;                     // Copy to return struct
    position_out->y=position.y;
    position_out->orientation=position.orientation;

    (src_frame_in).copyTo(shape_masked, *frame_out);  // Copy only the drone to shape_masked
    shape_masked.copyTo(*frame_out); // Copy only the drone back to frame_out


    if (show_result) {
      // Print information
      cout << "lowest_match_result = " << lowest_match_result << " x: "
        << position.x << " y: " << position.y  << " O: " << position.orientation
        << "\t frame: " << frame_number  << endl;
    }
  }
  else    // No match is found
    if (show_result) {
      cout << "lowest_match_result = INT_MAX -> no match \t frame: "
      << frame_number << endl;  // Print no match
    }

  if(show_result)
  {
    //namedWindow("Tracking", WINDOW_FREERATIO);
    show_frame(window_names[6], window_show[6], src_frame_color); // Show the result
    //namedWindow("Drone masked out, inside", WINDOW_FREERATIO);
    show_frame(window_names[7], window_show[7], *frame_out);
    if(frame_number==SAVE_FRAME_NUM)
      frame_save(src_frame_color, "frame_"+to_string(SAVE_FRAME_NUM));

  }
  match_results.clear();      // Delete results
  frame_contours.clear();     // Delete contours
  frame_number++;             // Increment framenumber (MAY FAIL!)
  return match_found;
}

void drone_tracking::load_shape_im()
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Loads the template image frame and finds contours in it. Contours
*              are stored to shape_contours. Sets shape_loaded = true.
******************************************************************************/
{
  bool debug = false;
  cout << "Loading shape image." << endl;
  // Test if shape image exists on path
  fstream shape_file(SHAPE_IM_PATH);
  if(shape_file.good())
  {
    // Load shape image in gray scale from SHAPE_IM_PATH
    Mat shape_im = imread(SHAPE_IM_PATH,CV_LOAD_IMAGE_GRAYSCALE);

    shape_contours = get_contours(shape_im);      // Get contours in shape_im

    shape_loaded = true;      // Mark shape as loaded
    cout << "Shape loaded." << endl;
    /*************** DEBUG ******************************************************/
    if(debug)
    {
      Mat shape_im_color = imread(SHAPE_IM_PATH); // Read frame in color
      Mat shape_contour0, shape_contour1, shape_contourx;         // Frame for contour[0] and [1]
      shape_im_color.copyTo(shape_contour0);      // Copy color frame to the new
      shape_im_color.copyTo(shape_contour1);      // frames
      shape_im_color.copyTo(shape_contourx);      // frames

      show_frame(window_names[8], window_show[8], shape_im);        // Show gray scale shape frame

      for(int i=0;i<shape_contours.size();i++)    // For all contours in frame
      {
        drawContours(shape_im_color,shape_contours,i,color_green,4,8,
          shape_hierarchy,0,Point(0,0));  // Draw it on color frame. Arguments - 1st aug: mat to be drawed upon, 2nd aug: contours to be drawed, 3rd aug: index for contour (-1 is all), 4th aug: color as a Scalar, 5th aug: thickness, 6th aug: line type (8 standard), 7th aug: hierarchy, 8th aug: maxlevel of hierarchy (0 is only the specified one), 9th aug: offset to shift contours (standard: don't shift Point(0,0))
      }

      cout << shape_contours.size() << endl;      // Print number of contours
      //namedWindow("Contours on shape frame", WINDOW_FREERATIO);
      show_frame(window_names[9], window_show[9], shape_im_color);  // Show color frame

      drawContours(shape_contour0,shape_contours,0,color_green,4,8,
        shape_hierarchy,0,Point(0,0));  // Draw contour0 on contour[0]. Arguments - 1st aug: mat to be drawed upon, 2nd aug: contours to be drawed, 3rd aug: index for contour (-1 is all), 4th aug: color as a Scalar, 5th aug: thickness, 6th aug: line type (8 standard), 7th aug: hierarchy, 8th aug: maxlevel of hierarchy (0 is only the specified one), 9th aug: offset to shift contours (standard: don't shift Point(0,0))
      drawContours(shape_contour1,shape_contours,1,color_green,4,8,
        shape_hierarchy,0,Point(0,0));  // Draw contour1 on contour[1]. Arguments - 1st aug: mat to be drawed upon, 2nd aug: contours to be drawed, 3rd aug: index for contour (-1 is all), 4th aug: color as a Scalar, 5th aug: thickness, 6th aug: line type (8 standard), 7th aug: hierarchy, 8th aug: maxlevel of hierarchy (0 is only the specified one), 9th aug: offset to shift contours (standard: don't shift Point(0,0))
      drawContours(shape_contourx,shape_contours,-1,color_green,4,8,
        shape_hierarchy,0,Point(0,0));  // Draw contour1 on contour[1]. Arguments - 1st aug: mat to be drawed upon, 2nd aug: contours to be drawed, 3rd aug: index for contour (-1 is all), 4th aug: color as a Scalar, 5th aug: thickness, 6th aug: line type (8 standard), 7th aug: hierarchy, 8th aug: maxlevel of hierarchy (0 is only the specified one), 9th aug: offset to shift contours (standard: don't shift Point(0,0))

      //namedWindow("Contour0", WINDOW_FREERATIO);
      //namedWindow("Contour1", WINDOW_FREERATIO);
      show_frame(window_names[10], window_show[10], shape_contour0); // Show frames with contour[0] and 1
      show_frame(window_names[11], window_show[11], shape_contour1);
      show_frame(window_names[16],window_show[16],shape_contourx);
    }
    /*********** END DEBUG ******************************************************/
  }
  else
  {
    cout << "ERROR: Shape image does not exist. Check file path." << endl;
    exit(1);
  }
  cout << "Returning from load_shape_im" << endl;
}

vector<vector<Point>> drone_tracking::get_contours(Mat src_in)
/*****************************************************************************
*   Input    : Mat with frame to find contours in
*   Output   : Vector with contours (each index is a vector with points)
*   Function : Finds contours in src_in. Erodes and dilates frame before
*              contours are found.
******************************************************************************/
{
  bool debug = true;
  Mat src;                              // Don't alter src_in -> make a new mat
  src_in.copyTo(src);                   // Copy src_in to src
  vector<vector<Point>> local_contours; // Contours found
  vector<Vec4i> local_hierarchy;        // Hierarchy of contours

  // Threshold src and store result in src
  threshold(src, src, thresh_tresh, THRESH_MAXVAL, THRESH_TYPE); // 1st aug: mat in, 2nd aug: mat out, 3rd aug: threshold, 4th aug: maximum value to assign when src(x,y)<= thresh, 5th aug: thresholding type (most are defined in top of file)

  if(debug)
  {
    //namedWindow("Thresholded frame",WINDOW_FREERATIO);
    show_frame(window_names[12], window_show[12],src);  // Show the thresholded frame
  }
  local_erode(src);                     // Erode
  local_dilate(src);                    // Dilate

  // Find contours in src, store in local_contours and local_hieararchy
  // Organize contours in tree structure and make lines where possible (only two
  // points for a line)
  findContours(src, local_contours, local_hierarchy, CV_RETR_TREE,
    CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); // 1st aug: mat to draw on, 2nd aug: where to store the contours, 3rd aug: where to store the hierarchy, 4th aug: mode (CV_RETR_TREE retrieves all of the contours and reconstructs a full hierarchy of nested contours), 5th aug: method (how many points to store: CV_CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments and leaves only their end points), 6th aug: offset by which each contour point is shifted (default: Point(0, 0) = no shift)
  shape_hierarchy = local_hierarchy;      //Necessary to draw contours elsewhere

  return local_contours;
}

Mat drone_tracking::local_erode(Mat src)
/*****************************************************************************
*   Input    : Mat to be eroded
*   Output   : Mat
*   Function : Erodes mat based on definitions in start of module. Based on
*              http://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html
******************************************************************************/

{
  bool debug = false;
  // Make an element for eroding (its like the shape and size)
  Mat element = getStructuringElement(erosion_type,
    Size(2*erosion_size + 1, 2*erosion_size+1),
    Point(erosion_size, erosion_size)); // 1st aug: type (of kernel), 2nd aug: size, 3rd aug: anchor (default: Point(-1,-1) = center of element)

  erode(src, src, element, Point(-1,-1), erode_iterations);  // Erode from center. Arguments - 1st aug: mat in, 2nd aug: mat out, 3rd aug: kernel, 4th aug: anchor (default: Point(-1,-1) = center of element), 4th aug: iterations (number of times erosion is applied)

  /*************** DEBUG ******************************************************/
  if(debug)
  {
    //namedWindow("Erode", CV_WINDOW_FREERATIO);
    show_frame(window_names[13], window_show[13],src);                    // Show the result
  }
  /*********** END DEBUG ******************************************************/

  return src;
}

Mat drone_tracking::local_dilate(Mat src)
/*****************************************************************************
*   Input    : Mat to be dilated
*   Output   : Mat
*   Function : Dilates mat based on definitions in start of module. Based on
*              http://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html
******************************************************************************/
{
  bool debug = false;
  // Make an element for dilating (its like the shape and size)
  Mat element = getStructuringElement(dilation_type,
    Size(2*dilation_size+1, 2*dilation_size+1),
    Point(dilation_size, dilation_size)); // 1st aug: type (of kernel), 2nd aug: size, 3rd aug: anchor (default: Point(-1,-1) = center of element)

  dilate(src, src, element, Point(-1,-1), dilate_iterations); //Dilate from center. Arguments - 1st aug: type (of kernel), 2nd aug: size, 3rd aug: anchor (default: Point(-1,-1) = center of element)

  /*************** DEBUG ******************************************************/
  if(debug)
  {
    //namedWindow("Dilate", CV_WINDOW_FREERATIO);
    show_frame(window_names[14], window_show[14], src);                // Show result
  }
  /*********** END DEBUG ******************************************************/

  return src;
}

void drone_tracking::get_position(vector<Point> contour, xy_position *pos)
/*****************************************************************************
*   Input    : Contour in vector (vector of points)
             : Struct for x-y position
*   Output   : None
*   Function : Calculates center of drone from center of contour. Based on
*              http://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html
               http://stackoverflow.com/questions/14720722/binary-image-orientation
******************************************************************************/
{
  Moments moment = moments(contour,false);  // Calculate moments. Arguments - 1st: a contour, 2nd: binary image, if true all nonzero pixels are treated as 1's
  pos->x = moment.m10/moment.m00;            // Calculate x position
  pos->y = moment.m01/moment.m00;            // Calculate y position
  // Calculate orientation (may not be correct). Based on link above
  double orientation = 0.5 * atan(2 * moment.m11 / (moment.m20 - moment.m02));
  pos->orientation = (orientation / M_PI) * 180;
}

/*
void drone_tracking::handle_trackbars()
{

  createTrackbar(const cv::String &trackbarname, const cv::String &winname, int *value, int count)
  createTrackbar("erosion_size", "Settings", &erosion_size, 10);
  //createTrackbar("erosion_type","Settings",&erosion_type,3)
  create("erode_iterations","Settings",)

  int erosion_type = EROSION_TYPE;
  int erosion_size = EROSION_SIZE;
  int erode_iterations = ERODE_ITERATIONS;
  int dilation_type = DILATION_TYPE;
  int dilation_size = DILATION_SIZE;
  int dilate_iterations = DILATE_ITERATIONS;
  int thresh_thresh = THRESH_THRESH;
}
*/
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

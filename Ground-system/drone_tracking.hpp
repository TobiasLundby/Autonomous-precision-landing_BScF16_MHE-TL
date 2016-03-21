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
#include "opencv2/photo.hpp"
#include <opencv2/video/background_segm.hpp>
#include <vector>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

//#include "uav_locator_inc.cpp"



using namespace cv;
using namespace std;
/*****************************    Defines    *******************************/
# define M_PI           3.14159265358979323846  /* pi */

// Drone shape tracking
  // Load shape
#define SHAPE_IM_PATH       "src/shape.jpg" // Path to template image
#define SHAPE_CONTOUR_INDEX 0

  // Erosion and dilation
#define EROSION_TYPE        MORPH_ELLIPSE // MORPH_RECT MORPH_CROSS MORPH_ELLIPSE
#define EROSION_SIZE        1
#define ERODE_ITERATIONS    2
#define DILATION_TYPE       MORPH_ELLIPSE // MORPH_RECT MORPH_CROSS MORPH_ELLIPSE
#define DILATION_SIZE       EROSION_SIZE
#define DILATE_ITERATIONS   2

  // Thresholding image
#define THRESH_THRESH       60
#define THRESH_MAXVAL       255
#define THRESH_TYPE         THRESH_BINARY_INV // If src(x,y)>TRESH_TRESH 0
                                              // else THRESH_MAXVAL

#define SHAPE_FOUND_THRESH  1             // Value below is a match

typedef struct xy_position{   // Struct for xy-position of drone
   double x;
   double y;
   double orientation;
 } xy_position;



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
  void simple_shape_tracking();

  // Match shape methods
  xy_position get_drone_position(Mat);
  void load_shape_im();
  vector<vector<Point>> get_contours(Mat);
  Mat local_erode(Mat);
  Mat local_dilate(Mat);
  void get_position(vector<Point> contour, xy_position&);

private: // Variables
  string filename;
  Mat frame_bgr;
  VideoCapture capture;
  string video_window_text = "Drone tracking";

  // MatchShape variables
  bool shape_loaded = false;        // True if shape is loaded
  vector<vector<Point>> shape_contours, frame_contours; // Contours
  vector<Vec4i> shape_hierarchy;    // Hierarchy
  int frame_number = 0;             // Frame number for debug

  bool wait_enable = true;
  int wait_time = 100;

  // Match shape constants
  Scalar color_green = Scalar(0,255,0), color_red = Scalar(0,0,255);

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

  //locate_uav(frame_bgr);
  //simple_shape_tracking();
  get_drone_position(frame_bgr);


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
  show_frame("Contours",frame_local);

  namedWindow("Biggest contour",CV_WINDOW_FREERATIO);
  show_frame("Biggest contour",frame_local2);

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
  show_frame("Center",frame_local2);

}


xy_position drone_tracking::get_drone_position(Mat src_frame_in)
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
  if(wait_enable)
    waitKey(wait_time);                         // Wait so visual debugging is possible
  // Prepare the frame for tracking
  bool debug = false;
  Mat src_frame_color, src_frame_gray;  // A frame for color and gray
  src_frame_in.copyTo(src_frame_color); // Make sure not to alter original frame
  cvtColor(src_frame_color,src_frame_gray,COLOR_BGR2GRAY);  // Convert to gray

  // Variables
  xy_position position;                   // Return struct
  vector<double> match_results;           // Values from match
  double lowest_match_result = INT_MAX;   // No match at the beginning
  int best_match_index;                   // Best match (if any)

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
      0,Point(0,0));
    //namedWindow("Frame contours", WINDOW_FREERATIO);
    show_frame("Frame contours", src_frame_color);
  }
  /*********** END DEBUG ******************************************************/

  // Match shape contour with found contours
  for(int i = 0; i < frame_contours.size(); i++)  // For all found contours
  {
    match_results.push_back(matchShapes(shape_contours[SHAPE_CONTOUR_INDEX],
       frame_contours[i], CV_CONTOURS_MATCH_I1, 0));  // Match them
  }

  // Find match with lovest value (0 is two identical contours)
  for(int i = 0; i < static_cast<int>(match_results.size()); i++)
  {
    if(match_results[i] < lowest_match_result  // Current value less than lowest
      && match_results[i] < SHAPE_FOUND_THRESH) // & it is a match
    {
      lowest_match_result = match_results[i];   // Update lowest
      best_match_index = i;                     // Save the index
    }
  }

  // Process the match
  if(lowest_match_result != INT_MAX)            // If a match is found
  {
    get_position(frame_contours[best_match_index], position); //Get its position
    drawContours(src_frame_color,frame_contours,best_match_index,
      color_green,4,8,shape_hierarchy,0,Point(0,0));  // Draw the shape (drone)
    circle(src_frame_color, Point2f(position.x,position.y), 4,
      color_red, -1, 8, 0); // Draw center of shape (mass_center)
    imshow("Tracking",src_frame_color); // Show the result
    // Print information
    cout << "lowest_match_result = " << lowest_match_result << " x: "
      << position.x << " y: " << position.y  << " O: " << position.orientation
      << "\t frame: " << frame_number  << endl;
  }
  else    // No match is found
    cout << "lowest_match_result = INT_MAX -> no match \t frame: "
    << frame_number << endl;  // Print no match

  match_results.clear();      // Delete results
  frame_contours.clear();     // Delete contours
  frame_number++;             // Increment framenumber (MAY FAIL!)

  return position;
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

  // Load shape image in gray scale
  Mat shape_im = imread(SHAPE_IM_PATH,CV_LOAD_IMAGE_GRAYSCALE);

  shape_contours = get_contours(shape_im);      // Get contours in shape_im

  shape_loaded = true;      // Mark shape as loaded

  /*************** DEBUG ******************************************************/
  if(debug)
  {
    Mat shape_im_color = imread(SHAPE_IM_PATH); // Read frame in color
    Mat shape_contour0, shape_contour1;         // Frame for contour[0] and [1]
    shape_im_color.copyTo(shape_contour0);      // Copy color frame to the new
    shape_im_color.copyTo(shape_contour1);      // frames

    show_frame("Shape frame", shape_im);        // Show gray scale shape frame

    for(int i=0;i<shape_contours.size();i++)    // For all contours in frame
    {
      drawContours(shape_im_color,shape_contours,i,color_green,1,8,
        shape_hierarchy,0,Point(0,0));  // Draw it on color frame
    }

    cout << shape_contours.size() << endl;      // Print number of contours
    namedWindow("Contours on shape frame", WINDOW_FREERATIO);
    show_frame("Contours on shape frame", shape_im_color);  // Show color frame

    drawContours(shape_contour0,shape_contours,0,color_green,4,8,
      shape_hierarchy,0,Point(0,0));  // Draw contour0 on contour[0]
    drawContours(shape_contour1,shape_contours,1,color_green,4,8,
      shape_hierarchy,0,Point(0,0));  // Draw contour1 on contour[1]
    namedWindow("Contour0", WINDOW_FREERATIO);
    namedWindow("Contour1", WINDOW_FREERATIO);
    show_frame("Contour0", shape_contour0); // Show frames with contour[0] and 1
    show_frame("Contour1", shape_contour1);

  }
  /*********** END DEBUG ******************************************************/
}

vector<vector<Point>> drone_tracking::get_contours(Mat src_in)
/*****************************************************************************
*   Input    : Mat with frame to find contours in
*   Output   : Vector with contours (each index is a vector with points)
*   Function : Finds contours in src_in. Erodes and dilates frame before
*              contours are found.
******************************************************************************/
{
  bool debug = false;
  Mat src;                              // Don't alter src_in -> make a new mat
  src_in.copyTo(src);                   // Copy src_in to src
  vector<vector<Point>> local_contours; // Contours found
  vector<Vec4i> local_hierarchy;        // Hierarchy of contours

  // Threshold src and store result in src
  threshold(src, src, THRESH_THRESH, THRESH_MAXVAL, THRESH_TYPE);

  if(debug)
  {
    namedWindow("Thresholded frame",WINDOW_FREERATIO);
    show_frame("Thresholded frame",src);  // Show the thresholded frame
  }
  local_erode(src);                     // Erode
  local_dilate(src);                    // Dilate

  // Find contours in src, store in local_contours and local_hieararchy
  // Organize contours in tree structure and make lines where possible (only two
  // points for a line)
  findContours(src, local_contours, local_hierarchy, CV_RETR_TREE,
    CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
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
  Mat element = getStructuringElement(EROSION_TYPE,
    Size(2*EROSION_SIZE + 1, 2*EROSION_SIZE+1),
    Point(EROSION_SIZE, EROSION_SIZE));

  erode(src, src, element, Point(-1,-1), ERODE_ITERATIONS);  // Erode from center

  /*************** DEBUG ******************************************************/
  if(debug)
  {
    namedWindow("Erode", CV_WINDOW_FREERATIO);
    show_frame("Erode",src);                    // Show the result
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
  Mat element = getStructuringElement(DILATION_TYPE,
    Size(2*DILATION_SIZE+1, 2*DILATION_SIZE+1),
    Point(DILATION_SIZE, DILATION_SIZE));

  dilate(src, src, element, Point(-1,-1), DILATE_ITERATIONS); //Dilate from center

  /*************** DEBUG ******************************************************/
  if(debug)
  {
    namedWindow("Dilate", CV_WINDOW_FREERATIO);
    show_frame("Dilate", src);                // Show result
  }
  /*********** END DEBUG ******************************************************/

  return src;
}

void drone_tracking::get_position(vector<Point> contour, xy_position &pos)
/*****************************************************************************
*   Input    : Contour in vector (vector of points)
             : Struct for x-y position
*   Output   : None
*   Function : Calculates center of drone from center of contour. Based on
*              http://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html
               http://stackoverflow.com/questions/14720722/binary-image-orientation
******************************************************************************/
{
  Moments moment = moments(contour,false);  // Calculate moments
  pos.x = moment.m10/moment.m00;            // Calculate x position
  pos.y = moment.m01/moment.m00;            // Calculate y position
  // Calculate orientation (may not be correct)
  double orientation = 0.5 * atan(2 * moment.m11 / (moment.m20 - moment.m02));
  pos.orientation = (orientation / M_PI) * 180;
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

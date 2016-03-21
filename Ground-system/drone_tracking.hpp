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

#include "uav_locator_inc.cpp"



using namespace cv;
using namespace std;
/*****************************    Defines    *******************************/
# define M_PI           3.14159265358979323846  /* pi */

// Drone shape tracking
#define SHAPE_IM_PATH       "src/shape.jpg"
#define SHAPE_CONTOUR_INDEX 0

#define EROSION_TYPE        MORPH_ELLIPSE // MORPH_RECT MORPH_CROSS MORPH_ELLIPSE
#define EROSION_SIZE        1             //1
#define ERODE_ITERATIONS    2             // 2 finds shape_contour and removes stick
#define DILATION_TYPE       MORPH_ELLIPSE // MORPH_RECT MORPH_CROSS MORPH_ELLIPSE
#define DILATION_SIZE       EROSION_SIZE             //1
#define DILATE_ITERATIONS   2             // 4 finds shape_contour and removes stick

#define THRESH_THRESH       60              //40
#define THRESH_MAXVAL       255             //255
#define THRESH_TYPE         THRESH_BINARY_INV   // THRESH_BINARY

#define SHAPE_FOUND_THRESH  1

typedef struct xy_position{
   int x;
   int y;
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
  void find_black_mask();
  Mat subtract_background(Mat);
  Mat canny_edge_detect(Mat);
  void find_contours(Mat);
  Mat remove_noise(Mat);
  void subtract_background_manual(Mat);
  Mat complete_drone_shape(Mat);
  void load_shape();
  void compare_shapes(Mat src);
  void simple_shape_tracking();

  // Match shape
   xy_position get_drone_position(Mat);
   void load_shape_im();
   vector<vector<Point>> get_contours(Mat);
   Mat local_erode(Mat);
   Mat local_dilate(Mat);

private: // Variables
  string filename;
  Mat frame_bgr;
  VideoCapture capture;
  string video_window_text = "Drone tracking";

//******************* DRONE SHAPE DETECTION ********************

  // MatchShape detection
  bool shape_loaded = false;
  vector<vector<Point>> shape_contours, frame_contours;
  vector<Vec4i> shape_hierarchy;          // NOTE: Temp, to be removed
  int frame_number = 0;



  //http://docs.opencv.org/3.1.0/d1/dc5/tutorial_background_subtraction.html#gsc.tab=0
  // Test change - not relevant
  // Test 2


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
  //******* Init *******
//  MOG2 = createBackgroundSubtractorMOG2();  // Create background subtractor
//  KNN = createBackgroundSubtractorKNN();
  //********************


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
    //load_shape();     // MUST BE PLACED SOMEWHERE ELSE
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
  //find_black_mask();
  //subtract_background_manual(frame_bgr);
  //subtract_background(frame_bgr);
  //remove_noise(frame_foreground);
  //Mat canny_result = canny_edge_detect(frame_foreground);
  //find_contours(canny_output);
  //show_frame(video_window_text, frame_bgr);
  //load_shape();
  //compare_shapes(frame_bgr);
  //simple_shape_tracking();
  waitKey(200);
  get_drone_position(frame_bgr);
  //locate_uav(frame_bgr);

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
// With inspiration from code written by Stig Halfdan Juhl Turner
{
  // Prepare the frame for tracking
  bool debug = false;
  Mat src_frame_color, src_frame_gray;
  src_frame_in.copyTo(src_frame_color); // Make sure not to alter original frame
  cvtColor(src_frame_color,src_frame_gray,COLOR_BGR2GRAY);

  // Variables
  xy_position position;
  vector<double> match_results;
  double lowest_match_result = INT_MAX;
  int best_match_index;

  // Load shape
  if(!shape_loaded)   // Shape must be loaded first
    load_shape_im();

  //Find contours in frame
  frame_contours = get_contours(src_frame_gray);

  if(debug)
  {
    cout << "Frame contours: " << frame_contours.size() << endl;
    Scalar color = Scalar(0,255,0);
    drawContours(src_frame_color,shape_contours,-1,color,1,8,shape_hierarchy,
      0,Point(0,0));
    //namedWindow("Frame contours", WINDOW_FREERATIO);
    show_frame("Frame contours", src_frame_color);
  }

  for(int i = 0; i < frame_contours.size(); i++)
  {
    match_results.push_back(matchShapes(shape_contours[SHAPE_CONTOUR_INDEX],
       frame_contours[i], CV_CONTOURS_MATCH_I1, 0));
  }

  for(int i = 0; i < static_cast<int>(match_results.size()); i++)
  {
    if(match_results[i] < lowest_match_result
      && match_results[i] < SHAPE_FOUND_THRESH)
    {
      lowest_match_result = match_results[i];
      best_match_index = i;
    }
  }


  if(lowest_match_result != INT_MAX)
  {
    Scalar color = Scalar(0,255,0);
    drawContours(src_frame_color,frame_contours,best_match_index,color,4,8,shape_hierarchy,0,Point(0,0));
    imshow("Tracking",src_frame_color);
    cout << "lowest_match_result = " << lowest_match_result << "\t frame: " << frame_number << endl;
  }
  else
    cout << "lowest_match_result = INT_MAX \t frame: " << frame_number << endl;

  match_results.clear();
  frame_contours.clear();
  frame_number++;

  return position;
}

void drone_tracking::load_shape_im()
{
  bool debug = false;
  Mat shape_im = imread(SHAPE_IM_PATH,CV_LOAD_IMAGE_GRAYSCALE);   // Load shape image in gray scale

  // for debug:

    Mat shape_im_color = imread(SHAPE_IM_PATH);
    Mat shape_contour0, shape_contour1;
    shape_im_color.copyTo(shape_contour0);
    shape_im_color.copyTo(shape_contour1);
  if(debug)
  {
    show_frame("Shape frame", shape_im);
  }
  // end debug


  shape_contours = get_contours(shape_im);      // Get contours in shape_im

  Scalar color = Scalar(0,255,0);
  for(int i=0;i<shape_contours.size();i++)
  {
    drawContours(shape_im_color,shape_contours,i,color,1,8,shape_hierarchy,0,Point(0,0));
  }


  // debug
  if(debug)
  {
    cout << shape_contours.size() << endl;
    namedWindow("Contours on shape frame", WINDOW_FREERATIO);
    show_frame("Contours on shape frame", shape_im_color);

    drawContours(shape_contour0,shape_contours,0,color,1,8,shape_hierarchy,0,Point(0,0));
    drawContours(shape_contour1,shape_contours,1,color,1,8,shape_hierarchy,0,Point(0,0));
    namedWindow("Contour0", WINDOW_FREERATIO);
    namedWindow("Contour1", WINDOW_FREERATIO);
    show_frame("Contour0", shape_contour0);
    show_frame("Contour1", shape_contour1);
  }
  // end debug

  shape_loaded = true;

}

vector<vector<Point>> drone_tracking::get_contours(Mat src_in)
{
  Mat src;
  src_in.copyTo(src);
  vector<vector<Point>> local_contours;
  vector<Vec4i> local_hierarchy;
  threshold(src, src, THRESH_THRESH, THRESH_MAXVAL, THRESH_TYPE);
  namedWindow("Thresholded frame",WINDOW_FREERATIO);
  show_frame("Thresholded frame",src);
  local_erode(src);
  local_dilate(src);
  findContours(src, local_contours, local_hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
  shape_hierarchy = local_hierarchy;      // Temporary test. Should be removed.
  return local_contours;

}

Mat drone_tracking::local_erode(Mat src)
// Based on http://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html
{
  Mat element = getStructuringElement(EROSION_TYPE,
                                     Size(2*EROSION_SIZE + 1, 2*EROSION_SIZE+1),
                                     Point(EROSION_SIZE, EROSION_SIZE));

  erode(src, src, element, Point(-1,-1), ERODE_ITERATIONS);
  namedWindow("Erode", CV_WINDOW_FREERATIO);
  show_frame("Erode",src);
  return src;
}

Mat drone_tracking::local_dilate(Mat src)
// Based on http://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html
{
  Mat element = getStructuringElement( DILATION_TYPE,
                                       Size(2*DILATION_SIZE+1, 2*DILATION_SIZE+1),
                                       Point(DILATION_SIZE, DILATION_SIZE));

  dilate(src, src, element, Point(-1,-1), DILATE_ITERATIONS);
  namedWindow("Dilate", CV_WINDOW_FREERATIO);
  show_frame("Dilate", src);
  return src;
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

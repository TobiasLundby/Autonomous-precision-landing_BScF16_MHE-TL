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
  Mat subtract_background(Mat);
  Mat canny_edge_detect(Mat);
  void find_contours(Mat);
  Mat remove_noise(Mat);
  void subtract_background_manual(Mat);
  Mat complete_drone_shape(Mat);
  void load_shape();
  void compare_shapes(Mat src);
  void simple_shape_tracking();

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

  // Background subtraction
  Mat foreground_mask_KNN;
  Mat foreground_mask_MOG2;
  Ptr<BackgroundSubtractor> KNN;
  Ptr<BackgroundSubtractor> MOG2;
  Mat frame_foreground;

  // Canny edge
  int edgeThresh = 1;
  int lowThresh = 10;
  int const max_lowThreshold = 100;
  int ratio = 3;
  int kernel_size = 3;
  Mat canny_output;

  // Manual background subtraction
  int frame_number = 0;
  Mat frame1;
  Mat frame2;
  Mat frame3;
  Mat background_frame;

  // Load shape and Compare shapes
  Mat shape;
  vector<vector<Point>> shape_contours;
  vector<Vec4i> shape_hierarchy;
  int biggest_index = -1;

  //HSV values for black
  int bgr_b_black_low = 0;
  int bgr_g_black_low = 0;
  int bgr_r_black_low = 0;
  int bgr_b_black_upper = 5;
  int bgr_g_black_upper = 5;
  int bgr_r_black_upper = 5;

  int hsv_h_black_base        = 0; //60 is for green
  int hsv_h_black_sensitivity = 30;
  int hsv_h_black_low         = 0;//hsv_h_black_base //- hsv_h_black_sensitivity;
  int hsv_h_black_upper       = 0;//hsv_h_black_base + hsv_h_black_sensitivity;
  int hsv_s_black_low         = 0;
  int hsv_s_black_upper       = 0;
  int hsv_v_black_low         = 0;
  int hsv_v_black_upper       = 10;


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
  MOG2 = createBackgroundSubtractorMOG2();  // Create background subtractor
  KNN = createBackgroundSubtractorKNN();
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
  simple_shape_tracking();
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

Mat drone_tracking::subtract_background(Mat src)
{
  Mat dst,dst_inv,black;
  dst = Scalar::all(255);
  black = Scalar::all(255);
  KNN->apply(src,foreground_mask_KNN);
  MOG2->apply(src,foreground_mask_MOG2);

  //show_frame("Foreground KNN",foreground_mask_KNN);
  //show_frame("Foreground MOG2",foreground_mask_MOG2);

  frame_bgr.copyTo(dst, foreground_mask_MOG2);
  bitwise_not(dst, dst_inv,~foreground_mask_MOG2);
  frame_foreground = dst;
  show_frame("Foreground masked",dst);
  //show_frame("Foreground masked inverted",dst_inv);
  //show_frame("Foreground mask", foreground_mask_MOG2);

  Mat frame_foregrund_blackmask;
  inRange(dst, Scalar(bgr_b_black_low,bgr_g_black_low,bgr_r_black_low), Scalar(bgr_b_black_upper, bgr_g_black_upper, bgr_r_black_upper), frame_foregrund_blackmask);

  show_frame("Foreground blackmask",frame_foregrund_blackmask);




  return dst;
}

Mat drone_tracking::canny_edge_detect(Mat src)
// Inpsired from http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
{
  Mat frame_gray;
  //Mat canny_output;
  Mat dst;
  cvtColor(src, frame_gray, CV_BGR2GRAY );
  blur(frame_gray,canny_output,Size(3,3));         // Reduce noise with kernel of 3*3

  Canny(canny_output,canny_output,lowThresh,lowThresh*ratio,kernel_size);

  dst = Scalar::all(0);
  src.copyTo( dst, canny_output);
  imshow("Canny edges",dst);

  return dst;
}

void drone_tracking::find_contours(Mat src)
{
  //RNG rng(12345);
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;

  findContours(src, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,Point(0,0));

  int biggest_index;
  double area, biggest_area = 0;
  for(int i=0;i<contours.size();i++)
  {
    area = contourArea(contours[i]);
    if( area > biggest_area)
    {
      biggest_area = area;
      biggest_index = i;
    }
  }

  Mat contours_drawed = Mat::zeros( src.size(), CV_8UC3 ); // Make mat with size of src and 8-bit, unsigned, 3 channels
  Scalar color = Scalar(100,100,100);
/*  for(int i=0;i<contours.size();i++)
  {
    drawContours(contours_drawed, contours, i,color);
  } */

  drawContours(contours_drawed, contours, biggest_index,color);

  show_frame("Contours",contours_drawed);
}

Mat drone_tracking::remove_noise(Mat src)
{
  fastNlMeansDenoisingColored(src,src,3,7,21);
  show_frame("Noise removed",src);
  //fastNlMeansDenoising(InputArray src, OutputArray dst, float h=3, int templateWindowSize=7, int searchWindowSize=21 )
}

void drone_tracking::subtract_background_manual(Mat src)
{
  frame_number++;
  cout << frame_number << endl;
  if(frame_number < 0)
    ;
  else if(frame_number==1)
    //frame1 = src;
    src.copyTo(frame1);
  else if(frame_number==2)
    //frame2 = src;
    src.copyTo(frame2);
  else if(frame_number==3)
  {
    //frame3 = src;
    src.copyTo(frame3);
    Mat temp_frame, temp_frame2, temp_frame3;
    temp_frame = frame1 + frame2;
    temp_frame2 = temp_frame + frame3;
    // //addWeighted(frame1, 1.0, frame2, 1.0, 0.0, temp_frame)
    //addWeighted(frame3, 1.0, temp_frame, 1.0, 0.0,temp_frame2)
    //background_frame = temp_frame2 / 3.0;
    //show_frame("Background",temp_frame2);
    //show_frame("Frame3",frame3);
    //show_frame("Frame2",frame2);
    //show_frame("Frame1",frame1);

    //temp_frame3 = temp_frame2 / 3.0f;
    //show_frame("temp_frame2",temp_frame3);

  }
  else
    {
      frame_foreground = src - frame3;
      show_frame("Manual foreground",frame_foreground);



      // Frame number reset to avoid avoid overflow problems
      if(frame_number==65530)
      {
        frame_number = 4;     // Reset to first frame not used as background
        cout << "frame_number reset" << endl;
      }

    }
}

/*
Mat drone_tracking::complete_drone_shape(Mat src)
{
  for()
}
*/

void drone_tracking::compare_shapes(Mat src)
{
  Mat src_gray, src_result;
  int thresh=150;

  vector<vector<Point>> src_contours;
  vector<Vec4i> src_hierarchy;

  cvtColor(src, src_gray,CV_BGR2GRAY);
  Canny(src_gray,src_result, thresh,thresh*2);
  findContours(src_result,src_contours,src_hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

  vector<float> answers;
  RNG rng(12345);
  for(int i=0;i<src_contours.size();i++)
  {
      Scalar color=Scalar(0,255,0);
      //Scalar color=Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
      answers.push_back( matchShapes(src_contours[i], shape_contours[biggest_index],CV_CONTOURS_MATCH_I2,0) );
      drawContours(src,src_contours,i,color,5,8,shape_hierarchy,0,Point(0,0));
  }

  float lowest_value = 1000;
  int lowest_index;
  for(int i;i<answers.size();i++)
  {
    if(answers[i]<lowest_value)
    {
      lowest_value = answers[i];
      lowest_index = i;
    }
  }

  cout << "Lowest value: " << answers[lowest_index] << " at index: " << lowest_index << " of: " << src_contours.size() << " indexes" << endl;

  namedWindow("src_result",CV_WINDOW_FREERATIO);
  namedWindow("src_result_below",CV_WINDOW_FREERATIO);
  show_frame("src_result",src);

  if(answers[lowest_index]<0.001)
    show_frame("src_result_below",src);






}

void drone_tracking::load_shape()
{
  Mat shape_gray, shape_result;
  int thresh=150;

  shape = imread("src/shape.jpg");   // Load as color image
  //show_frame("Shape source",shape);
  cvtColor(shape, shape_gray,CV_BGR2GRAY);
  //show_frame("Shape_gray",shape_gray);
  Canny(shape_gray,shape_result,thresh,thresh*2);

  show_frame("Shape",shape_result);

  //vector<vector<Point>> shape_contours;
  //vector<Vec4i> shape_hierarchy;


  Mat shape_result_color;
  cvtColor(shape_result, shape_result_color,CV_GRAY2BGR);
  findContours(shape_result,shape_contours,shape_hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

  RNG rng(12345);
  // for(int i=0;i<shape_contours.size();i++)
  // {
  //     //Scalar color=Scalar(60,60,60);
  //     Scalar color=Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
  //     drawContours(shape_result_color,shape_contours,i,color,1,8,shape_hierarchy,0,Point(0,0));
  // }

  cout << "Shape contours.size()=" << shape_contours.size() << endl;

  biggest_index = -1;
  double area, biggest_area = 0;
  for(int i=0;i<shape_contours.size();i++)
  {
    area = contourArea(shape_contours[i]);
    if( area > biggest_area)
    {
      biggest_area = area;
      biggest_index = i;
    }
  }

  if(biggest_index != -1)
  {
    Scalar color=Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    drawContours(shape_result_color,shape_contours,biggest_index,color,1,8,shape_hierarchy,0,Point(0,0));
    cout << "Shape biggest_index: " << biggest_index << endl;
  }


  //show_frame("Shape contours",shape_result_color);

}

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

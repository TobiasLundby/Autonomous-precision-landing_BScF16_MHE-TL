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


using namespace cv;
using namespace std;
/*****************************    Defines    *******************************/


/*****************************   Class   *******************************/


class drone_tracking
{
public:
  drone_tracking();
  drone_tracking(string);
  void show_video();
private:
  string filename;
  Mat frame;
  VideoCapture capture;


};

drone_tracking::drone_tracking()
/*****************************************************************************
*   Input    :
*   Output   :
*   Function : Default constructor
******************************************************************************/
{
  int methaman = 0;
}

drone_tracking::drone_tracking(string filenameIn)
/*****************************************************************************
*   Input    : (int) number of commandline arguments
*            : (char**) commandline arguments
*   Output   : None
*   Function : Constructor
******************************************************************************/
{
  filename = filenameIn;

  // Open specified video file or webcam
  if(filename=="") // If filename is webcam
  {
    cout << "Opening external webcam"<< endl;
    capture.open(1);            // Open external webcam

  }
  else if(filename=="webcam")
  {
    cout << "Opening external webcam"<< endl;
    capture.open(0);            // Open internal webcam
  }
  else
  {
    cout << "Opening video" << endl;
    capture.open(filename);                  // Open file
  }

}

void drone_tracking::show_video()
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Displays video
******************************************************************************/
{
// Test if capture is opened
  if(capture.isOpened())
  {
      cout << "Capture is opened" << endl;
// Processing
      for(;;)
      {
          capture >> frame;
          if(frame.empty())
              break;
          imshow("Sample", frame);
          if(waitKey(10) >= 0)
              break;
      }
  }
  else    // Error capture is not opened
  {
      cout << "No capture" << endl;
      frame = Mat::zeros(480, 640, CV_8UC1);
      imshow("Sample", frame);
      waitKey(0);
  }

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

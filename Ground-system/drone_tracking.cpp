/*****************************************************************************
* University of Southern Denmark
* UAS Center
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: drone_tracking.cpp
* PROJECT....: Autonomous precision landing ground system
* DESCRIPTION: See module specification file (.h-file).
*
*****************************************************************************/
#include "drone_tracking.hpp"


using namespace cv;
using namespace std;

void drone_tracking::drone_tracking()
/*****************************************************************************
*   Input    :
*   Output   :
*   Function : Default constructor
******************************************************************************/
{
  int methaman = 0;
}

void drone_tracking::drone_tracking(int cmd_argc,char** cmd_argv)
/*****************************************************************************
*   Input    : (int) number of commandline arguments
*            : (char**) commandline arguments
*   Output   : None
*   Function : Default constructor
******************************************************************************/
{
  argc = cmd_argc;
  argv = cmd_argv;

  // Open specified video file or webcam
  if(argc==2) // If two arguments, open file
  {
    cout << "Opening video"<< endl;
    capture.open(argv[2]);            // Open video file

  }
  else
  {
    cout << "Opening webcam" << endl;
    capture.open(0);                  // Open webcam
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
      image = Mat::zeros(480, 640, CV_8UC1);
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

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

/*****************************    Defines    *******************************/


/*****************************   Class   *******************************/


class drone_tracking
{
public:
  drone_tracking();
private:
  int argc;
  char** argv;
  UMat frame;
  VideoCapture capture;


};
// Function dummy
/*****************************************************************************
*   Input    :
*   Output   :
*   Function :
******************************************************************************/







/****************************** End Of Module *******************************/

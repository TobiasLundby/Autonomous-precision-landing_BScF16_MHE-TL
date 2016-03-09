/*****************************************************************************
* University of Southern Denmark
* UAS Center
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: main.cpp
* PROJECT....: Autonomous precision landing ground system
* DESCRIPTION: ???
*
*****************************************************************************/

// #include "opencv2/core.hpp"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/videoio.hpp"
#include <iostream>

#include "drone_tracking.hpp"

using namespace cv;
using namespace std;

//void drawText(Mat & image);

int main(int argc,char* argv[])
{
  string filename;
  if(argc<2)
    filename = "";
  else
    filename = argv[1];

  drone_tracking video(filename);
  video.show_video();

  return 0;
}

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

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

#include "drone_tracking.hpp"

using namespace cv;
using namespace std;

void drawText(Mat & image);

int main(int argc,char** argv)
{
    cout << "Built with OpenCV " << CV_VERSION << endl;
    Mat image;
    VideoCapture capture;

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

// Test if capture is opened
    if(capture.isOpened())
    {
        cout << "Capture is opened" << endl;
// Processing
        for(;;)
        {
            capture >> image;
            if(image.empty())
                break;
            imshow("Sample", image);
            if(waitKey(10) >= 0)
                break;
        }
    }
    else    // Error capture is not opened
    {
        cout << "No capture" << endl;
        image = Mat::zeros(480, 640, CV_8UC1);
        imshow("Sample", image);
        waitKey(0);
    }
    return 0;
}

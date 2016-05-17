/*****************************************************************************
* University of Southern Denmark
* UAS Center
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: main.cpp
* PROJECT....: Autonomous precision landing ground system
* DESCRIPTION: Please write a description???
*
*****************************************************************************/

// #include "opencv2/core.hpp"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/videoio.hpp"
#include <iostream>
#include <cstdlib>
#include <pthread.h>

#include "drone_tracking.hpp"
#include "interthread_communication.hpp"
#include "server.hpp"

using namespace cv;
using namespace std;

//void drawText(Mat & image);

#define NUM_THREADS     1

void *Socket_communication(void *threadid)
{
  long tid;
  tid = (long)threadid;
  cout << "Socket_communication! Thread ID, " << tid << endl;

  socket_server socket;   // Initialize socket connection
  while(true)
  {
    // Do the communication
    pthread_mutex_lock(&mutex_sock_pack_in);
    socket.socket_get_frame(&sock_pack_in);
    pthread_mutex_unlock(&mutex_sock_pack_in);
    pthread_mutex_lock(&mutex_sock_pack_out);
    socket.socket_send_frame(sock_pack_out);
    pthread_mutex_unlock(&mutex_sock_pack_out);
  }


  pthread_exit(NULL);
}

int main(int argc,char* argv[])
{
  pthread_mutex_init(&mutex_sock_pack_in, NULL);
  pthread_mutex_init(&mutex_sock_pack_out, NULL);

  pthread_t threads[NUM_THREADS];
  int rc;
  int i;
  for( i=0; i < NUM_THREADS; i++ ){
      cout << "main() : creating thread, " << i << endl;
      rc = pthread_create(&threads[i], NULL,
                          Socket_communication, (void *)i);
      if (rc){
         cout << "Error:unable to create thread," << rc << endl;
         exit(-1);
      }
   }

  string filename;
  if(argc<2)
    filename = "";
  else
    filename = argv[1];

  drone_tracking video(filename);

  pthread_mutex_destroy(&mutex_sock_pack_in);
  pthread_mutex_destroy(&mutex_sock_pack_out);

  return 0;
}

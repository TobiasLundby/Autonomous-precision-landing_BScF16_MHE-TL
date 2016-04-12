#include "client.hpp"
#include <string.h>
#include <string>

#define SERVER_IP "192.168.43.193"
#define PORT "3400"


using namespace std;

int main(int argc,char* argv[])
{
  char portin[10];
  strcpy(portin,argv[1]);
  //socket_client socket(SERVER_IP,portin);
  socket_client socket;
  printf("Please enter the message: ");
  char buffer[256];
  bzero(buffer,256);
  fgets(buffer,255,stdin);
  socket.socket_send_msg(buffer);
  socket.socket_get_msg();
  bzero(buffer,256);
  socket.socket_close();
  return 0;
}

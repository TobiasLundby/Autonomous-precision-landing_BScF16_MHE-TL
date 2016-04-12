#include "server.hpp"


using namespace std;

int main(int argc,char* argv[])
{
  char portin[10];
  strcpy(portin,argv[1]);
  //socket_server socket(portin,portin);
  socket_server socket;
  socket.socket_get_msg();
  socket.socket_send_msg("I got your message");
  socket.socket_close();
  return 0;
}

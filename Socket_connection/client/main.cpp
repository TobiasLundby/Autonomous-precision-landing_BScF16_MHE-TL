#include "client.hpp"
#include <string.h>
#include <string>
#include <iostream>

#define SERVER_IP "192.168.43.193"
#define PORT "3400"


using namespace std;

int main(int argc,char* argv[])
{
  socket_package package_out, package_in;
  package_out.field0=0000;
  package_out.field1=1111;
  package_out.field2=2222;
  package_out.field3=3333;
  package_out.field4=4444;
  package_out.field5=5555;
  package_out.field6=6666;
  package_out.field7=7777;
  socket_client socket;
  string stringout;


  socket.socket_send_frame(package_out);
  socket.socket_get_frame(&package_in);

  std::cout << package_in.field0 << std::endl;
  std::cout << package_in.field1 << std::endl;
  std::cout << package_in.field2 << std::endl;
  std::cout << package_in.field3 << std::endl;
  std::cout << package_in.field4 << std::endl;
  std::cout << package_in.field5 << std::endl;
  std::cout << package_in.field6 << std::endl;
  std::cout << package_in.field7 << std::endl;






/*  char portin[10];
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
*/  socket.socket_close();

  return 0;
}

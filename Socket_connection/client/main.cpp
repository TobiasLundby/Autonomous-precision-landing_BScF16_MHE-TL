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
  package_out.field0=0;
  package_out.field1=1;
  package_out.field2=2;
  package_out.field3=3;
  package_out.field4=4;
  package_out.field5=5;
  package_out.field6=6;
  package_out.field7=7;
  package_out.field8=8;
  package_out.field9=9;
  package_out.field10=10;
  package_out.field11=11;
  package_out.field12=12;
  package_out.field13=13;
  package_out.field14=14;
  package_out.field15=15;
  package_out.field16=16;
  package_out.field17=17;
  package_out.field18=18;
  package_out.field19=19;
  socket_client socket;
  string stringout;

  while(true)
  {
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
    std::cout << package_in.field8 << std::endl;
    std::cout << package_in.field9 << std::endl;
    std::cout << package_in.field10 << std::endl;
    std::cout << package_in.field11 << std::endl;
    std::cout << package_in.field12 << std::endl;
    std::cout << package_in.field13 << std::endl;
    std::cout << package_in.field14 << std::endl;
    std::cout << package_in.field15 << std::endl;
    std::cout << package_in.field16 << std::endl;
    std::cout << package_in.field17 << std::endl;
    std::cout << package_in.field18 << std::endl;
    std::cout << package_in.field19 << std::endl;
  }





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

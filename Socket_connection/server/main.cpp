#include "server.hpp"
#include <iostream>

using namespace std;

int main(int argc,char* argv[])
{

  socket_package package_in;
  //char portin[10];
  //strcpy(portin,argv[1]);
  //socket_server socket(portin,portin);
  socket_server socket;
  //socket.socket_get_msg();
  //socket.socket_send_msg("I got your message");

  std::cout << "From main:" << std::endl;
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
  socket.socket_send_frame(package_in);


  socket.socket_close();
  return 0;
}

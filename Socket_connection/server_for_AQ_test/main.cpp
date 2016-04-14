#include "server.hpp"
#include <iostream>

using namespace std;

int add10(int val)
{
  if(val<500)
    val += 1;
  else
    val = 0;
  return val;
}


void add10_pack(socket_package* pack)
{

  pack->field0 = add10(pack->field0);
  pack->field1 = add10(pack->field1);
  pack->field2 = add10(pack->field2);
  pack->field3 = add10(pack->field3);
  pack->field4 = add10(pack->field4);
  pack->field5 = add10(pack->field5);
  pack->field6 = add10(pack->field6);
  pack->field7 = add10(pack->field7);
}

int main(int argc,char* argv[])
{
  socket_package package_in, package_out;
  socket_server socket;

  package_out.field0 = 0;
  package_out.field1 = 50;
  package_out.field2 = 100;
  package_out.field3 = 150;
  package_out.field4 = 200;
  package_out.field5 = 250;
  package_out.field6 = 300;
  package_out.field7 = 350;
  int counter = 0;

  while(true)
  {
    socket.socket_get_frame(&package_in);
    socket.socket_send_frame(package_out);
    //add10_pack(&package_out);

    if(counter==200)
    {
      counter=0;
      if(package_out.field0 == 0)
      {
        package_out.field0 = 500;
        package_out.field1 = 171;
        package_out.field2 = 250;
        package_out.field3 = 355;
        package_out.field4 = 499;
        package_out.field5 = 450;
        package_out.field6 = 298;
        package_out.field7 = 400;
      }
      else
      {
        package_out.field0 = 0;
        package_out.field1 = 0;
        package_out.field2 = 0;
        package_out.field3 = 0;
        package_out.field4 = 0;
        package_out.field5 = 0;
        package_out.field6 = 0;
        package_out.field7 = 0;
      }
    }

    counter++;

    // package_out.field0 = add10(package_out.field0);
    // package_out.field1 = add10(package_out.field1);
    // package_out.field2 = add10(package_out.field2);
    // package_out.field3 = add10(package_out.field3);
    // package_out.field4 = add10(package_out.field4);
    // package_out.field5 = add10(package_out.field5);
    // package_out.field6 = add10(package_out.field6);
    // package_out.field7 = add10(package_out.field7);


    std::cout << "From main:  counter " << counter << std::endl;
    std::cout << package_in.field0 << " " << package_out.field0 << std::endl;
    std::cout << package_in.field1 << " " << package_out.field1 << std::endl;
    std::cout << package_in.field2 << " " << package_out.field2 << std::endl;
    std::cout << package_in.field3 << " " << package_out.field3 << std::endl;
    std::cout << package_in.field4 << " " << package_out.field4 << std::endl;
    std::cout << package_in.field5 << " " << package_out.field5 << std::endl;
    std::cout << package_in.field6 << " " << package_out.field6 << std::endl;
    std::cout << package_in.field7 << " " << package_out.field7 << std::endl;

  }



  socket.socket_close();
  return 0;
}

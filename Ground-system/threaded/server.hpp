/*****************************************************************************
* University of Southern Denmark
* UAS Center
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: client.hpp
* PROJECT....: Autonomous precision landing ground system
* DESCRIPTION: Includes functionality to set up a socket server and send and
*              receive through it.
*              Most of the technical socket connection stuff is from this page:
*              http://www.cs.rpi.edu/~moorthy/Courses/os98/Pgms/socket.html
*****************************************************************************/


/***************************** Include files *******************************/
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <iostream>

/*****************************    Defines    *******************************/
#define PORT "3400"

/*****************************    Structs    *******************************/
/*typedef struct socket_package{
  int field0;
  int field1;
  int field2;
  int field3;
  int field4;
  int field5;
  int field6;
  int field7; // Original packet ends here
  int field8;
  int field9;
  int field10;
  int field11;
  int field12;
  int field13;
  int field14;
  int field15;
  int field16;
  int field17;
  int field18;
  int field19;
} socket_package;*/

/*****************************   Class   *******************************/
class socket_server
{
public:
  socket_server();
//  socket_server(char*, char*);
  char* socket_get_msg();
  int socket_send_msg(char []);
  void socket_close();
  void socket_send_frame(socket_package);
  void socket_get_frame(socket_package*);

private:
  // Functions
  void init_socket_connection();
  void error(char*);
  std::string encode_frame(socket_package);
  int get_field_value(std::string, int*);
  socket_package decode_frame(char []);

  // Variables
//  char port[10] = PORT;
  int sockfd, newsockfd, portno, clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;
  int n;
  char string[256];

  // Control variables
  bool debug = false;


};

socket_server::socket_server()
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Default constructor. Initializes connection.
******************************************************************************/
{
  init_socket_connection();
}


// socket_server::socket_server(char port_in[10],char dummy[10])
// /*****************************************************************************
// *   Input    : server_ip, port_nr
// *   Output   : None
// *   Function : Overload constructor. Initializes connnection with specified ip
// *              and port. Does not work with g++
// ******************************************************************************/
// {
//   strcpy(port,port_in);
//   init_socket_connection();
// }

void socket_server::error(char *msg)
/*****************************************************************************
*   Input    : Error message
*   Output   : None
*   Function : Handles erors.
******************************************************************************/
{
  perror(msg);
  exit(1);
}

void socket_server::init_socket_connection()
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Estabelishes connection.
*              Taken from: http://www.cs.rpi.edu/~moorthy/Courses/os98/Pgms/socket.html
******************************************************************************/
{
  //printf("Waiting for connection on port %s...\n",port);
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error((char *)"ERROR opening socket");
  bzero((char *) &serv_addr, sizeof(serv_addr));
  portno = atoi(PORT);                  // changed from port
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);
  if (bind(sockfd, (struct sockaddr *) &serv_addr,
        sizeof(serv_addr)) < 0)
        error((char *)"ERROR on binding");
  listen(sockfd,5);
  clilen = sizeof(cli_addr);
  newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t*) &clilen);
  if (newsockfd < 0)
    error((char *)"ERROR on accept");
  printf("Connection established.\n");
}

char* socket_server::socket_get_msg()
/*****************************************************************************
*   Input    : None
*   Output   : Received string
*   Function : Receives string.
*              Taken from: http://www.cs.rpi.edu/~moorthy/Courses/os98/Pgms/socket.html
******************************************************************************/
{
  bzero(buffer,256);
  n = read(newsockfd,buffer,255);
  if (n < 0)
    error((char *)"ERROR reading from socket");
  return buffer;
}

int socket_server::socket_send_msg(char message[])
/*****************************************************************************
*   Input    : Message to send
*   Output   : Control variable (<0: error, else succeed)
*   Function : Send string.
*              Taken from: http://www.cs.rpi.edu/~moorthy/Courses/os98/Pgms/socket.html
******************************************************************************/
{
  n = write(newsockfd,message,strlen(message));
  if (n < 0) error((char *)"ERROR writing to socket");
  return n;
}

void socket_server::socket_close()
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Terminates connection.
******************************************************************************/
{
  std::cout << "Closing connection..." << std::endl;
  close(newsockfd);
  close(sockfd);
  std::cout << "Done." << std::endl;
}


socket_package socket_server::decode_frame(char string_in[])
/*****************************************************************************
*   Input    : Received string
*   Output   : Received string as socket_package
*   Function : Converts received string to socket_package
******************************************************************************/
{
  socket_package package_in;
  int m;
  std::string string_string_in(string_in);
  if(debug)
  {
    printf("string_in: %s\n",string_in);
    std::cout << "string_string: " << string_string_in << std::endl;
    std::cout << "string_string length : " << string_string_in.size() << std::endl;
  }
  m = 0;
  package_in.field0 = get_field_value(string_string_in, &m);/*
  package_in.field1 = get_field_value(string_string_in, &m);
  package_in.field2 = get_field_value(string_string_in, &m);
  package_in.field3 = get_field_value(string_string_in, &m);
  package_in.field4 = get_field_value(string_string_in, &m);
  package_in.field5 = get_field_value(string_string_in, &m);
  package_in.field6 = get_field_value(string_string_in, &m);
  package_in.field7 = get_field_value(string_string_in, &m);
  package_in.field8 = get_field_value(string_string_in, &m);
  package_in.field9 = get_field_value(string_string_in, &m);
  package_in.field10 = get_field_value(string_string_in, &m);
  package_in.field11 = get_field_value(string_string_in, &m);
  package_in.field12 = get_field_value(string_string_in, &m);
  package_in.field13 = get_field_value(string_string_in, &m);
  package_in.field14 = get_field_value(string_string_in, &m);
  package_in.field15 = get_field_value(string_string_in, &m);
  package_in.field16 = get_field_value(string_string_in, &m);
  package_in.field17 = get_field_value(string_string_in, &m);
  package_in.field18 = get_field_value(string_string_in, &m);
  package_in.field19 = get_field_value(string_string_in, &m);*/

  return package_in;
}

int socket_server::get_field_value(std::string string_string_in, int *m)
/*****************************************************************************
*   Input    : String with field value
*   Output   : Field value as integer
*   Function : Converts a number in string_string_in to an integer.
******************************************************************************/
{
  std::string str_field;
  for (int i = *m; i < string_string_in.size(); i++){
    if(string_string_in.at(i)==';')
    {
      *m = i+1;
      break;
    }
    else
      str_field+=string_string_in.at(i);
  }
  int retval = std::stoi(str_field);
  if(debug)
    std::cout << retval << std::endl;
  return retval;
}

void socket_server::socket_get_frame(socket_package *package_in)
/*****************************************************************************
*   Input    : socket_package to store frame in
*   Output   : None
*   Function : Receives a frame by calling by socket_get_msg() and decode_frame().
******************************************************************************/
{
  char* string_in =  socket_get_msg();
  *package_in = decode_frame(string_in);
}

std::string socket_server::encode_frame(socket_package package)
/*****************************************************************************
*   Input    : Package to send
*   Output   : Package encoded as a string
*   Function : Convert socket_package to string
******************************************************************************/
{
  std::string string_out;
  string_out += std::to_string(package.field0);
  string_out += ';';/*
  string_out += std::to_string(package.field1);
  string_out += ';';
  string_out += std::to_string(package.field2);
  string_out += ';';
  string_out += std::to_string(package.field3);
  string_out += ';';
  string_out += std::to_string(package.field4);
  string_out += ';';
  string_out += std::to_string(package.field5);
  string_out += ';';
  string_out += std::to_string(package.field6);
  string_out += ';';
  string_out += std::to_string(package.field7);
  string_out += ';';      // Original packet ends here
  string_out += std::to_string(package.field8);
  string_out += ';';
  string_out += std::to_string(package.field9);
  string_out += ';';
  string_out += std::to_string(package.field10);
  string_out += ';';
  string_out += std::to_string(package.field11);
  string_out += ';';
  string_out += std::to_string(package.field12);
  string_out += ';';
  string_out += std::to_string(package.field13);
  string_out += ';';
  string_out += std::to_string(package.field14);
  string_out += ';';
  string_out += std::to_string(package.field15);
  string_out += ';';
  string_out += std::to_string(package.field16);
  string_out += ';';
  string_out += std::to_string(package.field17);
  string_out += ';';
  string_out += std::to_string(package.field18);
  string_out += ';';
  string_out += std::to_string(package.field19);
  string_out += ';';*/
  //retval"return_value";
  //return string_out;
  return string_out;
}

void socket_server::socket_send_frame(socket_package package_out)
/*****************************************************************************
*   Input    : None
*   Output   : socket_package to send
*   Function : Sends a frame by calling encode_frame() and socket_send_msg().
******************************************************************************/
{
  char* retval;
  std::string string_out = encode_frame(package_out);
  const char* temp_str = string_out.c_str();
  if(debug)
    std::cout << "string_out: " << temp_str << std::endl;
  retval=const_cast<char*>(temp_str);
  socket_send_msg(retval);
}

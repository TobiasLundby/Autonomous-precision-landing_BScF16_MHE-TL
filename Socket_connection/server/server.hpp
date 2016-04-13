/* A simple server in the internet domain using TCP
   The port number is passed as an argument
   INSPIRED BY: http://www.cs.rpi.edu/~moorthy/Courses/os98/Pgms/socket.html
   May solve problems: http://hea-www.harvard.edu/~fine/Tech/addrinuse.html

   */

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


typedef struct socket_package{
  int field0;
  int field1;
  int field2;
  int field3;
  int field4;
  int field5;
  int field6;
  int field7;
} socket_package;

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
{
  init_socket_connection();
}

/*
socket_server::socket_server(char port_in[10],char dummy[10])
{
  strcpy(port,port_in);
  init_socket_connection();
}
*/
void socket_server::error(char *msg)
{
  perror(msg);
  exit(1);
}

void socket_server::init_socket_connection()
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
{
  bzero(buffer,256);
  n = read(newsockfd,buffer,255);
  if (n < 0)
    error((char *)"ERROR reading from socket");
  return buffer;
}

int socket_server::socket_send_msg(char message[])
{
  n = write(newsockfd,message,strlen(message));
  if (n < 0) error((char *)"ERROR writing to socket");
  return n;
}

void socket_server::socket_close()
{
  std::cout << "Closing connection..." << std::endl;
  close(newsockfd);
  close(sockfd);
  std::cout << "Done." << std::endl;
}


socket_package socket_server::decode_frame(char string_in[])
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
  package_in.field0 = get_field_value(string_string_in, &m);
  package_in.field1 = get_field_value(string_string_in, &m);
  package_in.field2 = get_field_value(string_string_in, &m);
  package_in.field3 = get_field_value(string_string_in, &m);
  package_in.field4 = get_field_value(string_string_in, &m);
  package_in.field5 = get_field_value(string_string_in, &m);
  package_in.field6 = get_field_value(string_string_in, &m);
  package_in.field7 = get_field_value(string_string_in, &m);

  return package_in;
}

int socket_server::get_field_value(std::string string_string_in, int *m)
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
{
  char* string_in =  socket_get_msg();
  *package_in = decode_frame(string_in);
}

std::string socket_server::encode_frame(socket_package package)
{
  std::string string_out;
  string_out += std::to_string(package.field0);
  string_out += ';';
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
  string_out += ';';
  //retval"return_value";
  //return string_out;
  return string_out;
}

void socket_server::socket_send_frame(socket_package package_out)
{
  char* retval;
  std::string string_out = encode_frame(package_out);
  const char* temp_str = string_out.c_str();
  if(debug)
    std::cout << "string_out: " << temp_str << std::endl;
  retval=const_cast<char*>(temp_str);
  socket_send_msg(retval);
}

/*
  INSPIRED BY: http://www.cs.rpi.edu/~moorthy/Courses/os98/Pgms/socket.html
*/

/***************************** Include files *******************************/
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netdb.h>
#include <string.h>
#include <string>
#include <iostream>


/*****************************    Defines    *******************************/
#define SERVER_IP "192.168.43.193"
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

/*****************************   Class   ***********************************/
class socket_client
{
public:
  socket_client();
//  socket_client(char *, char *);
  char* socket_get_msg();
  int socket_send_msg(char *);
  void socket_close();
  void socket_send_frame(socket_package);
  void socket_get_frame(socket_package*);

private:
  // Functions
  void error(char *msg);
  void init_socket_connection();
  std::string encode_frame(socket_package);
  int get_field_value(std::string, int*);
  socket_package decode_frame(char []);

  // Variables
  //char server_ip[20] = SERVER_IP;
  //char port[10] = PORT;
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[256];

  // Control variables
  bool debug = false;

};

socket_client::socket_client()
{
  init_socket_connection();
}
/*
socket_client::socket_client(char server_ip_in[20], char port_in[10])
{
  printf("%s %s\n",server_ip_in,port_in);
  strcpy (server_ip,server_ip_in);
  strcpy (port,port_in);

  printf("%s %s\n",server_ip,port);
  init_socket_connection();
}
*/

void socket_client::error(char *msg)
{
  perror(msg);
  exit(0);
}

void socket_client::init_socket_connection()
{
  //printf("Opening socket on %s:%s...\n",server_ip,port);
  portno = atoi(PORT);                      // changed from port
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error((char*)"ERROR opening socket");
  server = gethostbyname(SERVER_IP);      // changed from server_ip
  if (server == NULL) {
    fprintf(stderr,"ERROR, no such host\n");
    exit(0);
  }
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
       (char *)&serv_addr.sin_addr.s_addr,
       server->h_length);
  serv_addr.sin_port = htons(portno);
  if (connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0)
    error((char*)"ERROR connecting");
  printf("Done.\n");
}


int socket_client::socket_send_msg(char message[])
{
  n = write(sockfd,message,strlen(message));
  if (n < 0)
    error((char*)"ERROR writing to socket");

  return n;
}

char* socket_client::socket_get_msg()
{
  bzero(buffer,256);
  n = read(sockfd,buffer,255);
  if (n < 0)
    error((char*)"ERROR reading from socket");
  return buffer;
}

void socket_client::socket_close()
{
  std::cout << "Closing connection..." << std::endl;
  close(sockfd);
  std::cout << "Done." << std::endl;
}

std::string socket_client::encode_frame(socket_package package)
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

void socket_client::socket_send_frame(socket_package package_out)
{
  char* retval;
  std::string string_out = encode_frame(package_out);
  const char* temp_str = string_out.c_str();
  if(debug)
    std::cout << "string_out: " << temp_str << std::endl;
  retval=const_cast<char*>(temp_str);
  socket_send_msg(retval);
}


socket_package socket_client::decode_frame(char string_in[])
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

int socket_client::get_field_value(std::string string_string_in, int *m)
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

void socket_client::socket_get_frame(socket_package *package_in)
{
  char* string_in =  socket_get_msg();
  *package_in = decode_frame(string_in);
}

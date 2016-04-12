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

/*****************************    Defines    *******************************/
#define PORT "3400"

/*****************************   Class   *******************************/
class socket_server
{
public:
  socket_server();
  socket_server(char*, char*);
  int socket_get_msg();
  int socket_send_msg(char []);
  void socket_close();

private:
  // Functions
  void init_socket_connection();
  void error(char*);

  // Variables
  char port[10] = PORT;
  int sockfd, newsockfd, portno, clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;
  int n;
  char string[256];


};

socket_server::socket_server()
{
  init_socket_connection();
}


socket_server::socket_server(char port_in[10],char dummy[10])
{
  strcpy(port,port_in);
  init_socket_connection();
}

void socket_server::error(char *msg)
{
  perror(msg);
  exit(1);
}

void socket_server::init_socket_connection()
{
  printf("Waiting for connection on port %s...\n",port);
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error((char *)"ERROR opening socket");
  bzero((char *) &serv_addr, sizeof(serv_addr));
  portno = atoi(port);
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

int socket_server::socket_get_msg()
{
  bzero(buffer,256);
  n = read(newsockfd,buffer,255);
  if (n < 0)
    error((char *)"ERROR reading from socket");
  printf("Here is the message: %s\n",buffer);
  return n;
}

int socket_server::socket_send_msg(char message[])
{
  n = write(newsockfd,message,strlen(message));
  if (n < 0) error((char *)"ERROR writing to socket");
  return n;
}

void socket_server::socket_close()
{
  close(newsockfd);
  close(sockfd);
}

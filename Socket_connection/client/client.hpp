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


/*****************************    Defines    *******************************/
#define SERVER_IP "192.168.43.193"
#define PORT "3400"

/*****************************   Class   ***********************************/
class socket_client
{
public:
  socket_client();
  socket_client(char *, char *);
  int socket_get_msg();
  int socket_send_msg(char message[]);
  void socket_close();

private:
  // Functions
  void error(char *msg);
  void init_socket_connection();

  // Variables
  char server_ip[20] = SERVER_IP;
  char port[10] = PORT;
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[256];

};

socket_client::socket_client()
{
  init_socket_connection();
}

socket_client::socket_client(char server_ip_in[20], char port_in[10])
{
  printf("%s %s\n",server_ip_in,port_in);
  strcpy (server_ip,server_ip_in);
  strcpy (port,port_in);

  printf("%s %s\n",server_ip,port);
  init_socket_connection();
}


void socket_client::error(char *msg)
{
  perror(msg);
  exit(0);
}

void socket_client::init_socket_connection()
{
  printf("Opening socket on %s:%s...\n",server_ip,port);
  portno = atoi(port);
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error((char*)"ERROR opening socket");
  server = gethostbyname(server_ip);
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

int socket_client::socket_get_msg()
{
  bzero(buffer,256);
  n = read(sockfd,buffer,255);
  if (n < 0)
    error((char*)"ERROR reading from socket");
  printf("%s\n",buffer);
  return n;
}

void socket_client::socket_close()
{
  close(sockfd);
}

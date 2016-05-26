/*****************************************************************************
* University of Southern Denmark
* UAS Center
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: interthread_communication.cpp
* PROJECT....: Autonomous precision landing ground system
* DESCRIPTION: Declares datatypes and control types for interthread communication
*
*****************************************************************************/

#pragma once
#include <pthread.h>
//#include "server.hpp"

typedef struct socket_package{
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
} socket_package;

// Data types
socket_package sock_pack_in, sock_pack_out; // Packages for socket in and out

// Control types
pthread_mutex_t mutex_sock_pack_in, mutex_sock_pack_out; // Mutexes for socket in and out

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
#include "server.hpp"

// Data types
socket_package sock_pack_in, sock_pack_out; // Packages for socket in and out

// Control types
pthread_mutex_t mutex_sock_pack_in, mutex_sock_pack_out; // Mutexes for socket in and out

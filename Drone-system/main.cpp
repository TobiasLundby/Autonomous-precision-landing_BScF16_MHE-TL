/*****************************************************************************
* University of Southern Denmark
* UAS Center
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: main.cpp
* PROJECT....: Autonomous precision landing ground system
* DESCRIPTION: ???
*
*****************************************************************************/

#include <stdio.h>
#include <string>

#include "DSM_analyser.hpp"

using namespace std;


int main(int argc,char* argv[])
{
  DSM_RX_TX serial_con("/dev/ttyAMA0");

  printf("Waiting for packet\n");
  serial_con.DSM_analyse(false);

  return 0;
}

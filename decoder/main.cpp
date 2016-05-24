/*****************************************************************************
* University of Southern Denmark
* NUM
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: main.cpp
* PROJECT....:
* DESCRIPTION:
*
*****************************************************************************/

/***************************** Include files *******************************/
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

/*****************************   Namespaces  *******************************/
using namespace std;

/*****************************   Structs   *******************************/

/*****************************    Defines    *******************************/

/*****************************    Functions    *******************************/

/*****************************    MAIN    *******************************/

int main(void) {
	int add_val = 20;

	int high_byte_in = 0x0C;
	int low_byte_in  = 0x10;
	cout << endl;
	cout << "Please enter high byte (as hex): ";
	cin >> hex >> high_bylte_in;
	cout << "Please enter low byte (as hex): ";
	cin >> hex >> low_byte_in;
	cout << endl;

	int chan_num = (high_byte_in>>3) & 0x0f;
	int chan_val = ((high_byte_in & 0x07)<<8) | low_byte_in;

	cout << "[INPUT] Channel: " << chan_num << ", value: " << chan_val << endl;

	chan_val += add_val;

	cout << "[OUTPUT] Channel: " << chan_num << ", value: " << chan_val << endl;
	cout << endl;

	int high_byte_out = ((chan_num<<3)|((chan_val & 0x700) >> 8));
	int low_byte_out  = (chan_val & 0xFF);

	cout << "High byte (as hex): ";
	cout << hex << high_byte_out;
	cout << ", low byte (as hex): ";
	cout << hex << low_byte_out << endl;

	return 0;
}

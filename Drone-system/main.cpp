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

/***************************** Include files *******************************/
#include <stdio.h>
#include <string>

#include "DSM_analyser.hpp"

/*****************************   Namespaces  *******************************/
using namespace std;

/*****************************   Structs   *******************************/

/*****************************    Defines    *******************************/
#define MAIN_S_INIT         0
#define MAIN_S_OUTSIDE_VIEW 1
#define MAIN_S_INSIDE_VIEW  2
#define MAIN_S_FORCE_DOWN   3

void print_in_frame(DSM_RX_TX &con)
{
    printf(
        "Channel  Value\n0\t%i\n1\t%i\n2\t%i\n3\t%i\n4\t%i\n5\t%i\n6\t%i\n",
        con.get_in_channel_value(0),
        con.get_in_channel_value(1),
        con.get_in_channel_value(2),
        con.get_in_channel_value(3),
        con.get_in_channel_value(4),
        con.get_in_channel_value(5),
        con.get_in_channel_value(6)
    );
}


int main(int argc,char* argv[])
{
    int MAIN_STATE = MAIN_S_INIT;
    bool error = false;
    //DSM_RX_TX serial_con("/dev/ttyAMA0");
    DSM_RX_TX serial_con;

    int ch0_off = 0;
    int ch1_off = 0;
    int ch2_off = 0;
    int ch3_off = 0;
    int ch4_off = 0;
    int ch5_off = 0;
    int ch6_off = 0;

    while (!error) {
        switch (MAIN_STATE) {
            case MAIN_S_INIT:
                serial_con.DSM_analyse(false); // RX and TX until it is safe.
                MAIN_STATE = MAIN_S_FORCE_DOWN; // Change state
                printf("Switching from MAIN_S_INIT to MAIN_S_FORCE_DOWN"); // Debug for change state
                break;
            case MAIN_S_OUTSIDE_VIEW:
                break;
            case MAIN_S_INSIDE_VIEW:
                break;
            case MAIN_S_FORCE_DOWN:
                ch0_off = -20;
                serial_con.change_channel_offsets(ch0_off,ch1_off,ch2_off,ch3_off,ch4_off,ch5_off,ch6_off);
                print_in_frame(serial_con);
                break;
        }
        serial_con.DSM_analyse(false); /* RX AND TX */
        /* HOEJGAARD PUT YOUR SOCKET RX TX HERE */
    }

    /* FUN STUFF - DO NOT RUN ON DRONE - ONLY FOR QGroundControl tests
    while (true) {
        if (ch0_off > 200)
            ch0_off = 0;
        else
            ch0_off++;
        if (ch1_off > 200)
            ch1_off = 0;
        else
            ch1_off++;
        if (ch2_off > 200)
            ch2_off = 0;
        else
            ch2_off++;
        if (ch3_off > 200)
            ch3_off = 0;
        else
            ch3_off++;
        if (ch4_off > 200)
            ch4_off = 0;
        else
            ch4_off++;
        if (ch5_off > 200)
            ch5_off = 0;
        else
            ch5_off++;
        if (ch6_off > 200)
            ch6_off = 0;
        else
            ch6_off++;

        serial_con.change_channel_offsets(ch0_off,ch1_off,ch2_off,ch3_off,ch4_off,ch5_off,ch6_off);
        serial_con.DSM_analyse(false);
    }

    printf("Something went wrong");
    */

    return 0;
}

/*
void print_out_frame(con)
{
    printf(
        "Channel \t Value\n0\t%i\n1\t%i\n2\t%i\n3\t%i\n4\t%i\n5\t%i\n6\t%i\n",
        serial_con.get_out_channel_value(0),
        serial_con.get_out_channel_value(1),
        serial_con.get_out_channel_value(2),
        serial_con.get_out_channel_value(3),
        serial_con.get_out_channel_value(4),
        serial_con.get_out_channel_value(5),
        serial_con.get_out_channel_value(6)
    );
}
*/

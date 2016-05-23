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
#include "client.hpp"

/*****************************   Namespaces  *******************************/
using namespace std;

/*****************************   Structs   *******************************/

/*****************************    Defines    *******************************/
#define MAIN_S_INIT         0
#define MAIN_S_OUTSIDE_VIEW 1
#define MAIN_S_INSIDE_VIEW  2
#define MAIN_S_FORCE_DOWN   3
#define MAIN_S_TEST_DSM     4  // For test of DSM_analyzer

#define OFFSET_MAX_CHANGE   10 // The max change in the offset value each 22ms / packet
#define OFFSET_MAX_RANGE    501 // Max range for the offsets

#define CH4POS0             1704 //
#define CH4POS1             1192 //
#define CH4POS2             340  //

#define CH5POS0             340 //
#define CH5POS1             1704 //
#define CH5THR              5

/*****************************    Functions    *******************************/
void print_in_frame(DSM_RX_TX &con)
{
  printf(
        "\nRX frame\nChannel  Value\n0\t%i\n1\t%i\n2\t%i\n3\t%i\n4\t%i\n5\t%i\n6\t%i\n7\t%i\n",
        con.get_in_channel_value(0),
        con.get_in_channel_value(1),
        con.get_in_channel_value(2),
        con.get_in_channel_value(3),
        con.get_in_channel_value(4),
        con.get_in_channel_value(5),
        con.get_in_channel_value(6),
        con.get_in_channel_value(7)
    );
}

void print_out_frame(DSM_RX_TX &con)
{
    printf(
        "\nTX frame\nChannel  Value\n0\t%i\n1\t%i\n2\t%i\n3\t%i\n4\t%i\n5\t%i\n6\t%i\n7\t%i\n",
        con.get_out_channel_value(0),
        con.get_out_channel_value(1),
        con.get_out_channel_value(2),
        con.get_out_channel_value(3),
        con.get_out_channel_value(4),
        con.get_out_channel_value(5),
        con.get_out_channel_value(6),
        con.get_out_channel_value(7)
    );
}

/*****************************    MAIN    *******************************/
int main(int argc,char* argv[])
{
    int MAIN_STATE = MAIN_S_INIT;
    bool error = false;
    bool packet_type = 0;

    /* Various for client start */
       // socket_client socket;
       // socket_package sock_pack_in, sock_pack_out;
       // sock_pack_out.field0=0;
       // sock_pack_out.field1=1;
       // sock_pack_out.field2=2;
       // sock_pack_out.field3=3;
       // sock_pack_out.field4=4;
       // sock_pack_out.field5=5;
       // sock_pack_out.field6=6;
       // sock_pack_out.field7=7;
       // sock_pack_out.field8=0;
       // sock_pack_out.field9=0;
       // sock_pack_out.field10=0;
       // sock_pack_out.field11=0;
       // sock_pack_out.field12=0;
       // sock_pack_out.field13=0;
       // sock_pack_out.field14=0;
       // sock_pack_out.field15=0;
       // sock_pack_out.field16=0;
       // sock_pack_out.field17=0;
       // sock_pack_out.field18=0;
       // sock_pack_out.field19=1;

       // socket.socket_send_frame(sock_pack_out);
       // socket.socket_get_frame(&sock_pack_in);
       // cout << sock_pack_in.field0;
       // cout << sock_pack_in.field1;
       // cout << sock_pack_in.field2;
       // cout << sock_pack_in.field3;
       // cout << sock_pack_in.field4;
       // cout << sock_pack_in.field5;
       // cout << sock_pack_in.field6;
       // cout << sock_pack_in.field7;
       // cout << sock_pack_in.field8;
       // cout << sock_pack_in.field9;
       // cout << sock_pack_in.field10;
       // cout << sock_pack_in.field11;
       // cout << sock_pack_in.field12;
       // cout << sock_pack_in.field13;
       // cout << sock_pack_in.field14;
       // cout << sock_pack_in.field15;
       // cout << sock_pack_in.field16;
       // cout << sock_pack_in.field17;
       // cout << sock_pack_in.field18;
       // cout << sock_pack_in.field19 << endl;

    /* Various for client end */



    /* Various for DSM_analyser start */
      //DSM_RX_TX serial_con("/dev/ttyAMA0");
      DSM_RX_TX serial_con;
      int ch0_off = 0;
      int ch0_off_goal = 0;
      int ch1_off = 0;
      int ch1_off_goal = 0;
      int ch2_off = 0;
      int ch2_off_goal = 0;
      int ch3_off = 0;
      int ch3_off_goal = 0;
      int ch4_off = 0;
      int ch4_off_goal = 0;
      int ch5_off = 0;
      int ch5_off_goal = 0;
      int ch6_off = 0;
      int ch6_off_goal = 0;
      int ch7_off = 0;
      int ch7_off_goal = 0;
    /* Various for DSM_analyser end */

    while (!error) {
        switch (MAIN_STATE) {
            case MAIN_S_INIT:
                serial_con.DSM_analyse(false); // RX and TX until it is safe.
                MAIN_STATE = MAIN_S_TEST_DSM;
                printf("Switching from MAIN_S_INIT to MAIN_S_TEST_DSM"); // Debug for change state
                // MAIN_STATE = MAIN_S_FORCE_DOWN; // Change state
                // printf("Switching from MAIN_S_INIT to MAIN_S_FORCE_DOWN"); // Debug for change state
                //serial_con.enable_all_max();
                break;
            case MAIN_S_OUTSIDE_VIEW:
                break;
            case MAIN_S_INSIDE_VIEW:
                break;
            case MAIN_S_FORCE_DOWN:

                 // if (sock_pack_in.field0 > -OFFSET_MAX_RANGE and sock_pack_in.field0 < OFFSET_MAX_RANGE)
                 //     ch0_off_goal = sock_pack_in.field0;
                 // if (sock_pack_in.field1 > -OFFSET_MAX_RANGE and sock_pack_in.field1 < OFFSET_MAX_RANGE)
                 //     ch1_off_goal = sock_pack_in.field1;
                 // if (sock_pack_in.field2 > -OFFSET_MAX_RANGE and sock_pack_in.field2 < OFFSET_MAX_RANGE)
                 //     ch2_off_goal = sock_pack_in.field2;
                 // if (sock_pack_in.field3 > -OFFSET_MAX_RANGE and sock_pack_in.field3 < OFFSET_MAX_RANGE)
                 //     ch3_off_goal = sock_pack_in.field3;
                 // if (sock_pack_in.field4 > -OFFSET_MAX_RANGE and sock_pack_in.field4 < OFFSET_MAX_RANGE)
                 //     ch4_off_goal = sock_pack_in.field4;
                 // if (sock_pack_in.field5 > -OFFSET_MAX_RANGE and sock_pack_in.field5 < OFFSET_MAX_RANGE)
                 //     ch5_off_goal = sock_pack_in.field5;
                 // if (sock_pack_in.field6 > -OFFSET_MAX_RANGE and sock_pack_in.field6 < OFFSET_MAX_RANGE)
                 //     ch6_off_goal = sock_pack_in.field6;

                 if (ch0_off > ch0_off_goal)
                     ch0_off -= OFFSET_MAX_CHANGE;
                 else if (ch0_off < ch0_off_goal)
                     ch0_off += OFFSET_MAX_CHANGE;
                 if (ch1_off > ch1_off_goal)
                     ch1_off -= OFFSET_MAX_CHANGE;
                 else if (ch1_off < ch1_off_goal)
                     ch1_off += OFFSET_MAX_CHANGE;
                 if (ch2_off > ch2_off_goal)
                     ch2_off -= OFFSET_MAX_CHANGE;
                 else if (ch2_off < ch2_off_goal)
                     ch2_off += OFFSET_MAX_CHANGE;
                 if (ch3_off > ch3_off_goal)
                     ch3_off -= OFFSET_MAX_CHANGE;
                 else if (ch3_off < ch3_off_goal)
                     ch3_off += OFFSET_MAX_CHANGE;
                 if (ch4_off > ch4_off_goal)
                     ch4_off -= OFFSET_MAX_CHANGE;
                 else if (ch4_off < ch4_off_goal)
                     ch4_off += OFFSET_MAX_CHANGE;
                 if (ch5_off > ch5_off_goal)
                     ch5_off -= OFFSET_MAX_CHANGE;
	               else if (ch5_off < ch5_off_goal)
                     ch5_off += OFFSET_MAX_CHANGE;
                 if (ch6_off > ch6_off_goal)
                     ch6_off -= OFFSET_MAX_CHANGE;
                 else if (ch6_off < ch6_off_goal)
                     ch6_off += OFFSET_MAX_CHANGE;

                serial_con.change_channel_offsets(ch0_off,ch1_off,ch2_off,ch3_off,ch4_off,ch5_off,ch6_off);

                //printf("Value from socket is %i and current offset is %i\n", ch0_off_goal, ch0_off);
//                print_in_frame(serial_con);
                //print_out_frame(serial_con);

                // if (serial_con.get_in_channel_value(5) < CH5POS1 - CH5THR and serial_con.get_in_channel_value(5) > CH5POS1 + CH5THR) {
                //      if(sock_pack_in.field0 == 1)
                //        ch0_off_goal = -20;
                // }

                //  sock_pack_out.field0 = serial_con.get_out_channel_value(0);
                //  sock_pack_out.field1 = serial_con.get_out_channel_value(1);
                //  sock_pack_out.field2 = serial_con.get_out_channel_value(2);
                //  sock_pack_out.field3 = serial_con.get_out_channel_value(3);
                //  sock_pack_out.field4 = serial_con.get_out_channel_value(4);
                //  sock_pack_out.field5 = serial_con.get_out_channel_value(5);
                //  sock_pack_out.field6 = serial_con.get_out_channel_value(6);
                //  sock_pack_out.field7 = 0;

                break;
            case MAIN_S_TEST_DSM:
                if(packet_type==1)
                {
                  int twenty = 20;
                  serial_con.change_channel_offsets(twenty,twenty,twenty,twenty,twenty,twenty,twenty);
                  packet_type = 0;
                }
                else
                {
                  packet_type = 1;
                  int twenty = 20;
                  serial_con.change_channel_offsets(twenty,twenty,twenty,twenty,twenty,twenty,twenty);
                }
                break;
        }
        serial_con.DSM_analyse(false); /* RX AND TX */
        // socket.socket_send_frame(sock_pack_out);
        // socket.socket_get_frame(&sock_pack_in);

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

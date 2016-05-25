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
#include <iostream>
#include <fstream>
#include <thread>

#include "DSM_analyser.hpp"
#include "client.hpp"

/*****************************   Namespaces  *******************************/
using namespace std;

/*****************************   Structs   *******************************/
typedef struct control_params{
  double Kp   = 0;
  double Ki   = 0;
  double Kd   = 0;
  double error      = 0;
  double error_pre  = 0;
  double integral   = 0;
  double min        = 0;
  double max        = 0;
} control_params;
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

#define dt                  0.022

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

int p_controller(double setpoint, double current, control_params &params)
{
    double error = setpoint - current;
    params.error_pre = params.error;
    params.error = error;

    double Pout = params.Kp * error;

    params.integral += error * dt;
    double Iout = params.Ki * params.integral;

    double derivative = (params.error - params.error_pre) / dt;
    double Dout = params.Kd * derivative;

    double output = Pout + Iout + Dout;

    if( output > params.max )
        output = params.max;
    else if( output < params.min )
        output = params.min;

    return output;
}

/*****************************    MAIN    *******************************/
int main(int argc,char* argv[])
{
    ofstream data_logger;
    data_logger.open ("data_log.csv");
    data_logger << "time,packet_number\n"; //,ground_sync_frame,MAIN_STATE,ch0_in,ch1_in,ch2_in,ch3_in,ch4_in,ch5_in,ch6_in,ch0_out,ch1_out,ch2_out,ch3_out,ch4_out,ch5_out,ch6_out,ch0_off,ch1_off,ch2_off,ch3_off,ch4_off,ch5_off,ch6_off,socket_x,socket_y,socket_z,socket_dx,socket_dy,socket_dz,pid_x_error,pid_y_error,pid_z_error,packet_errors\n";
    int MAIN_STATE = MAIN_S_INIT;
    int packet_number = 0;
    bool error = false;
    bool packet_type = 0;

    /* Various for client start */
       socket_client socket;
       socket_package sock_pack_in, sock_pack_out;
       sock_pack_out.field0=0;
       sock_pack_out.field1=1;
       sock_pack_out.field2=2;
       sock_pack_out.field3=3;
       sock_pack_out.field4=4;
       sock_pack_out.field5=5;
       sock_pack_out.field6=6;
       sock_pack_out.field7=7;
       sock_pack_out.field8=0;
       sock_pack_out.field9=0;
       sock_pack_out.field10=0;
       sock_pack_out.field11=0;
       sock_pack_out.field12=0;
       sock_pack_out.field13=0;
       sock_pack_out.field14=0;
       sock_pack_out.field15=0;
       sock_pack_out.field16=0;
       sock_pack_out.field17=0;
       sock_pack_out.field18=0;
       sock_pack_out.field19=0;

       socket.socket_send_frame(sock_pack_out);
       socket.socket_get_frame(&sock_pack_in);
       cout << sock_pack_in.field0; // control, 0=not in frame, 1=in frame position transmitted and control
       cout << sock_pack_in.field1; // x, drone pos
       cout << sock_pack_in.field2; // y, drone pos
       cout << sock_pack_in.field3; // z, drone pos
       cout << sock_pack_in.field4; // phi, yaw
       cout << sock_pack_in.field5; // theta, pitch
       cout << sock_pack_in.field6; // psi, roll
       cout << sock_pack_in.field7; // delta x, target pos offset
       cout << sock_pack_in.field8; // delta y, target pos offset
       cout << sock_pack_in.field9; // delta z, target pos offset
       cout << sock_pack_in.field10; // delta phi
       cout << sock_pack_in.field11; // delta theta
       cout << sock_pack_in.field12; // delta psi
       cout << sock_pack_in.field13; // ch0 val
       cout << sock_pack_in.field14; // ch1 val
       cout << sock_pack_in.field15; // ch2 val
       cout << sock_pack_in.field16; // ch3 val
       cout << sock_pack_in.field17; // ch4 val
       cout << sock_pack_in.field18; // ch5 val
       cout << sock_pack_in.field19 << endl;  // ch6 val //// USED FOR SYNC BETWEEN LOGGERS

    /* Various for client end */

    int ch0_off = 0;
    int ch1_off = 0;
    int ch2_off = 0;
    int ch3_off = 0;
    int ch4_off = 0;
    int ch5_off = 0;
    int ch6_off = 0;

    control_params x_params, y_params, z_params;
    x_params.Kp = 0.001;
    x_params.min = -100; // defines the min control value
    x_params.max = 100;  // defines the max control value
    y_params.Kp = 0.001;
    y_params.min = -100; // defines the min control value
    y_params.max = 100;  // defines the max control value
    z_params.Kp = 0.001;
    z_params.min = -100; // defines the min control value
    z_params.max = 100;  // defines the max control value


    /* Various for DSM_analyser start */
      //DSM_RX_TX serial_con("/dev/ttyAMA0");
      DSM_RX_TX serial_con;
    /* Various for DSM_analyser end */

    while (!error) {
        switch (MAIN_STATE) {
            case MAIN_S_INIT:
                serial_con.DSM_analyse(false); // RX and TX until it is safe.
                //MAIN_STATE = MAIN_S_TEST_DSM;
                //printf("Switching from MAIN_S_INIT to MAIN_S_TEST_DSM\n"); // Debug for change state
                MAIN_STATE = MAIN_S_FORCE_DOWN; // Change state
                printf("Switching from MAIN_S_INIT to MAIN_S_FORCE_DOWN"); // Debug for change state
                //serial_con.enable_all_max();
                break;
            case MAIN_S_OUTSIDE_VIEW:
                break;
            case MAIN_S_INSIDE_VIEW:
                break;
            case MAIN_S_FORCE_DOWN:
                /*
                 if (sock_pack_in.field0 > -OFFSET_MAX_RANGE and sock_pack_in.field0 < OFFSET_MAX_RANGE)
                     ch0_off_goal = sock_pack_in.field0;
                 if (sock_pack_in.field1 > -OFFSET_MAX_RANGE and sock_pack_in.field1 < OFFSET_MAX_RANGE)
                     ch1_off_goal = sock_pack_in.field1;
                 if (sock_pack_in.field2 > -OFFSET_MAX_RANGE and sock_pack_in.field2 < OFFSET_MAX_RANGE)
                     ch2_off_goal = sock_pack_in.field2;
                 if (sock_pack_in.field3 > -OFFSET_MAX_RANGE and sock_pack_in.field3 < OFFSET_MAX_RANGE)
                     ch3_off_goal = sock_pack_in.field3;
                 if (sock_pack_in.field4 > -OFFSET_MAX_RANGE and sock_pack_in.field4 < OFFSET_MAX_RANGE)
                     ch4_off_goal = sock_pack_in.field4;
                 if (sock_pack_in.field5 > -OFFSET_MAX_RANGE and sock_pack_in.field5 < OFFSET_MAX_RANGE)
                     ch5_off_goal = sock_pack_in.field5;
                 if (sock_pack_in.field6 > -OFFSET_MAX_RANGE and sock_pack_in.field6 < OFFSET_MAX_RANGE)
                     ch6_off_goal = sock_pack_in.field6;

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
                */
                ch1_off = p_controller(sock_pack_in.field7, sock_pack_in.field1, x_params); // Roll
                ch2_off = p_controller(sock_pack_in.field8, sock_pack_in.field2, y_params); // Pitch
                ch0_off = p_controller(sock_pack_in.field9, sock_pack_in.field3, z_params); // Throttle

                if (serial_con.get_in_channel_value(5) < CH5POS1 - CH5THR and serial_con.get_in_channel_value(5) > CH5POS1 + CH5THR) {
                     if(sock_pack_in.field0 == 1)
                       serial_con.change_channel_offsets(ch0_off,ch1_off,ch2_off,ch3_off,ch4_off,ch5_off,ch6_off);
                } else {
                    serial_con.change_channel_offsets(0,0,0,0,0,0,0);
                }

                //print_in_frame(serial_con);
                //print_out_frame(serial_con);

                /*
                if (serial_con.get_in_channel_value(5) < CH5POS1 - CH5THR and serial_con.get_in_channel_value(5) > CH5POS1 + CH5THR) {
                     if(sock_pack_in.field0 == 1)
                       ch0_off_goal = -20;
                }
                */

                 sock_pack_out.field0 = serial_con.get_out_channel_value(0);
                 sock_pack_out.field1 = serial_con.get_out_channel_value(1);
                 sock_pack_out.field2 = serial_con.get_out_channel_value(2);
                 sock_pack_out.field3 = serial_con.get_out_channel_value(3);
                 sock_pack_out.field4 = serial_con.get_out_channel_value(4);
                 sock_pack_out.field5 = serial_con.get_out_channel_value(5);
                 sock_pack_out.field6 = serial_con.get_out_channel_value(6);
                 sock_pack_out.field7 = 0;

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

        /*
        socket.socket_send_frame(sock_pack_out);
        socket.socket_get_frame(&sock_pack_in);
        int int_in_factor = 1000;
        sock_pack_in.field1  = sock_pack_in.field1/int_in_factor; // x, drone pos
        sock_pack_in.field2  = sock_pack_in.field2/int_in_factor; // y, drone pos
        sock_pack_in.field3  = sock_pack_in.field3/int_in_factor; // z, drone pos
        sock_pack_in.field4  = sock_pack_in.field4/int_in_factor; // phi, yaw
        sock_pack_in.field5  = sock_pack_in.field5/int_in_factor; // theta, pitch
        sock_pack_in.field6  = sock_pack_in.field6/int_in_factor; // psi, roll
        sock_pack_in.field7  = sock_pack_in.field7/int_in_factor; // delta x, target pos offset
        sock_pack_in.field8  = sock_pack_in.field8/int_in_factor; // delta y, target pos offset
        sock_pack_in.field9  = sock_pack_in.field9/int_in_factor; // delta z, target pos offset
        sock_pack_in.field10 = sock_pack_in.field10/int_in_factor; // delta phi
        sock_pack_in.field11 = sock_pack_in.field11/int_in_factor; // delta theta
        sock_pack_in.field12 = sock_pack_in.field12/int_in_factor; // delta psi
        */


        data_logger <<
            serial_con.get_time() << "," << packet_number << "\n"; /* << "," << sock_pack_in.field19 << "," << MAIN_STATE << "," <<
            serial_con.get_in_channel_value(0) << "," << serial_con.get_in_channel_value(1) << "," << serial_con.get_in_channel_value(2) << "," << serial_con.get_in_channel_value(3) << "," << serial_con.get_in_channel_value(4) << "," << serial_con.get_in_channel_value(5) << "," << serial_con.get_in_channel_value(6) << "," <<
            serial_con.get_out_channel_value(0) << "," << serial_con.get_out_channel_value(1) << "," << serial_con.get_out_channel_value(2) << "," << serial_con.get_out_channel_value(3) << "," << serial_con.get_out_channel_value(4) << "," << serial_con.get_out_channel_value(5) << "," << serial_con.get_out_channel_value(6) << "," <<
            ch0_off << "," << ch1_off << "," << ch2_off << "," << ch3_off << "," << ch4_off << "," << ch5_off << "," << ch6_off << "," <<
            sock_pack_in.field1 << "," << sock_pack_in.field2 << "," << sock_pack_in.field3 << "," << sock_pack_in.field7 << "," << sock_pack_in.field8 << "," << sock_pack_in.field9 << "," <<
            x_params.error << "," << y_params.error << "," << z_params.error << "," << serial_con.get_packet_errors() << "\n"; */
        packet_number++;

    }
    return 0;
}

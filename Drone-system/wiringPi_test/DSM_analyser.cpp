/*****************************************************************************
* University of Southern Denmark
* UAS Center
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: DSM_analyser.cpp
* PROJECT....: Autonomous precision landing ground system
* DESCRIPTION: RX and TX of modified frames using the DSMX protocol
*
*****************************************************************************/

/***************************** Include files *******************************/
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>

#include <time.h>       /* time */
#include <sys/time.h>   /* time */

#include <wiringSerial.h>

/*****************************   Namespaces  *******************************/
using namespace std;

/*****************************   Structs   *******************************/
typedef struct package{
  int channel_value[16] = { };
  int byte_H[8];
  int byte_L[8];
} package;

/*****************************    Defines    *******************************/
#define BYTES_IN_FRAME      16
#define BAUD_RATE           115200
#define HIGH                1
#define LOW                 0
#define SYNC_TOLERANCE      5
#define SAFE_ZONE_THRESHOLD 450 // Approximate number of packets ins 10 seconds (packets come with a frequency of ~45,5Hz)
#define RESET_SYNC_THRESHOLD 700
#define FATAL_SYNC_THRESHOLD 1000
#define CHANNEL0_DEFAULT    334
#define CHANNEL1_DEFAULT    1190
#define CHANNEL2_DEFAULT    1114
#define CHANNEL3_DEFAULT    1022
#define CHANNEL4_DEFAULT    1704
#define CHANNEL5_DEFAULT    340
#define CHANNEL6_DEFAULT    0
#define CHANNEL_MAXVALUE    1700

#define safe_mode_threshold 10 /* Amount of seconds before change from UNSAFE to SAFE mode */

#define DSM_S_IDLE          0 /* Used while there is no bytes to recieve and changes values if needed */
#define DSM_S_SAFE          1 /* Used if it has not made any errors in UNSAFE mode for safe_mode_threshold seconds */
#define DSM_S_UNSAFE        2 /* Used when it is not certain that the frames are synced correctly */

/*****************************   Variables   *******************************/
int DSM_STATE = DSM_S_UNSAFE;
bool safe_mode = false; // Used when going from IDLE mode to either UNSAFE or SAFE
bool fatal_error = false;
bool modify_packets = true;
bool packet_modified = true;

bool PREAMBLE = true;
bool BYTE_TYPE = HIGH;
bool in_sync = true;
int byte_in;
int old_byte_in;
int byte_counter = 0;
int last_sync_dist = 0;
int UNSAFE_counter = 0;

int sync_value;
int sync_value_expected = 0;
int sync_value_expected_next = sync_value_expected + 45;
int safe_zone_syncs = 0;

long long time_byte = 0;
long long time_last_byte = 0;
const int frame_timeout = 5000; // micro seconds 1ms=1000us

int UNSAFE_syncs;
int avail_bytes = 0; // 0 since no avaliable bytes when starting up

/*****************************   Methods / functions   *******************************/
long long currentTimeUs()
{
    timeval current;
    gettimeofday(&current, 0);
    return (long long)current.tv_sec * 1000000L + current.tv_usec;
}

void set_channel_value(package &p,int channel, int value)
/*****************************************************************************
*   Input    : Package, channel number, and value
*   Output   : Changes the package channel content
*   Function : Sets the value of a channel
******************************************************************************/
{
   p.channel_value[channel] = value;
   p.byte_H[channel+1] = ((channel<<3)|((value & 0x700) >> 8));
   p.byte_L[channel+1] = (value & 0xFF);
}

void change_packet_values(package &p_in, package &p_out)
/*****************************************************************************
*   Input    : 2 packages
*   Output   : Changes the package content
*   Function : Changes a packet to another
******************************************************************************/
{
    p_out = p_in;         // Echo package as standard
    set_channel_value(p_out,0,CHANNEL_MAXVALUE);
    set_channel_value(p_out,1,CHANNEL_MAXVALUE);
    set_channel_value(p_out,2,CHANNEL_MAXVALUE);
    set_channel_value(p_out,3,CHANNEL_MAXVALUE);
    set_channel_value(p_out,4,CHANNEL_MAXVALUE);
    set_channel_value(p_out,5,CHANNEL_MAXVALUE);
    set_channel_value(p_out,6,CHANNEL_MAXVALUE);
    // set_channel_value(p_out,1,p_in.channel_value[1]);
    // set_channel_value(p_out,2,p_in.channel_value[2]);
    // set_channel_value(p_out,3,p_in.channel_value[3]);
    // set_channel_value(p_out,4,p_in.channel_value[4]);
    // set_channel_value(p_out,5,p_in.channel_value[5]);
    // set_channel_value(p_out,6,p_in.channel_value[6]);
}

int main(int argc,char* argv[])
{
    printf("Program started\nTrying to open serial device...\n");

    int ser_handle; // The serial connection (file descriptor)
    if ((ser_handle = serialOpen("/dev/ttyAMA0", BAUD_RATE)) < 0)
    {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        return 1;
    }
    else
        printf("Serial device has been opened successfully\n");

    package package_in, package_out;
    package_out.channel_value[0] = CHANNEL0_DEFAULT;
    package_out.channel_value[1] = CHANNEL1_DEFAULT;
    package_out.channel_value[2] = CHANNEL2_DEFAULT;
    package_out.channel_value[3] = CHANNEL3_DEFAULT;
    package_out.channel_value[4] = CHANNEL4_DEFAULT;
    package_out.channel_value[5] = CHANNEL5_DEFAULT;
    package_out.channel_value[6] = CHANNEL6_DEFAULT;

    DSM_STATE = DSM_S_IDLE; // Ensure startup in UNSAFE mode
    if (DSM_STATE = DSM_S_IDLE && safe_mode == false)
        printf("Starting RX and TX in IDLE mode (safe_mode is false)\n");
    while(!fatal_error) {
        switch (DSM_STATE) {
            case DSM_S_IDLE: // *** IDLE mode ***
                if (modify_packets and !packet_modified) {
                    change_packet_values(package_in, package_out);
                    packet_modified = true; // Do it once per packet
                } else if (!modify_packets)
                    package_out = package_in;

                if(avail_bytes = serialDataAvail(ser_handle))
                {
                    if (safe_mode)
                        DSM_STATE = DSM_S_SAFE;
                    else
                        DSM_STATE = DSM_S_UNSAFE;
                }
                break;
            case DSM_S_SAFE: // *** SAFE mode ***
                if(avail_bytes = serialDataAvail(ser_handle))
                {
                    // RX byte
                    old_byte_in = byte_in;
                    time_last_byte = time_byte;
                    time_byte = currentTimeUs();
                    byte_in = serialGetchar(ser_handle); //RX byte
                    byte_counter++;

                    // TX prevoius frame
                    switch (BYTE_TYPE) {
                        case  HIGH:
                            serialPutchar(ser_handle,package_out.byte_H[ (byte_counter / 2) ]);
                            break;
                        case  LOW:
                            serialPutchar(ser_handle,package_out.byte_L[ (byte_counter / 2) -1]);
                            break;
                        default:
                            printf("BYTE_TYPE bool has unrecognizable value\n");
                            fatal_error = true;
                            break;
                    }

                    // Interpret RX byte
                    switch (PREAMBLE) {
                        case true: // *** PREAMBLE = true ***
                            switch (BYTE_TYPE) {
                                case  HIGH: // *** BYTE_TYPE = HIGH ***
                                    package_in.byte_H[0] = byte_in;
                                    BYTE_TYPE = LOW;
                                    break; // Break for BYTE_TYPE HIGH
                                case  LOW: // *** BYTE_TYPE = LOW ***
                                    package_in.byte_L[0] = byte_in;
                                    BYTE_TYPE = HIGH;
                                    sync_value = (256*old_byte_in)+byte_in;
                                    //printf("******* Preamble ********* Sync_val: %i\n",sync_value);

                                    if(sync_value == sync_value_expected
                                        || (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value)
                                        && (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))))
                                    {
                                       sync_value_expected = sync_value;
                                       sync_value_expected_next = sync_value + 45;
                                    }
                                    else
                                    {
                                        printf("Switching to UNSAFE mode due to bad sync\n");
                                        safe_zone_syncs = 0;
                                        last_sync_dist = 0;
                                        safe_mode = false;
                                        UNSAFE_counter = 0;
                                        DSM_STATE = DSM_S_UNSAFE;
                                    }

                                    PREAMBLE = false;
                                    break; // Break for BYTE_TYPE LOW
                                default: // *** BYTE_TYPE = not known ***
                                    printf("BYTE_TYPE bool has unrecognizable value\n");
                                    fatal_error = true; // This is really bad!
                                    break; // Break for BYTE_TYPE not known
                            }
                            break; // Break for PREAMBLE true
                        case false:  // *** PREAMBLE = false ***
                            switch (BYTE_TYPE) {
                                case  HIGH: // *** BYTE_TYPE = HIGH ***
                                    package_in.byte_H[ (byte_counter / 2) ] = byte_in;
                                    BYTE_TYPE = LOW;
                                    break; // Break for BYTE_TYPE HIGH
                                case  LOW: // *** BYTE_TYPE = LOW ***
                                    package_in.byte_L[ (byte_counter / 2) - 1 ] = byte_in;
                                    BYTE_TYPE = HIGH;
                                    if (byte_counter >= BYTES_IN_FRAME) // If end of frame note this
                                    {
                                        DSM_STATE = DSM_S_IDLE; // Complete packet has been recieved so go into idle
                                        packet_modified = false; // Indicate for IDLE mode that is has to modify the packet if nessecary
                                        safe_mode = true; // Indicate that it has to return to SAFE mode from IDLE mode when a byte is received
                                        PREAMBLE = true; // Next time a byte is received the first part is the preamble
                                        byte_counter = 0; // Next time a byte is received this is the 1 byte; note the increment at each receive
                                    }
                                    break; // Break for BYTE_TYPE LOW
                                default:
                                    printf("BYTE_TYPE bool has unrecognizable value\n");
                                    fatal_error = true;
                                    break; // Break for BYTE_TYPE not known
                            }
                            break; // Break for PREAMBLE false
                        default:  // *** BYTE_TYPE = not known ***
                            printf("PREAMBLE bool has unrecognizable value\n");
                            fatal_error = true; // This is really bad!
                            break; // Break for BYTE_TYPE not known
                    }
                }
                break; // Break for DSM_STATE SAFE
            case DSM_S_UNSAFE: // *** UNSAFE mode ***
                if(avail_bytes = serialDataAvail(ser_handle))
                {
                    UNSAFE_counter++;
                    old_byte_in = byte_in;
                    time_last_byte = time_byte;
                    time_byte = currentTimeUs();
                    byte_in = serialGetchar(ser_handle); //RX byte
                    serialPutchar(ser_handle,byte_in); //TX byte

                    /*
                    switch (BYTE_TYPE) {
                        case  HIGH: // *** BYTE_TYPE = HIGH ***
                            BYTE_TYPE = LOW;
                            break; // Break for BYTE_TYPE HIGH
                        case  LOW: // *** BYTE_TYPE = LOW ***
                            BYTE_TYPE = HIGH;
                            */
                            sync_value = (256*old_byte_in)+byte_in;
                            if(sync_value == sync_value_expected ||
                                (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value) &&
                                   (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))) &&
                                ((time_byte - time_last_byte) > frame_timeout))
                            {
                                sync_value_expected = sync_value;
                                sync_value_expected_next = sync_value + 45;
                                printf("Last_sync_dist: %i\n",last_sync_dist);
                                if((safe_zone_syncs > 0 && last_sync_dist == 15) || safe_zone_syncs == 0)
                                {
                                    safe_zone_syncs++;
                                    last_sync_dist = 0;
                                    printf("Safe_zone_zyncs: %i Sync_value: %i Sync_value_expected: %i Sync_value_expected_next: %i \n",safe_zone_syncs,sync_value,sync_value_expected,sync_value_expected_next);
                                }
                                else if(safe_zone_syncs>0)
                                    safe_zone_syncs--;
                            }
                            else
                                last_sync_dist++;
                            /*
                            break; // Break for BYTE_TYPE LOW
                        default:
                            cout << "BYTE_TYPE bool has unrecognizable value" << endl;
                            fatal_error = true;
                            break; // Break for BYTE_TYPE not known
                    }
                    */

                    //DETERMINE WHEATHER OR NOT THE THING BETEETH IS VALID
                    if(safe_zone_syncs == SAFE_ZONE_THRESHOLD && last_sync_dist == 14)
                    {
                        byte_counter = 0;
                        PREAMBLE = true;
                        BYTE_TYPE = HIGH;
                        DSM_STATE = DSM_S_SAFE;
                        printf("Exiting unsafe zone\n");
                        break;
                    }

                    if (UNSAFE_counter == RESET_SYNC_THRESHOLD*16)
                    {
                        sync_value_expected = 0;
                        sync_value_expected_next = sync_value + 45;
                    }
                    else if (UNSAFE_counter >= FATAL_SYNC_THRESHOLD*16)
                    {
                        fatal_error = true;
                    }
                }
                break; // Break for DSM_STATE UNSAFE
            default:
                DSM_STATE = DSM_S_UNSAFE;
                break; // Break for DSM_STATE not known
        }
    }

    printf("Encountered a FATAL ERROR - echoing serial bytes until termination\n");
    while (true)
        if(serialDataAvail(ser_handle))
            serialPutchar(ser_handle,serialGetchar(ser_handle));

    return 0; // Should never reach this point but kept although
}

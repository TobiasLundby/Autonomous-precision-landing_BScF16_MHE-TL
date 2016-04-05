/*
 * sync_test.c:
 *   Tests serial connection on RPi
 *   Echoes if not in sync. Interferes if in sync
 *
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>

#include <time.h>       /* time */
#include <sys/time.h>   /* time */

#include <wiringSerial.h>

#define BYTES_IN_FRAME  16
#define BAUD_RATE       115200
#define SYNC_TOLERANCE  5
#define HIGH            true
#define LOW             false

typedef struct package{
  int data[16]; // Sync is not saved.
  int byte_H[16];
  int byte_L[16];
} package;

long long currentTimeUs()
{
    timeval current;
    gettimeofday(&current, 0);
    return (long long)current.tv_sec * 1000000L + current.tv_usec;
}

int main ()
{
  printf("Program started.\n");

  int ser_handle; // the serial connection (file descriptor)

  if ((ser_handle = serialOpen("/dev/ttyAMA0", BAUD_RATE)) < 0)
  {
    fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno)) ;
    return 1 ;
  }
  else
	printf("Serial open\n");

  int byte_in, last_byte_in, last_sync;
  int byte_num = 0;
  int sync_value_expected = 0;
  int sync_value_expected_next = sync_value_expected +45;
  int byte_since_sync = 0;
  int sync_value = 0;
  int transmit_data;
  int expected_byte = 0;

  long long time_byte = 0;
  long long time_last_byte = 0;
  const int frame_timeout = 5;

  bool in_sync = false;
  bool time_to_sync = true;
  bool modify_data = false;
  bool byte_type = HIGH;

  bool test_bool = false;

  package package_in, package_out;

    for(;;)
    {
        if(serialDataAvail(ser_handle))
        {
          last_byte_in = byte_in; // Rotate data
          time_last_byte = time_byte; // Rotate data
          byte_in = serialGetchar(ser_handle); //Recieve the byte on the serial port
          time_byte = currentTimeUs(); // Note time of recieve byte

          if(!in_sync && ((time_byte - time_last_byte) > frame_timeout))
          {
              cout << "here1" << endl;
            if (byte_type == HIGH) {
                package_in.byte_H[0] = byte_in;
                byte_in = LOW;
            } else if (byte_type == LOW) {
                package_in.byte_L[0] = byte_in;

                sync_value = package_in.byte_H[0]*256+byte_in;
                if(sync_value == sync_value_expected || (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value) && (sync_value <(sync_value_expected_next + SYNC_TOLERANCE))))
                {
                  printf("prev: %d cur: %d dist: %d sync_val: %d Expected: %d or %d\n",byte_num-1,byte_num, byte_num-last_sync, sync_value, sync_value_expected, sync_value_expected_next);
                  last_sync =  byte_num;
                  sync_value_expected =  sync_value;
                  sync_value_expected_next = sync_value + 45;
                  in_sync = true;
                  byte_since_sync = 0;
                }
                else
                {
                  printf("sync %d\n",sync_value);
                  in_sync = false; // Is actually already false but you know :D
                }
                byte_in = HIGH;
            }
            // Echo when syncing for safety reasons - EVENTUALLY MAKE A SAFE ZONE
            serialPutchar(ser_handle,byte_in);
          }
          else if(in_sync)
          {
              if (test_bool) {
                 test_bool = false;
                 printf("SYNCED");
              }
              /*
              byte_since_sync = byte_since_sync + 1;
              if (byte_type == HIGH) {
                  byte_H[byte_since_sync] = byte_in;
                  byte_in = LOW;
              } else if (byte_type == LOW) {
                  byte_L[byte_since_sync] = byte_in;
                  int chan_num = (last_byte_in>>3) & 0x0f;
                  int value = ((last_byte_in & 0x07) <<8) | byte_in;

                  byte_in = HIGH;
              }
            // transfer directly
            //serialPutchar(ser_handle,byte_in);

            printf("chan: %d val: %d\n",chan_num,value);

            switch(chan_num){
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                  if(high_byte)
                    package_in.byte_H[chan_num] = byte_in;
                  else
                    package_in.byte_L[chan_num] = byte_in;
                  break;
                default:
                  time_to_sync = true;
                  break;
            }

            if(modify_data)
            {
                // modify the data
            }
            switch(chan_num){
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                    if(high_byte)
                        transmit_data = package_in.byte_H[chan_num];
                    else
                        transmit_data = package_in.byte_L[chan_num];
                    break;
                default:
                    break;
            }

            serialPutchar(ser_handle,transmit_data);
            byte_since_sync = byte_since_sync + 1;
            if(byte_since_sync == 15)
                time_to_sync = true;
            */
            } else {
                //THIS SHOUDL BE MODIFIED
                printf("Not in sync\n");
            }
        byte_num = byte_num + 1;
        }
    }
}

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

#include <wiringSerial.h>

#define BYTES_IN_FRAME  16
#define BAUD_RATE       115200
#define SYNC_TOLERANCE  5

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

  int last_byte;
  int current_byte;
  int sync_val_expected = 0x0000;
  int byte_count = 0;
  int bytes_since_sync = 0;
  int avail_bytes;

  bool in_sync = false;
  bool time_to_sync;
  bool modify_signals = false;


  for (;;)
  {
    if((avail_bytes = serialDataAvail(ser_handle)))
    {
      last_byte = current_byte;
      current_byte = serialGetchar(ser_handle);

      if(time_to_sync)
      {
        int sync_val = last_byte*255+current_byte;
        if(sync_val_expected - SYNC_TOLERANCE < sync_val < sync_val_expected + SYNC_TOLERANCE)
        {
          in_sync = true;
          bytes_since_sync = 0;
          time_to_sync = false;
          sync_val_expected = sync_val + 45;
        }
        else
          printf("Sync failed. Expected value %d. Received value %d",sync_val_expected,sync_val);

        // echo bytes
        serialPutchar(ser_handle,current_byte);
      }
      else
      {
        if(modify_signals)
        {
          printf("No modify_signals code yet");
          //Files can be modified here
        }
        else
        {
          serialPutchar(ser_handle,current_byte);
        }

        // Housekeeping
        bytes_since_sync = bytes_since_sync + 1;
        if(bytes_since_sync >= BYTES_IN_FRAME - 1)
          time_to_sync = true;
      }

      byte_count = byte_count + 1;
    }
  }
}

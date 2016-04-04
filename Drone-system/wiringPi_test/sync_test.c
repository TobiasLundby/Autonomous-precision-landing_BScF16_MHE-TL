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


  int data, last_data, last_zero; 
  int byte_num = 0;
  int sync_expected = 0;
  int sync_expected_next = sync_expected +45;
  int byte_since_sync = 0;

  bool in_sync = false;
  bool time_to_sync = true;
  bool modify_data = false;
  for(;;)
  {
    if(serialDataAvail(ser_handle))
    {
      last_data = data;
      data = serialGetchar(ser_handle);
      if(time_to_sync)
      {
        int sync = last_data*255+data;
        if(sync == sync_expected || (((sync_expected_next - SYNC_TOLERANCE) < sync) && (sync <(sync_expected_next + SYNC_TOLERANCE))))
        {
          printf("prev: %d cur: %d dist: %d sync_val: %d Expected: %d or %d\n",byte_num-1,byte_num, byte_num-last_zero, sync, sync_expected, sync_expected_next);
          last_zero =  byte_num;
          sync_expected =  sync;
          sync_expected_next = sync + 45;
          in_sync = true;
          byte_since_sync = 0;
          time_to_sync = false;
        }
        else
        {
          printf("sync %d\n",sync);
          in_sync = false;
        }
        // Echo when syncing for safety reasons
        serialPutchar(ser_handle,data);
      }
      else
      {

        if(in_sync && modify_data)
        {
         // modify them
        }
        else
        {
          // transfer directly
          serialPutchar(ser_handle,data);
        }
        byte_since_sync = byte_since_sync + 1;
        if(byte_since_sync == 15)
          time_to_sync = true;
      }
      byte_num = byte_num + 1;
    }
  }

/*
  int last_byte = 0;
  int current_byte;
  int sync_val_expected = 0;
  int sync_val_expected_next = sync_val_expected;
  int byte_count = 0;
  int bytes_since_sync = 0;
  int avail_bytes;

  bool in_sync = false;
  bool time_to_sync = true;
  bool modify_signals = false;

  int i = 0;
  for(i;i<100;i++)
  {
    if(serialDataAvail(ser_handle))
      serialPutchar(ser_handle,serialGetchar(ser_handle));
  }


  for (;;)
  {
    if((avail_bytes = serialDataAvail(ser_handle)))
    {
      last_byte = current_byte;
      current_byte = serialGetchar(ser_handle);

      if(time_to_sync)
      {
        int sync_val = last_byte*255+current_byte;
        if( sync_val == sync_val_expected || (((sync_val_expected - SYNC_TOLERANCE) <= sync_val) && (sync_val <= (sync_val_expected + SYNC_TOLERANCE))))
        {
          in_sync = true;
          bytes_since_sync = 0;
          time_to_sync = false;
          sync_val_expected = sync_val;
          sync_val_expected_next = sync_val + 45;
          printf("Sync succced with value %d. Expected value %d or %d\n",sync_val,sync_val_expected,sync_val_expected_next);
        }
        else
          printf("Sync failed. Expected value %d or %d. Received value %d\n",sync_val_expected,sync_val_expected_next,sync_val);

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
  } */
}

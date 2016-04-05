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

typedef struct package{
  int data[14];
  int byte_H[14];
  int byte_L[14];
/*  int chan_0_H;
  int chan_0_L;
  int chan_1_H;
  int chan_1_L;
  int chan_2_H;
  int chan_2_L;
  int chan_3_H;
  int chan_3_L;
  int chan_4_H;
  int chan_4_L;
  int chan_5_H;
  int chan_5_L;
  int chan_6_H;
  int chan_6_L; */
} package;

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

  package package_in, package_out;
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

        bool high_byte = false;
        int transmit_data;
        if((byte_num % 2)==0)
          high_byte = true;

        if(in_sync)
        {
          // transfer directly
          //serialPutchar(ser_handle,data);
	  int chan_num = (last_data>>3) & 0x0f;
          int value = ((last_data & 0x07) <<8) | data;
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
                package_in.byte_H[chan_num] = data;
              else
                package_in.byte_L[chan_num] = data;
              break;
            default:
              time_to_sync = true;
              break;
          }
/*
          switch(chan_num)
          {
	    case 0:
              if(high_byte)
                package_in.chan_0_H = value;
              else
                package_in.chan_0_L = value;
              break;
            case 1:
              if(high_byte)
                package_in.chan_1_H = value;
              else
                package_in.chan_1_L = value;
              break;
            case 2:
              if(high_byte)
                package_in.chan_2_H = value;
              else
                package_in.chan_2_L = value;
              break;
            case 3:
              if(high_byte)
                package_in.chan_3_H = value;
              else
                package_in.chan_3_L = value;
              break;
            case 4:
              if(high_byte)
                package_in.chan_4_H = value;
              else
                package_in.chan_4_L = value;
              break;
            case 5:
              if(high_byte)
                package_in.chan_5_H = value;
              else
                package_in.chan_5_L = value;
              break;
            case 6:
              if(high_byte)
                package_in.chan_6_H = value;
              else
                package_in.chan_6_L = value;
              break;
            default:
              time_to_sync = true;
              break;
          }
*/
          if(modify_data)
          {
            // modify the data
          } 

        

        // transmit

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

  /*      switch(chan_num)
        {
          case 0:
            if(high_byte)
              transmit_data = package_out.chan_0_H;
            else
              transmit_data = package_out.chan_0_L;
            break;
          case 1:
            if(high_byte)
              transmit_data = package_out.chan_1_H;
            else
              transmit_data = package_out.chan_1_L;
            break;
          case 2:
            if(high_byte)
              transmit_data = package_out.chan_2_H;
            else
              transmit_data = package_out.chan_2_L;
          case 3:
            if(high_byte)
              transmit_data = package_out.chan_3_H;
            else
              transmit_data = package_out.chan_3_L;
            break;
          case 4:
            if(high_byte)
              transmit_data = package_out.chan_4_H;
            else
              transmit_data = package_out.chan_4_L;
            break;
          case 5:
            if(high_byte)
              transmit_data = package_out.chan_5_H;
            else
              transmit_data = package_out.chan_5_L;
            break;
          case 6:
            if(high_byte)
              transmit_data = package_out.chan_6_H;
            else
              transmit_data = package_out.chan_6_L;
            break;
          default:
            time_to_sync = true;
            break;
          }
*/
        }
        else  // Not in sync
          // Echo for safety reasons
          transmit_data = data;
         printf("Not in sync\n");

        serialPutchar(ser_handle,transmit_data);
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

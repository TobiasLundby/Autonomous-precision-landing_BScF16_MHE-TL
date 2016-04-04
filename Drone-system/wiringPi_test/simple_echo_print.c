/*
 * test.c:
 *   Tests serial connection on RPi
 *   Receive frame of 16 bytes. Print the frame and transmit it back when complete
 *
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>

#include <wiringSerial.h>

#define BYTES_IN_FRAME  16
#define BAUD_RATE       115200
#define HIGH            0
#define LOW             1

int main ()
{
    printf("Program started.\n");

    int ser_handle; // the serial connection (file descriptor)

    if ((ser_handle = serialOpen("/dev/ttyAMA0", BAUD_RATE)) < 0)
    {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        return 1 ;
    }
    else
        printf("Serial open\n");

    bool preamble = true;
    bool high_byte = true;
    int byte;
    int old_byte;
    int counter = 0;

    int avail_bytes;
    for(;;) {
        if(avail_bytes = serialDataAvail(ser_handle))
        {
          if(preamble)
          {
            if(high_byte)
              high_byte = false;
            else
            {
              preamble = false;
              high_byte = true;
            }
          }
          else
          {
            old_byte = byte;
            byte = serialGetchar(ser_handle);
            serialPutchar(ser_handle,byte);
            if(!high_byte)
            {
              int chan_num = (old_byte>>3) & 0x0f;
              int value = ((old_byte & 0x02)<<8) | byte;
              printf("%d\t%d\n",chan_num,value);
              high_byte = true;
            }
	    else
              high_byte = false;
          }
      
            counter = counter + 1;
/*            if(counter%100 == 0) {
                printf("Byte %i has been passed on\n", counter);
            
              } */
        }
    }
}

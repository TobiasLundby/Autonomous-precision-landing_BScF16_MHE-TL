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

    int counter = 0;

    for(;;) {
        if(avail_bytes = serialDataAvail(ser_handle))
        {
            serialPutchar(ser_handle,serialGetchar(ser_handle));
            counter = counter + 1;
            if(counter%100 == 0) {
                printf("Byte %i has been passed on\n", counter);
            }
        }
    }
}

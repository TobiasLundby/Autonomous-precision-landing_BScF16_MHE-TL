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

#define BYTES_IN_FRAME 16

int main ()
{
  printf("Program started.\n");
//  int test_int = 1; // test of shifting
//  test_int = test_int << 2;
//  printf("%d\n",test_int);

  int ser_handle; // the serial connection (file descriptor)

  if ((ser_handle = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }
  else
	printf("Serial open\n");

// Loop, getting and printing characters

   int package[16];
   int avail_bytes;
   bool package_complete = false;
   bool package_printed = true;
   int index = 0;

  for (;;)
  {
     if(avail_bytes = serialDataAvail(ser_handle))
     {
//	printf("%d\n",index); // print byte num
        package[index] = serialGetchar(ser_handle); // store byte
        index = index + 1;
        if(index ==  BYTES_IN_FRAME) // frame complete?
        {
           index = 0;
           package_complete = true;
           package_printed = false;
        }
     }

     if(package_complete && !package_printed)  // complete and not printed?
     {
        printf("Package complete:\n");
        int i = 0;
        for(i; i <BYTES_IN_FRAME ;i++)
        {
	   // print frame
           //printf("%i",package[i]); // print as integer
           //printf("%c ",package[i]); // print as character
           printf("%X ",package[i]); //  print as hex

           // echo the frame
           serialPutchar(ser_handle,package[i]);
        }
        printf("\n"); 
        package_printed = true;
     }

  }
}

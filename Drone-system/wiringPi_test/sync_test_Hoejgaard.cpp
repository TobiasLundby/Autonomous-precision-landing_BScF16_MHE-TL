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
#define HIGH            true
#define LOW             false
#define SYNC_TOLERANCE  5
#define SAFE_ZONE_THRESHOLD 450 // Approximate number of packets ins 10 seconds
#define CHANNEL0_DEFAULT 334
#define CHANNEL1_DEFAULT 1190
#define CHANNEL2_DEFAULT 1114
#define CHANNEL3_DEFAULT 1022
#define CHANNEL4_DEFAULT 1704
#define CHANNEL5_DEFAULT 340
#define CHANNEL6_DEFAULT 0
#define CHANNEL_MAXVALUE 1700

typedef struct package{
  int channel_value[16] = { };
  int byte_H[8];
  int byte_L[8];
} package;


void set_channel_value(package &p,int channel, int value)
{
   p.channel_value[channel] = value;
   p.byte_H[channel+1] = ((channel<<3)|(value & 0x700));
   p.byte_L[channel+1] = (value & 0xFF);
}

void change_values(package &p_in, package &p_out)
{
    p_in = p_out;         // Echo package as standard
    set_channel_value(p_out,0,CHANNEL_MAXVALUE);
    set_channel_value(p_out,1,p_in.channel_value[1]);
    set_channel_value(p_out,2,p_in.channel_value[2]);
    set_channel_value(p_out,3,p_in.channel_value[3]);
    set_channel_value(p_out,4,p_in.channel_value[4]);
    set_channel_value(p_out,5,p_in.channel_value[5]);
    set_channel_value(p_out,6,p_in.channel_value[6]);

}


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

    bool modify = false;
    bool preamble = true;
    bool byte_type = true;
    bool in_sync = true;
    int byte;
    int old_byte;
    int counter = 0;

    int sync_value;
    int sync_value_expected = 0;
    int sync_value_expected_next = sync_value_expected + 45;

    int safe_zone_syncs;
    package package_in, package_out;
    package_out.channel_value[0] = CHANNEL0_DEFAULT;
    package_out.channel_value[1] = CHANNEL1_DEFAULT;
    package_out.channel_value[2] = CHANNEL2_DEFAULT;
    package_out.channel_value[3] = CHANNEL3_DEFAULT;
    package_out.channel_value[4] = CHANNEL4_DEFAULT;
    package_out.channel_value[5] = CHANNEL5_DEFAULT;
    package_out.channel_value[6] = CHANNEL6_DEFAULT;

    int avail_bytes;
    for(;;) {
        if(avail_bytes = serialDataAvail(ser_handle))
        {
          counter++;
          old_byte = byte;
          byte = serialGetchar(ser_handle);
//          serialPutchar(ser_handle,byte);

          if(byte_type == HIGH)
            serialPutchar(ser_handle,package_out.byte_H[ (counter / 2) ]);
          else
            serialPutchar(ser_handle,package_out.byte_L[ (counter / 2) -1]);
          if(preamble)
          {
            if(byte_type == HIGH)
            {
              package_in.byte_H[0] = byte;
              byte_type = LOW;
            }
            else
            {
              preamble = false;
              package_in.byte_L[0] = byte;
              byte_type = HIGH;
              int sync_value = (256*old_byte)+byte;
              printf("******* Preamble ********* Sync_val: %i\n",sync_value);

              if(sync_value == sync_value_expected || (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value) && (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))))
              {
                 sync_value_expected = sync_value;
                 sync_value_expected_next = sync_value + 45;
                 in_sync = true; 
              }
              else
                 in_sync = false;
            }
          }
          else
          {
            if(byte_type == HIGH )
            {
                package_in.byte_H[ (counter / 2) ] = byte;
                byte_type = LOW;
            }
            else
            {
                package_in.byte_L[ (counter / 2) - 1 ] = byte;

                int chan_num = (old_byte>>3) & 0x0f;
                int value = ((old_byte & 0x07)<<8) | byte;
                printf("%i\t%i\t%i\n",chan_num,value,package_out.channel_value[chan_num]);
                package_in.channel_value[ chan_num ] = value;

                byte_type = HIGH;

                if (counter == BYTES_IN_FRAME)
                {
                    preamble = true;
                    counter = 0;
                }
            }

            if(package_in.channel_value[4]>1600)
               ;
               // change_values(package_in,package_out);
            else
                package_out = package_in;
          }
        }
        if(!in_sync)
        {
            printf("Safe zone - echoing frames\n");
            safe_zone_syncs = 0;
            sync_value_expected_next = 45;
            int last_sync_dist = 0;
            while(true)
                if(serialDataAvail(ser_handle))
                {
                    old_byte = byte;
                    byte = serialGetchar(ser_handle);
                    serialPutchar(ser_handle,byte);

                    if(byte_type == HIGH)
                    {
                        byte_type = LOW;
                    }
                    else
                    {
                        byte_type = HIGH;
                        int sync_value = (256*old_byte)+byte;
                        if(sync_value == sync_value_expected || (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value) && (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))))
                        {
                            sync_value_expected = sync_value;
                            sync_value_expected_next = sync_value + 45;
                            printf("Last_sync_dist: %i\n",last_sync_dist);
                            if((safe_zone_syncs > 0 && last_sync_dist == 14) || safe_zone_syncs == 0)
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
                    }

                    if(safe_zone_syncs == SAFE_ZONE_THRESHOLD)               // approximate
                    {
                        preamble = false;
                        in_sync = true;
                        printf("Exiting safe zone\n");
                        break;
                    }
                }
        }
    }
}

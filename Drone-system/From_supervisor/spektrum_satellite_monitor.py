#!/usr/bin/env python

# This script is capable of decoding a Spektrum DSMX satellite receiver
# Inspired by http://diydrones.com/profiles/blog/show?id=705844%3ABlogPost%3A64228

# After satellite powerup first byte is 0, then it increases after TX interruptions
# to 45, 90, 135, 180, 225, 279, 324, 369 (it seems to loose 1 at 0xFF rollower)

# Code is written very fast. I'm not proud, but it works ;-)
# 2016-03-04 Kjeld Jensen kjen@mmmi.sdu.dk

# parameters
device = '/dev/ttyUSB0'
baudrate = 115200
bit_11 = True

# imports
import serial
import time

# state variables
pause = True
even = False
highbyte = False
preamble = True

# ANSI goto somewhere on the terminal
def ansi_goto (x,y):
	print '\033[%d;%dH' % (y, x)

# ANSI clear the terminal
def ansi_clear ():
	ansi_goto (1,1);
	print '\033[2J'


ser_error = False

time.sleep(30)


try :
	ser = serial.Serial(device, baudrate, timeout=0.002)
except Exception as e:
	print 'unable to open serial device'
	ser_error = True

if ser_error == False:
	ansi_clear()
	ansi_goto (1,1)
	print 'First frame'
	while True:
		c = ser.read()

		if len(c) > 0: # number of elements in c
			pause = False
			if highbyte == False:
				high = ord(c) # convert c to ascii
				highbyte = True
			else:
				low = ord(c) # convert c to ascii
				if preamble  == True:
					print (high*265+low), '                  ' # spaces clears previous values 
					preamble = False
				else:
					if bit_11 == True:
						addr = (high>>3) & 0x0f # inspired by dsm.c line 880 to 890
						val = ((high & 0x07)<<8) | low
					else:
						addr = (high>>2) & 0x0f
						val = ((high & 0x02)<<8) | low

					if addr == 0:
						if val > 1000:
							print addr, val, 'High throttle '
						else:
							print addr, val, 'Low throttle  '
					else:					
						print addr, val, '              '
		
				highbyte = False
		else:
			# nasty code below, it detects end of frames
			# by serial timeout and the timing is not accurate
			# DX6 has a single frame sent each 22ms
			# DX9 has two 'half' frames alternately sent each 11ms (it says 22ms on the display at bind)
			if pause == False:
				highbyte = False
				pause = True
				preamble = True
				if even == True:
					even = False
					ansi_goto (1,1)
					print 'First frame'
				else:
					even = True
					print '            \nSecond frame' # spaces clears screen errors when timing fails




#!/usr/bin/env python
#################################################################
# extremely quick & dirty test:
# confirm ASCII Trance Vibrator hidden LEDs are independently controlled via USB packet

# Yep! The lowest 3 bits of wIndex (values 0x00 ~ 0x07) control LED1~LED3.

import usb
import time
import sys

hDev = usb.core.find(idVendor = 0x0b49, idProduct = 0x064f)
if hDev is None:
    print("Connection to USB device failed! Is it plugged in?")
    sys.exit()

time.sleep(0.2) # wave a dead chicken for 'access violation reading 0x00000018'

#self._connection.reset()

# For some reason, we have to set config, THEN reset,
# otherwise we segfault back in the ctypes (on linux, at
# least). 
hDev.set_configuration()

# HACK: Under Windows 'reset' immediately following set_configuration causes a timeout in ctrl_transfer and is immediately followed by a
#   "WindowsError: exception: access violation reading 0x00000018"
# In fact it seems that any use of 'reset' at all under Windows causes this behavior, even if it is the very first task.

time.sleep(0.2) # wave a dead chicken for 'access violation reading 0x00000018'

led_value = 0

while True:
    print( "LED value: %d" % led_value)
    try:
        #bmRequestType, bRequest, wValue, wIndex, dataphase
        hDev.ctrl_transfer(0x41, 0, 0, 0x0300 | (led_value & 0x07), None)
    except usb.USBError:
        print ("INFO: USB write timed out (probably)")
    time.sleep(1)
    led_value = led_value + 1

  

hDev = None # This seems to be the only official way of 'closing' the device...


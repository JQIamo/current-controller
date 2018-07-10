#!/bin/sh
avrdude -c buspirate -P /dev/ttyUSB0 -p m328p -v
avrdude -c buspirate -P /dev/ttyUSB0 -p m328p -U lfuse:w:0xE2:m
avrdude -c buspirate -P /dev/ttyUSB0 -p m328p -U hfuse:w:0xDE:m
avrdude -c buspirate -P /dev/ttyUSB0 -p m328p -U efuse:w:0x05:m
avrdude -c buspirate -P /dev/ttyUSB0 -p m328p -U flash:w:bin/CurrentController.ino.hex

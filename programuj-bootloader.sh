#!/bin/bash
date
ls -la  term-big-eth.ino_atmega1284_16000000L.hex
avr-objcopy -I ihex term-big-eth.ino_atmega1284_16000000L.hex -O binary term-big-eth.ino_atmega1284_16000000L.bin
~/elektronika/saric-bootloader/software/main -f term-big-eth.ino_atmega1284_16000000L.bin -d $1
#sudo avrdude -c usbasp -p m1284 -V  -U flash:w:term-big-eth.ino_atmega1284_16000000L.hex:i
rm  term-big-eth.ino_atmega1284_16000000L.hex term-big-eth.ino_atmega1284_16000000L.bin

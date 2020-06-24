#!/bin/bash
date
ls -la  term-big-eth.ino_atmega1284_16000000L.hex
sudo avrdude -c usbasp -p m1284 -V  -U flash:w:term-big-eth.ino_atmega1284_16000000L.hex:i
rm  term-big-eth.ino_atmega1284_16000000L.hex

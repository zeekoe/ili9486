#!/bin/sh

rm -f *.o *.so

gcc -O2 -fPIC -c ssd1322.c
gcc -shared -o ssd1322.so ssd1322.o -lwiringPi


sudo cp ssd1322.so /usr/local/lib/

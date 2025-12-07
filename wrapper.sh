#!/bin/sh

rm -f *.o *.so

gcc -O2 -fPIC -c ili9486.c 
gcc -shared -o ili9486.so ili9486.o -lwiringPi


sudo cp ili9486.so /usr/local/lib/

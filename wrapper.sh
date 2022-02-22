#!/bin/sh

rm -f *.o *.so

# Compile ili9486 as shared library
gcc -c -Wall -O2 -fPIC ili9486.c

# Link objects
ld -shared ili9486.o -o ili9486.so

sudo cp ili9486.so /usr/local/lib/
#!/bin/sh

rm -f *.o *.so

# Compile armbian as shared library
gcc -c -Wall -O2 -fPIC ili9486.c

# Link objects
ld -shared ili9486.o -o ili9486.so

# Compile JNA interface
javac -cp jna-4.5.0.jar Ili9486.java

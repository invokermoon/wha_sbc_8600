#!/bin/bash
make clean
if [ "$1" == "pc" ]; then
    make CC=gcc
else
    make CC=/home/sherlock/envtools/Gcc/arm-2009q1/bin/arm-none-linux-gnueabi-gcc

fi



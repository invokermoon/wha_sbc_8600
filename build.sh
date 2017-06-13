#!/bin/bash
if [ "$1" == "pc" ]; then
    make CC=gcc
else
    make
fi



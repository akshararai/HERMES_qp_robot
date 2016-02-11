#!/bin/sh
echo "unset non-realtim /dev/ttyS0..." 
setserial /dev/ttyS0 uart none
echo "unset non-realtim /dev/ttyS1..." 
setserial /dev/ttyS1 uart none
echo "load rtser0 and rtser1"
modprobe xeno_16550A io=0x3f8,0x2f8 irq=4,3 start_index=0
lsmod | grep xeno
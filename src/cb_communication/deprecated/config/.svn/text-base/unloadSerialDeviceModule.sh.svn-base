#!/bin/sh
echo "unload real-time device rtser0..."
rmmod xeno_16550A.ko
echo "set non real-time device /dev/ttyS0 and /dev/ttyS1..."
setserial /dev/ttyS0 uart 16550A
setserial /dev/ttyS1 uart 16550A

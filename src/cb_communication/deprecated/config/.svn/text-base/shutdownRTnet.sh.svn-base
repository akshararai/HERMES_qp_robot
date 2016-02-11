#!/bin/bash

echo "rtifconfig..."

/usr/local/rtnet/sbin/rtifconfig rteth0 down
/usr/local/rtnet/sbin/rtifconfig rteth1 down
/usr/local/rtnet/sbin/rtifconfig rteth2 down
/usr/local/rtnet/sbin/rtifconfig rteth3 down
/usr/local/rtnet/sbin/rtifconfig rteth4 down

echo "rmmod..."

rmmod rtcap.ko
rmmod rt_e1000.ko

sleep 1s

rmmod rtpacket.ko

rmmod rtcfg.ko
rmmod rtipv4.ko
rmmod rtnet.ko

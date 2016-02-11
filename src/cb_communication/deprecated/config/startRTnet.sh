#!/bin/bash

echo "insmod..."
insmod /usr/local/rtnet/modules/rtnet.ko
sleep 1s;

insmod /usr/local/rtnet/modules/rtipv4.ko

insmod /usr/local/rtnet/modules/rtcfg.ko
insmod /usr/local/rtnet/modules/rtpacket.ko

insmod /usr/local/rtnet/modules/rt_e1000.ko InterruptThrottleRate=0,0,0,0,0,0 RxIntDelay=200,200,200,200,200,200 RxAbsIntDelay=750,750,750,750,750,750 XsumRX=1,1,1,1,1,1 TxIntDelay=0,0,0,0,0,0 TxAbsIntDelay=0,0,0,0,0,0 cards=1,1,1,1,1,1

sleep 1s;

insmod /usr/local/rtnet/modules/rtcap.ko

echo "rtifconfig..."

/usr/local/rtnet/sbin/rtifconfig rteth0 up 192.168.5.100 netmask 255.255.255.0 hw ether 00:04:23:B8:80:A4
/usr/local/rtnet/sbin/rtifconfig rteth1 up 192.168.2.100 netmask 255.255.255.0 hw ether 00:04:23:B8:80:A5
/usr/local/rtnet/sbin/rtifconfig rteth2 up 192.168.3.100 netmask 255.255.255.0 hw ether 00:04:23:B8:80:A6
/usr/local/rtnet/sbin/rtifconfig rteth3 up 192.168.4.100 netmask 255.255.255.0 hw ether 00:04:23:B8:80:A7
/usr/local/rtnet/sbin/rtifconfig rteth4 up 192.168.1.100 netmask 255.255.255.0 hw ether 00:0e:0c:6e:a9:5e

/usr/local/rtnet/sbin/rtifconfig

echo "rtroute..."

# left leg
/usr/local/rtnet/sbin/rtroute add 192.168.2.1 0:0:0:22:34:56 dev rteth1
/usr/local/rtnet/sbin/rtroute add 192.168.2.2 0:0:0:22:34:57 dev rteth1
/usr/local/rtnet/sbin/rtroute add 192.168.2.3 0:0:0:22:34:58 dev rteth1
/usr/local/rtnet/sbin/rtroute add 192.168.2.4 0:0:0:22:34:59 dev rteth1
/usr/local/rtnet/sbin/rtroute add 192.168.2.5 0:0:0:22:34:5A dev rteth1
/usr/local/rtnet/sbin/rtroute add 192.168.2.6 0:0:0:22:34:5B dev rteth1
/usr/local/rtnet/sbin/rtroute add 192.168.2.7 0:0:0:22:34:5C dev rteth1
/usr/local/rtnet/sbin/rtroute add 192.168.2.8 0:0:0:22:34:5D dev rteth1
/usr/local/rtnet/sbin/rtroute add 192.168.2.9 0:0:0:22:34:5E dev rteth1

# right arm
/usr/local/rtnet/sbin/rtroute add 192.168.3.1 0:0:0:32:34:56 dev rteth2
/usr/local/rtnet/sbin/rtroute add 192.168.3.2 0:0:0:32:34:57 dev rteth2
/usr/local/rtnet/sbin/rtroute add 192.168.3.3 0:0:0:32:34:58 dev rteth2
/usr/local/rtnet/sbin/rtroute add 192.168.3.4 0:0:0:32:34:59 dev rteth2
/usr/local/rtnet/sbin/rtroute add 192.168.3.5 0:0:0:32:34:5A dev rteth2
/usr/local/rtnet/sbin/rtroute add 192.168.3.6 0:0:0:32:34:5B dev rteth2
/usr/local/rtnet/sbin/rtroute add 192.168.3.7 0:0:0:32:34:5C dev rteth2
/usr/local/rtnet/sbin/rtroute add 192.168.3.8 0:0:0:32:34:5D dev rteth2

# left arm
/usr/local/rtnet/sbin/rtroute add 192.168.4.1 0:0:0:42:34:56 dev rteth3
/usr/local/rtnet/sbin/rtroute add 192.168.4.2 0:0:0:42:34:57 dev rteth3
/usr/local/rtnet/sbin/rtroute add 192.168.4.3 0:0:0:42:34:58 dev rteth3
/usr/local/rtnet/sbin/rtroute add 192.168.4.4 0:0:0:42:34:59 dev rteth3
/usr/local/rtnet/sbin/rtroute add 192.168.4.5 0:0:0:42:34:5A dev rteth3
/usr/local/rtnet/sbin/rtroute add 192.168.4.6 0:0:0:42:34:5B dev rteth3
/usr/local/rtnet/sbin/rtroute add 192.168.4.7 0:0:0:42:34:5C dev rteth3
/usr/local/rtnet/sbin/rtroute add 192.168.4.8 0:0:0:42:34:5D dev rteth3

# right leg
/usr/local/rtnet/sbin/rtroute add 192.168.5.1 0:0:0:52:34:56 dev rteth0
/usr/local/rtnet/sbin/rtroute add 192.168.5.2 0:0:0:52:34:57 dev rteth0
/usr/local/rtnet/sbin/rtroute add 192.168.5.3 0:0:0:52:34:58 dev rteth0
/usr/local/rtnet/sbin/rtroute add 192.168.5.4 0:0:0:52:34:59 dev rteth0
/usr/local/rtnet/sbin/rtroute add 192.168.5.5 0:0:0:52:34:5A dev rteth0
/usr/local/rtnet/sbin/rtroute add 192.168.5.6 0:0:0:52:34:5B dev rteth0
/usr/local/rtnet/sbin/rtroute add 192.168.5.7 0:0:0:52:34:5C dev rteth0
/usr/local/rtnet/sbin/rtroute add 192.168.5.8 0:0:0:52:34:5D dev rteth0

# head
/usr/local/rtnet/sbin/rtroute add 192.168.1.1 0:0:0:12:34:56 dev rteth4
/usr/local/rtnet/sbin/rtroute add 192.168.1.2 0:0:0:12:34:57 dev rteth4
/usr/local/rtnet/sbin/rtroute add 192.168.1.3 0:0:0:12:34:58 dev rteth4
/usr/local/rtnet/sbin/rtroute add 192.168.1.4 0:0:0:12:34:59 dev rteth4
/usr/local/rtnet/sbin/rtroute add 192.168.1.5 0:0:0:12:34:5A dev rteth4
/usr/local/rtnet/sbin/rtroute add 192.168.1.6 0:0:0:12:34:5B dev rteth4
/usr/local/rtnet/sbin/rtroute add 192.168.1.7 0:0:0:12:34:5C dev rteth4
/usr/local/rtnet/sbin/rtroute add 192.168.1.8 0:0:0:12:34:5D dev rteth4

/usr/local/rtnet/sbin/rtroute

ifconfig rteth0 up
ifconfig rteth1 up
ifconfig rteth2 up
ifconfig rteth3 up
ifconfig rteth4 up

ifconfig
 

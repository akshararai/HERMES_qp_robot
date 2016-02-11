#!/bin/bash

ifdown eth1
ifdown eth2
ifdown eth3
ifdown eth4
ifdown eth5

echo "rmmod..."
rmmod e1000

sleep 1

lsmod | grep e1000


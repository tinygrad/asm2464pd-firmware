#!/bin/bash
# Reset the xHCI controller after it dies from stream errors
# Usage: sudo ./reset_xhci.sh
DEV=0000:43:00.3
echo 1 > /sys/bus/pci/devices/$DEV/remove
sleep 2
echo 1 > /sys/bus/pci/rescan
sleep 1
lsusb | grep -iE "add1|ftdi|0403"

#!/bin/bash

rmmod hcsr_platform_device.ko
rmmod hcsr_platform_driver.ko

echo "Removed earlier driver if any"
echo "Cleared dmesg"

dmesg -c

echo "Insert platform device and platform driver"

insmod hcsr_platform_device.ko MAX_DEVICES=$1
insmod hcsr_platform_driver.ko


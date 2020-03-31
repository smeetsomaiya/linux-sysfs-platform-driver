#!/bin/bash

SYSFS_CLASS_NAME="HCSR"
SYSFS_BASE_PATH="/sys/class/"$SYSFS_CLASS_NAME"/"

TRIGGER="/trigger_pin"
ECHO="/echo_pin"
SAMPLES="/number_samples"
PERIOD="/sampling_period"
ENABLE="/enable"
DISTANCE="/distance"

arg_idx = 0
per = 0
echo "Starting distance measurement via sysfs interface"

for (( i=0;i<$1;i++ ))
do
        DEVICE=$SYSFS_BASE_PATH$(echo "hcsr_$i")
        echo $DEVICE

	##Set pins (IO numbers)
	echo "Input trigger pin"
	read trig
	echo $trig > $DEVICE$TRIGGER
	echo "Input echo pin"
	read echo
        echo $echo > $DEVICE$ECHO

	## Set params
	echo "Input number of samples"
	read num
	echo $num > $DEVICE$SAMPLES

	echo "Input sampling period (ms)"
	read per
	echo $per > $DEVICE$PERIOD

done

echo "Enable all devices"
for (( i=0;i<$1;i++ ))
do
        DEVICE=$SYSFS_BASE_PATH$(echo "hcsr_$i")
	echo 1 > $DEVICE$ENABLE
done

echo "Sleep until distance is collected"
sleep 5

echo "Read distance for all devices"
for (( i=0;i<$1;i++ ))
do
        DEVICE=$SYSFS_BASE_PATH$(echo "hcsr_$i")
	DIST=$(cat $DEVICE$DISTANCE)
	echo $DEVICE $DIST
done

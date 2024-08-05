#!/bin/bash

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo


while true
do

SPORT_ProcessID=`ps aux | grep "Legged_sport" | grep -v "grep" | awk '{print $2}'`

if [ ! -n "$SPORT_ProcessID" ];then
eval echo "[sportMode] start" $toStartlog
./bin/Legged_sport > ./log  &
else
echo "" > /dev/null
fi

sleep 0.2
done

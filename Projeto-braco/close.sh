#!/bin/sh

sudo ip link delete can0

rm *.persist *.log

kill -9 -1

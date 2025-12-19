#!/bin/bash

INTERFACE="can0"
INTERFACE_BAUDRATE="1000000"

BITRATE_CORRECT="False"
DATA_RECEIVED="False"

# ==== Phase 1: Check CAN Interface Status ====

# Check if the interface exists at all
if [ ! -d "/sys/class/net/$INTERFACE" ]; then
	echo "Error: $INTERFACE does not exist."
	exit 1
fi

# 2. Check if the interface is "up"
STATUS=$(cat /sys/class/net/$INTERFACE/operstate)

if [ "$STATUS" = "up" ]; then
	echo "$INTERFACE is UP and running."
else
	echo "$INTERFACE is DOWN."
	exit 1
fi

if ip -details -statistics link show "$INTERFACE" | grep -q "bitrate 1000000"; then
	BITRATE_CORRECT="True"
else
	echo "Error: $INTERFACE bitrate is not set to $INTERFACE_BAUDRATE."
fi

# if ! ip -details -statistics link show "$INTERFACE" | grep -q "RX: bytes 0"; then
# 	DATA_RECEIVED="True"
# fi

check_motor_status.py "$INTERFACE" 1 2 3 4

#!/bin/bash

INTERFACE="can0"
INTERFACE_BITRATE="1000000"

# ==== Phase 1: Check CAN Interface Status ====

# Check if the interface exists at all
if [ ! -d "/sys/class/net/$INTERFACE" ]; then
	echo "Error: $INTERFACE does not exist."
	echo "Recommended fix:  Ensure the CAN interface is properly configured and connected."
	exit 1
fi

# 2. Check if the interface is "up"
STATUS=$(cat /sys/class/net/$INTERFACE/operstate)

if [ "$STATUS" = "up" ]; then
	echo "$INTERFACE is UP and running."
else
	echo "$INTERFACE is DOWN."
	echo "Recommended fix:  Reboot the can bus interface or check physical connections.
  To bring the interface up manually, use the command:
  sudo ip link set $INTERFACE DOWN && sudo ip link set $INTERFACE UP"
	exit 1
fi

if ! ip -details -statistics link show "$INTERFACE" | grep -q "bitrate $INTERFACE_BITRATE"; then
	echo "Error: $INTERFACE bitrate is not set to $INTERFACE_BITRATE."
	echo "Recommended fix:  Set the CAN interface bitrate to $INTERFACE_BITRATE using the command:
  sudo ip link set $INTERFACE type can bitrate $INTERFACE_BITRATE"

fi

# if ! ip -details -statistics link show "$INTERFACE" | grep -q "RX: bytes 0"; then
# 	DATA_RECEIVED="True"
# fi

echo "Checking motor status on $INTERFACE..."

scripts/check_motor_status.py " -interface $INTERFACE 1 2 3 4"

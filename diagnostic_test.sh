#!/bin/bash

# Configuration
INTERFACE="can0"
INTERFACE_BITRATE="1000000"

echo "=== CAN Interface Diagnostic Script ==="
echo "Target interface: $INTERFACE"
echo "Expected bitrate: $INTERFACE_BITRATE bit/s"
echo

# ==== Phase 1: Check if CAN Interface Exists ====
if [ ! -d "/sys/class/net/$INTERFACE" ]; then
	echo "ERROR: Interface $INTERFACE does not exist."
	echo
	echo "Recommended fix:"
	echo "  - Ensure the CAN hardware is properly connected."
	echo "  - Configure the interface with 'sudo ip link set $INTERFACE up type can bitrate $INTERFACE_BITRATE'"
	exit 1
fi

echo "$INTERFACE interface found."

# ==== Phase 2: Check if Interface is UP ====
STATUS=$(cat /sys/class/net/$INTERFACE/operstate 2>/dev/null || echo "unknown")

if [ "$STATUS" == "up" ]; then
	echo "$INTERFACE is UP and running."
else
	echo "WARNING: $INTERFACE is currently DOWN (operstate: $STATUS)."
	echo
	echo "Recommended fix:"
	echo "  - Check physical connections and power to the CAN transceiver."
	echo "  - Bring the interface up manually:"
	echo "      sudo ip link set $INTERFACE down"
	echo "      sudo ip link set $INTERFACE up type can bitrate $INTERFACE_BITRATE"
	echo "  - Or run the reboot script if available:"
	exit 1
fi

# ==== Phase 3: Verify Bitrate ====
if ip -details link show "$INTERFACE" | grep -q "bitrate $INTERFACE_BITRATE"; then
	echo "$INTERFACE bitrate is correctly set to $INTERFACE_BITRATE bit/s."
else
	echo "WARNING: Bitrate mismatch detected."
	echo "Current configuration does not show the expected bitrate of $INTERFACE_BITRATE."
	echo
	echo "Recommended fix:"
	echo "  sudo ip link set $INTERFACE down"
	echo "  sudo ip link set $INTERFACE up type can bitrate $INTERFACE_BITRATE"
fi

echo
echo "=== Proceeding to Motor Status Check ==="
echo

scripts/check_motor_status.py --interface $INTERFACE 1 2 3 4

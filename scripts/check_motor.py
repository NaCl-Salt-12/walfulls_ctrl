#!/usr/bin/env python3
"""
Simple test script to check if Cubemars motors are responding on CAN bus.
This script sends an "enter motor control mode" command and listens for responses.
"""

import can
import time
import sys
import argparse
import json

# Configuration
TIMEOUT = 2.0

# MIT Protocol Commands
CMD_ENTER_CONTROL = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
CMD_EXIT_CONTROL = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])

MOTOR_ERROR_CODES = {
    0: "No fault - Motor operating normally",
    1: "Motor over-temperature fault - Motor temperature exceeds safe operating limits",
    2: "Over-current fault - Current draw exceeds maximum allowable threshold",
    3: "Over-voltage fault - Input voltage exceeds maximum operating voltage",
    4: "Under-voltage fault - Input voltage below minimum operating voltage",
    5: "Encoder fault - Position feedback sensor malfunction or disconnection",
    6: "MOSFET over-temperature fault - Power transistor temperature exceeds safe limits",
    7: "Motor lock-up - Motor shaft mechanically blocked or seized",
}


def get_error_message(error_code):
    """
    Get human-readable error message for motor error code.

    Args:
        error_code (int): Error code (0-7)

    Returns:
        str: Error message or unknown error if code is invalid
    """
    return MOTOR_ERROR_CODES.get(error_code, f"Unknown error code: {error_code}")


def parse_motor_response(data):
    """Parse the motor response data"""
    if len(data) != 8:
        return None

    driver_id = data[0]

    # Position (16 bits)
    pos_raw = (data[1] << 8) | data[2]
    position = (pos_raw / 65535.0) * 25.0 - 12.5  # Convert to rad

    # Velocity (12 bits) - depends on motor type, using generic conversion
    vel_raw = (data[3] << 4) | (data[4] >> 4)

    # Current (12 bits)
    curr_raw = ((data[4] & 0x0F) << 8) | data[5]

    # Temperature
    temperature = data[6]

    # Error code
    error = data[7]

    return {
        "driver_id": driver_id,
        "position_raw": pos_raw,
        "position": position,
        "velocity_raw": vel_raw,
        "current_raw": curr_raw,
        "temperature": temperature,
        "error": error,
    }


def test_motor(bus, motor_id, label):
    """Test a single motor by sending enter control command and checking response"""
    print(f"\n{'=' * 60}")
    print(f"Testing: {label} (ID: {motor_id})")
    print(f"{'=' * 60}")

    # Create message to enter control mode
    msg = can.Message(
        arbitration_id=motor_id, data=CMD_ENTER_CONTROL, is_extended_id=False
    )

    try:
        # Send the command
        bus.send(msg)
        print(f"✓ Sent 'Enter Control Mode' command to motor {motor_id}")

        # Wait for response
        start_time = time.time()
        response_received = False

        while (time.time() - start_time) < TIMEOUT:
            rx_msg = bus.recv(timeout=0.5)

            if rx_msg and rx_msg.arbitration_id == motor_id:
                response_received = True
                print(f"✓ {label} RESPONDED!")

                # Parse and display response
                parsed = parse_motor_response(rx_msg.data)
                if parsed:
                    error_msg = get_error_message(parsed["error"])
                    print(f"  - Driver ID: {parsed['driver_id']}")
                    print(f"  - Position: {parsed['position']:.3f} rad")
                    print(f"  - Temperature: {parsed['temperature']}°C")
                    print(f"  - Error Code: {parsed['error']}")
                    print(f"  - Error Message: {error_msg}")

                    if parsed["error"] == 0:
                        print("  - Status: ✓ No errors")
                    else:
                        print("  - Status: ⚠ Error detected!")
                break

        if not response_received:
            print(f"✗ {label} did NOT respond (timeout)")
            return False

        return True

    except can.CanError as e:
        print(f"✗ CAN Error for motor {label}: {e}")
        return False


def main():
    """Main test function"""

    parser = argparse.ArgumentParser(description="Test motors via JSON config.")
    parser.add_argument("config", help="Path to the JSON configuration file")
    args = parser.parse_args()

    try:
        with open(args.config, "r") as f:
            config = json.load(f)

        interface = config.get("interface", "can0")
        bitrate = config.get("bitrate", 1000000)
        nodes = config.get("nodes", [])
    except Exception as e:
        print(f"Error loading JSON: {e}")
        sys.exit(1)

    print(f"CAN Interface: {interface}")
    print(f"Timeout: {TIMEOUT}s")

    try:
        # Initialize CAN bus
        print(f"\nInitializing CAN bus on '{interface}'...")
        bus = can.interface.Bus(bustype="socketcan", channel=interface)
        print("✓ CAN bus initialized successfully")

        # Test each motor
        results = {}
        for node in nodes:
            motor_id = node["id"]
            label = node["label"]
            results[label] = test_motor(bus, motor_id, label)
            time.sleep(0.5)  # Short delay between tests

        # Summary
        print(f"\n{'=' * 60}")
        print("TEST SUMMARY")
        print(f"{'=' * 60}")

        max_length = max(len(node["label"]) for node["label"] in nodes)

        for label, responded in results.items():
            status = "✓ ONLINE" if responded else "✗ OFFLINE"
            print(f"{label:<{max_length}} {status}")

        online_count = sum(results.values())
        total_count = len(results)
        print(f"\nTotal: {online_count}/{total_count} motors responding")

        # Exit control mode for all motors that responded
        print(f"\n{'=' * 60}")
        print("Sending 'Exit Control Mode' to all motors...")
        for node in nodes:
            if results.get(node["id"]):
                exit_msg = can.Message(
                    arbitration_id=node["id"],
                    data=CMD_EXIT_CONTROL,
                    is_extended_id=False,
                )
                bus.send(exit_msg)
                print(f"✓ Exit command sent to motor {node['label']}")

        # Cleanup
        bus.shutdown()
        print("\n✓ Test complete!")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

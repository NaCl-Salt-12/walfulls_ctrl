#!/usr/bin/env python3
"""
Simple test script to check if Cubemars motors are responding on CAN bus.
This script sends an "enter motor control mode" command and listens for responses.
"""

import can
import time
import sys
import argparse

# Configuration
TIMEOUT = 2.0  # Seconds to wait for responses

# MIT Protocol Commands
CMD_ENTER_CONTROL = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
CMD_EXIT_CONTROL = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])


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


def test_motor(bus, motor_id):
    """Test a single motor by sending enter control command and checking response"""
    print(f"\n{'=' * 60}")
    print(f"Testing Motor ID: {motor_id}")
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
                print(f"✓ Motor {motor_id} RESPONDED!")

                # Parse and display response
                parsed = parse_motor_response(rx_msg.data)
                if parsed:
                    print(f"  - Driver ID: {parsed['driver_id']}")
                    print(f"  - Position: {parsed['position']:.3f} rad")
                    print(f"  - Temperature: {parsed['temperature']}°C")
                    print(f"  - Error Code: {parsed['error']}")

                    if parsed["error"] == 0:
                        print("  - Status: ✓ No errors")
                    else:
                        print("  - Status: ⚠ Error detected!")
                break

        if not response_received:
            print(f"✗ Motor {motor_id} did NOT respond (timeout)")
            return False

        return True

    except can.CanError as e:
        print(f"✗ CAN Error for motor {motor_id}: {e}")
        return False


def main():
    """Main test function"""

    parser = argparse.ArgumentParser(
        description="Check Cubemars motor status over CAN bus."
    )

    parser.add_argument(
        "-i", "--interface", type=str, default="can0", help="CAN interface to use"
    )
    parser.add_argument(
        "can_ids", type=int, nargs="+", help="List of motor CAN IDs to test"
    )

    args = parser.parse_args()

    CAN_INTERFACE = args.interface
    MOTOR_IDS = args.can_ids

    print(f"CAN Interface: {CAN_INTERFACE}")
    print(f"Motor IDs to test: {MOTOR_IDS}")
    print(f"Timeout: {TIMEOUT}s")

    try:
        # Initialize CAN bus
        print(f"\nInitializing CAN bus on '{CAN_INTERFACE}'...")
        bus = can.interface.Bus(bustype="socketcan", channel=CAN_INTERFACE)
        print("✓ CAN bus initialized successfully")

        # Test each motor
        results = {}
        for motor_id in MOTOR_IDS:
            results[motor_id] = test_motor(bus, motor_id)
            time.sleep(0.5)  # Short delay between tests

        # Summary
        print(f"\n{'=' * 60}")
        print("TEST SUMMARY")
        print(f"{'=' * 60}")
        for motor_id, responded in results.items():
            status = "✓ ONLINE" if responded else "✗ OFFLINE"
            print(f"Motor {motor_id}: {status}")

        online_count = sum(results.values())
        total_count = len(results)
        print(f"\nTotal: {online_count}/{total_count} motors responding")

        # Exit control mode for all motors that responded
        print(f"\n{'=' * 60}")
        print("Sending 'Exit Control Mode' to all motors...")
        for motor_id in MOTOR_IDS:
            if results[motor_id]:
                exit_msg = can.Message(
                    arbitration_id=motor_id, data=CMD_EXIT_CONTROL, is_extended_id=False
                )
                bus.send(exit_msg)
                print(f"✓ Exit command sent to motor {motor_id}")

        # Cleanup
        bus.shutdown()
        print("\n✓ Test complete!")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Live Test Script for AS5600 Magnetic Encoder

This script provides a real-time monitoring tool for the AS5600 encoder.
It continuously reads the encoder angle and displays it with magnet detection
status in a live feed format.

Hardware Requirements:
    - Raspberry Pi 4
    - AS5600 encoder connected to I2C Bus 1 (GPIO 2=SDA, GPIO 3=SCL)
    - Magnet positioned above the AS5600 sensor

Usage:
    python tests/test_as5600_live.py
    
    Press Ctrl+C to stop the test.
"""

import sys
import time
import os
import subprocess
from pathlib import Path
from typing import Tuple, Optional

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from src.Drivers.as5600 import AS5600, AS5600I2CError
except ImportError:
    try:
        from src.drivers.as5600 import AS5600, AS5600I2CError
    except ImportError:
        print("Error: Could not import AS5600 driver")
        print("Make sure the driver is in src/Drivers/as5600.py or src/drivers/as5600.py")
        sys.exit(1)


def check_i2c_available(bus_number: int = 1) -> Tuple[bool, str]:
    """
    Check if I2C bus is available on the system.
    
    Args:
        bus_number: I2C bus number to check
    
    Returns:
        tuple: (is_available: bool, message: str)
    """
    device_path = f"/dev/i2c-{bus_number}"
    
    # Check if device file exists
    if not os.path.exists(device_path):
        return False, (
            f"I2C device file '{device_path}' not found.\n"
            f"\n"
            f"To enable I2C on Raspberry Pi:\n"
            f"1. Run: sudo raspi-config\n"
            f"2. Navigate to: Interface Options -> I2C -> Enable\n"
            f"3. Reboot the Raspberry Pi\n"
            f"\n"
            f"Alternatively, add this line to /boot/config.txt:\n"
            f"   dtparam=i2c_arm=on\n"
            f"\n"
            f"Then reboot: sudo reboot\n"
        )
    
    # Check if user has permissions to access I2C
    if not os.access(device_path, os.R_OK | os.W_OK):
        return False, (
            f"No permission to access '{device_path}'.\n"
            f"\n"
            f"To fix this, add your user to the i2c group:\n"
            f"   sudo usermod -a -G i2c $USER\n"
            f"   (Then log out and log back in, or reboot)\n"
            f"\n"
            f"Or run this script with sudo (not recommended for security):\n"
            f"   sudo python tests/test_as5600_live.py\n"
        )
    
    return True, "I2C bus available"


def scan_i2c_bus(bus_number: int = 1) -> Tuple[bool, list]:
    """
    Scan I2C bus for devices.
    
    Args:
        bus_number: I2C bus number to scan
    
    Returns:
        tuple: (success: bool, devices: list of addresses found)
    """
    devices = []
    
    try:
        # Try using i2cdetect if available
        result = subprocess.run(
            ['i2cdetect', '-y', str(bus_number)],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            # Parse i2cdetect output
            lines = result.stdout.strip().split('\n')
            for line in lines[1:]:  # Skip header line
                parts = line.split()
                if len(parts) > 0:
                    addr_hex = parts[0]  # First column is address
                    # Parse remaining columns (addresses found)
                    for addr_str in parts[1:]:
                        if addr_str != '--' and addr_str != 'UU':
                            try:
                                addr = int(addr_str, 16)
                                devices.append(addr)
                            except ValueError:
                                pass
            
            return True, devices
        else:
            return False, []
    
    except (subprocess.TimeoutExpired, FileNotFoundError, subprocess.SubprocessError):
        # i2cdetect not available or failed - try manual scan
        try:
            try:
                from smbus2 import SMBus
            except ImportError:
                try:
                    from smbus import SMBus
                except ImportError:
                    return False, []
            
            bus = SMBus(bus_number)
            
            # Scan addresses 0x08 to 0x77 (valid I2C addresses)
            for addr in range(0x08, 0x78):
                try:
                    # Quick read attempt (some devices may not respond)
                    bus.read_byte(addr)
                    devices.append(addr)
                except (IOError, OSError):
                    pass
            
            bus.close()
            return True, devices
        
        except Exception:
            return False, []


def check_as5600_present(bus_number: int = 1, expected_address: int = 0x36) -> Tuple[bool, str]:
    """
    Check if AS5600 is present on I2C bus.
    
    Args:
        bus_number: I2C bus number
        expected_address: Expected I2C address (0x36 for AS5600)
    
    Returns:
        tuple: (is_present: bool, message: str)
    """
    print(f"Scanning I2C Bus {bus_number} for devices...")
    success, devices = scan_i2c_bus(bus_number)
    
    if not success:
        return False, "Could not scan I2C bus. Try installing i2c-tools: sudo apt-get install i2c-tools"
    
    if not devices:
        return False, (
            f"No I2C devices found on bus {bus_number}.\n"
            f"\n"
            f"Troubleshooting:\n"
            f"  1. Verify AS5600 is physically connected:\n"
            f"     - SDA to GPIO 2 (Pin 3)\n"
            f"     - SCL to GPIO 3 (Pin 5)\n"
            f"     - VDD to 3.3V or 5V\n"
            f"     - GND to Ground\n"
            f"  2. Check wiring for loose connections\n"
            f"  3. Verify power supply is connected\n"
            f"  4. Try checking connections with a multimeter\n"
        )
    
    print(f"Found I2C devices at addresses: {[hex(addr) for addr in devices]}")
    
    if expected_address in devices:
        return True, f"AS5600 found at address {hex(expected_address)}"
    
    return False, (
        f"AS5600 not found at expected address {hex(expected_address)}.\n"
        f"\n"
        f"Found devices at: {[hex(addr) for addr in devices]}\n"
        f"\n"
        f"Possible issues:\n"
        f"  1. Wrong device connected\n"
        f"  2. AS5600 may be at different address (check datasheet)\n"
        f"  3. Wiring issue - device not responding properly\n"
        f"  4. Power issue - device not powered correctly\n"
        f"\n"
        f"Expected AS5600 address: 0x36\n"
        f"If you have i2c-tools, try: i2cdetect -y {bus_number}\n"
    )


def main():
    """Main test function for AS5600 encoder."""
    print("AS5600 Encoder Live Test")
    print("=" * 50)
    
    # Check I2C availability before attempting to initialize
    print("Checking I2C Bus 1 availability...")
    i2c_available, i2c_message = check_i2c_available(bus_number=1)
    
    if not i2c_available:
        print("ERROR: I2C Bus 1 is not available")
        print("=" * 50)
        print(i2c_message)
        print("=" * 50)
        
        # Additional diagnostic information
        print("\nDiagnostic Information:")
        print(f"  Checking for I2C device files:")
        for bus_num in [0, 1, 3]:
            dev_path = f"/dev/i2c-{bus_num}"
            exists = os.path.exists(dev_path)
            readable = os.access(dev_path, os.R_OK) if exists else False
            print(f"    {dev_path}: {'EXISTS' if exists else 'NOT FOUND'} "
                  f"{'(readable)' if readable else ''}")
        
        # Check if i2c-tools is installed
        import shutil
        if shutil.which('i2cdetect'):
            print("\n  i2c-tools is installed - you can run:")
            print("    i2cdetect -y 1  # to scan I2C bus 1")
        else:
            print("\n  i2c-tools not installed - install with:")
            print("    sudo apt-get install i2c-tools")
        
        return 1
    
    print("I2C Bus 1 is available.")
    
    # Check if AS5600 is present on the bus
    print("\n" + "=" * 50)
    as5600_present, as5600_message = check_as5600_present(bus_number=1, expected_address=0x36)
    
    if not as5600_present:
        print("ERROR: AS5600 not detected on I2C Bus 1")
        print("=" * 50)
        print(as5600_message)
        print("=" * 50)
        return 1
    
    print(f"✓ {as5600_message}")
    print("=" * 50)
    print("\nInitializing AS5600...")
    print("Press Ctrl+C to stop the test.\n")
    
    # Initialize driver on I2C Bus 1
    encoder = AS5600(i2c_bus=1)
    
    try:
        # Try a direct I2C read test before using the driver
        print("Performing low-level I2C communication test...")
        try:
            from smbus2 import SMBus
        except ImportError:
            try:
                from smbus import SMBus
            except ImportError:
                print("Warning: Cannot import smbus for direct testing")
                SMBus = None
        
        if SMBus:
            test_bus = SMBus(1)
            try:
                # Try reading STATUS register (0x0B) directly
                print("  Attempting direct read of STATUS register (0x0B)...")
                status = test_bus.read_byte_data(0x36, 0x0B)
                print(f"  ✓ Successfully read STATUS register: 0x{status:02X}")
                print(f"  Status bits: MD={bool(status & 0x08)}, ML={bool(status & 0x10)}, MH={bool(status & 0x20)}")
                
                # Try reading RAW ANGLE registers too
                print("  Attempting direct read of RAW ANGLE registers (0x0C, 0x0D)...")
                angle_h = test_bus.read_byte_data(0x36, 0x0C)
                angle_l = test_bus.read_byte_data(0x36, 0x0D)
                raw_angle = ((angle_h & 0x0F) << 8) | angle_l
                angle_deg = (raw_angle / 4096.0) * 360.0
                print(f"  ✓ Successfully read RAW ANGLE: {raw_angle} (0x{raw_angle:03X}) = {angle_deg:.2f}°")
                
            except Exception as e:
                print(f"  ✗ Direct read failed: {e}")
                print(f"  Error type: {type(e).__name__}")
                import traceback
                traceback.print_exc()
            finally:
                test_bus.close()
        
        print("\nAttempting initialization through driver...")
        # Initialize the encoder
        if not encoder.initialize():
            print("\nERROR: Failed to initialize AS5600 encoder")
            print("I2C bus is accessible and device was detected, but initialization failed.")
            print("\nPossible causes:")
            print("  1. Device may be in a bad state - try power cycling")
            print("  2. I2C communication timing issue")
            print("  3. Device may require pull-up resistors (usually 4.7kΩ on SDA/SCL)")
            print("  4. Voltage levels may be incorrect (check VDD)")
            return 1
        
        print("Encoder initialized successfully!")
        print("Starting live angle readout...\n")
        
        # Main reading loop
        while True:
            try:
                # Get status first to check all magnet conditions
                status = encoder.get_status()
                magnet_detected = status.get('magnet_detected', False)
                magnet_too_weak = status.get('magnet_too_weak', False)
                magnet_too_strong = status.get('magnet_too_strong', False)
                
                # Read angle - driver now handles MH case (magnet too strong but readable)
                angle = encoder.get_angle()
                
                # Format the output with detailed status
                if angle is not None:
                    if magnet_detected:
                        status_text = "Magnet Detected ✓"
                    elif magnet_too_strong:
                        status_text = "Magnet Too Strong ⚠"
                    elif magnet_too_weak:
                        status_text = "Magnet Too Weak ⚠"
                    else:
                        status_text = "Magnet Status Unknown"
                    output = f"Current Angle: {angle:.2f}° | Status: {status_text}"
                else:
                    if magnet_too_weak:
                        status_text = "Magnet Too Weak - Move magnet CLOSER"
                    elif magnet_too_strong:
                        status_text = "Magnet Too Strong - Move magnet FARTHER"
                    else:
                        status_text = "Magnet Missing - Check magnet position"
                    error_code = status.get('error_code', 'Unknown error')
                    output = f"Current Angle: N/A | Status: {status_text}"
                
                # Print with carriage return for in-place updates
                # Add spaces at the end to clear any remaining characters from previous line
                print(f"\r{output:<90}", end='', flush=True)
                
                # Sleep for 0.1 seconds before next reading
                time.sleep(0.1)
                
            except Exception as e:
                print(f"\nERROR during reading: {e}")
                time.sleep(0.1)
                continue
    
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print("\n\nTest Stopped.")
        print("Cleaning up resources...")
    
    except AS5600I2CError as e:
        print(f"\nERROR: I2C communication error: {e}")
        print("\nDetailed Troubleshooting:")
        print("  1. Verify AS5600 wiring:")
        print("     - SDA → GPIO 2 (Physical Pin 3)")
        print("     - SCL → GPIO 3 (Physical Pin 5)")
        print("     - VDD → 3.3V or 5V (check AS5600 module requirements)")
        print("     - GND → Ground")
        print("  2. Check for pull-up resistors:")
        print("     - I2C requires pull-up resistors (typically 4.7kΩ)")
        print("     - Many AS5600 modules have these built-in")
        print("  3. Power issues:")
        print("     - Verify power supply is stable")
        print("     - Check voltage levels with multimeter")
        print("  4. Try scanning the bus manually:")
        print("     sudo apt-get install i2c-tools")
        print("     i2cdetect -y 1")
        print("  5. If device shows 'UU' in i2cdetect, it may be in use by kernel driver")
        return 1
    
    except Exception as e:
        print(f"\nERROR: Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        # Cleanup
        try:
            encoder._cleanup()
            print("Cleanup completed.")
        except Exception as e:
            print(f"Warning: Error during cleanup: {e}")
    
    return 0


if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\nTest Stopped.")
        sys.exit(0)


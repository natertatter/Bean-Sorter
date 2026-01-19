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
from pathlib import Path

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


def main():
    """Main test function for AS5600 encoder."""
    print("AS5600 Encoder Live Test")
    print("=" * 50)
    print("Initializing AS5600 on I2C Bus 1...")
    print("Press Ctrl+C to stop the test.\n")
    
    # Initialize driver on I2C Bus 1
    encoder = AS5600(i2c_bus=1)
    
    try:
        # Initialize the encoder
        if not encoder.initialize():
            print("ERROR: Failed to initialize AS5600 encoder")
            print("Check wiring and I2C connection.")
            return 1
        
        print("Encoder initialized successfully!")
        print("Starting live angle readout...\n")
        
        # Main reading loop
        while True:
            try:
                # Read current angle
                angle = encoder.get_angle()
                
                # Get status to check magnet detection
                status = encoder.get_status()
                magnet_detected = status.get('magnet_detected', False)
                
                # Format the output
                if angle is not None:
                    status_text = "Magnet Detected" if magnet_detected else "Magnet Missing"
                    output = f"Current Angle: {angle:.2f}° | Status: {status_text}"
                else:
                    status_text = "Magnet Missing" if not magnet_detected else "Read Error"
                    error_code = status.get('error_code', 'Unknown error')
                    output = f"Current Angle: N/A | Status: {status_text} ({error_code})"
                
                # Print with carriage return for in-place updates
                # Add spaces at the end to clear any remaining characters from previous line
                print(f"\r{output:<80}", end='', flush=True)
                
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


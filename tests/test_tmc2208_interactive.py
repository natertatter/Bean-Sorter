#!/usr/bin/env python3
"""
Interactive TMC2208 Test Program

This script provides an interactive command-line interface for testing
TMC2208 stepper motor driver with UART configuration and GPIO step pulses.

Hardware Requirements:
    - Raspberry Pi 4
    - TMC2208 driver connected via UART (GPIO 14/TXD)
    - STEP pin connected to GPIO 5
    - DIR pin connected to GPIO 6
    - NEMA 17 Stepper Motor (1.8° / 200 steps per revolution)

Usage:
    python tests/test_tmc2208_interactive.py
    
    Press Ctrl+C to safely stop the motor and exit.
"""

import sys
import time
import signal
import os
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from src.Drivers.tmc2208 import TMC2208, TMC2208UARTError
except ImportError:
    try:
        from src.drivers.tmc2208 import TMC2208, TMC2208UARTError
    except ImportError:
        print("Error: Could not import TMC2208 driver")
        print("Make sure the driver is in src/Drivers/tmc2208.py or src/drivers/tmc2208.py")
        sys.exit(1)

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("Warning: RPi.GPIO not available. GPIO step pulses will be simulated.")
    GPIO_AVAILABLE = False
    GPIO = None


# GPIO Pin Definitions
STEP_PIN = 5
DIR_PIN = 6

# Motor Constants
STEPS_PER_REVOLUTION = 200  # NEMA 17: 1.8° per step


class TMC2208InteractiveTest:
    """Interactive test interface for TMC2208 driver."""
    
    def __init__(self):
        """Initialize the test interface."""
        self.motor = None
        self.running = True
        self.current_microsteps = 16  # Default
        self.current_rpm = 60  # Default
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Setup GPIO if available
        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully."""
        print("\n\nInterrupt received. Stopping motor and cleaning up...")
        self.running = False
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Clean up resources."""
        try:
            if self.motor:
                self.motor.stop()
                self.motor._cleanup()
            
            if GPIO_AVAILABLE:
                GPIO.output(STEP_PIN, GPIO.LOW)
                GPIO.output(DIR_PIN, GPIO.LOW)
                GPIO.cleanup()
            
            print("Cleanup completed.")
        except Exception as e:
            print(f"Warning: Error during cleanup: {e}")
    
    def check_uart_available(self, uart_port: str = '/dev/serial0') -> tuple:
        """
        Check if UART port is available.
        
        Args:
            uart_port: UART device path to check
        
        Returns:
            tuple: (is_available: bool, message: str)
        """
        # Check if device file exists
        if not os.path.exists(uart_port):
            # Check for alternative UART devices
            alternatives = ['/dev/ttyAMA0', '/dev/ttyS0', '/dev/ttyAMA1']
            found_alternatives = [alt for alt in alternatives if os.path.exists(alt)]
            
            message = (
                f"UART device '{uart_port}' not found.\n"
                f"\n"
            )
            
            if found_alternatives:
                message += (
                    f"Found alternative UART devices: {found_alternatives}\n"
                    f"\n"
                )
            
            message += (
                f"To enable UART on Raspberry Pi:\n"
                f"1. Run: sudo raspi-config\n"
                f"2. Navigate to: Interface Options -> Serial Port\n"
                f"3. Enable serial port (disable login shell if prompted)\n"
                f"4. Reboot the Raspberry Pi\n"
                f"\n"
                f"Alternatively, add these lines to /boot/config.txt:\n"
                f"   enable_uart=1\n"
                f"   dtoverlay=uart0  # or uart5 for second UART\n"
                f"\n"
                f"Then reboot: sudo reboot\n"
                f"\n"
                f"Note: /dev/serial0 is a symlink to the primary UART.\n"
                f"If it doesn't exist, check /boot/config.txt for UART configuration.\n"
            )
            
            return False, message
        
        # Check if user has permissions
        if not os.access(uart_port, os.R_OK | os.W_OK):
            return False, (
                f"No permission to access '{uart_port}'.\n"
                f"\n"
                f"To fix this, add your user to the dialout group:\n"
                f"   sudo usermod -a -G dialout $USER\n"
                f"   (Then log out and log back in, or reboot)\n"
                f"\n"
                f"Or run this script with sudo (not recommended for security):\n"
                f"   sudo python tests/test_tmc2208_interactive.py\n"
            )
        
        return True, "UART port available"
    
    def initialize_motor(self):
        """Initialize the TMC2208 motor driver."""
        print("Initializing TMC2208 driver...")
        print("UART Port: /dev/serial0 (GPIO 14/TXD)")
        
        # Check UART availability first
        print("\nChecking UART availability...")
        uart_available, uart_message = self.check_uart_available('/dev/serial0')
        
        if not uart_available:
            print("ERROR: UART port is not available")
            print("=" * 60)
            print(uart_message)
            print("=" * 60)
            
            # Additional diagnostics
            print("\nDiagnostic Information:")
            print("  Checking for UART device files:")
            uart_devices = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0', '/dev/ttyAMA1']
            for dev in uart_devices:
                exists = os.path.exists(dev)
                readable = os.access(dev, os.R_OK) if exists else False
                print(f"    {dev}: {'EXISTS' if exists else 'NOT FOUND'} "
                      f"{'(readable)' if readable else ''}")
            
            return False
        
        print("✓ UART port is available")
        
        try:
            self.motor = TMC2208(uart_port='/dev/serial0', baudrate=115200)
            
            if not self.motor.initialize():
                print("ERROR: Failed to initialize TMC2208 driver")
                print("\nTroubleshooting:")
                print("  1. Verify TMC2208 is connected to GPIO 14 (TXD)")
                print("  2. Check wiring: TX via 1kΩ resistor to PDN_UART pin")
                print("  3. Verify power: VIO to 3.3V, VM to motor power supply")
                print("  4. Check UART baudrate matches (115200)")
                return False
            
            print("✓ TMC2208 initialized successfully")
            return True
            
        except Exception as e:
            print(f"ERROR: Initialization failed: {e}")
            print("\nTroubleshooting:")
            print("  1. Verify TMC2208 wiring and connections")
            print("  2. Check that UART is properly enabled in /boot/config.txt")
            print("  3. Ensure TMC2208 is powered and VIO is at 3.3V")
            return False
    
    def get_microstepping_input(self) -> int:
        """Get microstepping resolution from user."""
        valid_values = [1, 2, 4, 8, 16, 32, 64, 128, 256]
        
        while True:
            try:
                print(f"\nMicrostepping Resolution (valid: {valid_values}):")
                user_input = input("Enter microsteps [default: 16]: ").strip()
                
                if not user_input:
                    return 16  # Default
                
                microsteps = int(user_input)
                
                if microsteps in valid_values:
                    return microsteps
                else:
                    print(f"Invalid value. Must be one of: {valid_values}")
            
            except ValueError:
                print("Invalid input. Please enter a number.")
            except KeyboardInterrupt:
                raise
    
    def get_current_input(self) -> tuple:
        """Get current settings from user (optional)."""
        while True:
            try:
                print("\nCurrent Settings (optional - press Enter to skip):")
                user_input = input("Enter run current in mA [default: skip]: ").strip()
                
                if not user_input:
                    return None, None, None
                
                run_current_ma = int(user_input)
                
                # Convert mA to TMC2208 current value (0-31)
                # This is a simplified conversion - adjust based on your motor and VREF
                # Typical: 0-31 maps to 0-100% of VREF current
                # For 1.5A motor with VREF=1.5V: ~50mA per step
                # This is a rough estimate - adjust for your setup
                run_current = min(31, max(0, run_current_ma // 50))
                hold_current = max(0, run_current - 4)  # Hold slightly less than run
                
                return run_current, hold_current, 4
                
            except ValueError:
                print("Invalid input. Please enter a number or press Enter to skip.")
            except KeyboardInterrupt:
                raise
    
    def get_speed_input(self) -> float:
        """Get target RPM from user."""
        while True:
            try:
                print("\nSpeed:")
                user_input = input("Enter target RPM [default: 60]: ").strip()
                
                if not user_input:
                    return 60.0
                
                rpm = float(user_input)
                
                if rpm <= 0:
                    print("RPM must be positive.")
                    continue
                
                return rpm
                
            except ValueError:
                print("Invalid input. Please enter a number.")
            except KeyboardInterrupt:
                raise
    
    def get_steps_input(self) -> int:
        """Get number of steps from user."""
        while True:
            try:
                print("\nSteps:")
                user_input = input("Enter number of steps: ").strip()
                
                steps = int(user_input)
                
                if steps <= 0:
                    print("Steps must be positive.")
                    continue
                
                return steps
                
            except ValueError:
                print("Invalid input. Please enter a number.")
            except KeyboardInterrupt:
                raise
    
    def get_direction_input(self) -> int:
        """Get direction from user."""
        while True:
            try:
                print("\nDirection:")
                user_input = input("Enter direction (CW/CCW) [default: CW]: ").strip().upper()
                
                if not user_input or user_input == 'CW':
                    return 1  # Forward
                elif user_input == 'CCW':
                    return -1  # Reverse
                else:
                    print("Invalid input. Enter 'CW' or 'CCW'.")
            
            except KeyboardInterrupt:
                raise
    
    def calculate_pulse_delay(self, rpm: float, microsteps: int) -> float:
        """
        Calculate pulse delay based on RPM and microstepping.
        
        Formula: StepsPerSec = (RPM × 200 × Microsteps) / 60
                 PulseDelay = 1 / StepsPerSec
        
        Args:
            rpm: Target RPM
            microsteps: Microstep resolution
        
        Returns:
            float: Pulse delay in seconds
        """
        steps_per_sec = (rpm * STEPS_PER_REVOLUTION * microsteps) / 60.0
        pulse_delay = 1.0 / steps_per_sec if steps_per_sec > 0 else 0.001
        return pulse_delay
    
    def generate_step_pulses(self, steps: int, direction: int, pulse_delay: float):
        """
        Generate step pulses via GPIO.
        
        Args:
            steps: Number of steps to execute
            direction: 1 for CW, -1 for CCW
            pulse_delay: Delay between pulses (seconds)
        """
        if not GPIO_AVAILABLE:
            # Simulate pulses if GPIO not available
            print(f"[SIMULATED] Generating {steps} step pulses...")
            time.sleep(steps * pulse_delay)
            return
        
        # Set direction
        GPIO.output(DIR_PIN, GPIO.HIGH if direction == 1 else GPIO.LOW)
        time.sleep(0.001)  # Small delay for direction to settle
        
        # Generate step pulses
        for _ in range(steps):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(pulse_delay / 2)  # Half period high
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(pulse_delay / 2)  # Half period low
    
    def execute_move(self, microsteps: int, current_settings: tuple, 
                     rpm: float, steps: int, direction: int):
        """Execute a move with the given parameters."""
        print("\n" + "=" * 60)
        print("Status Report:")
        print("=" * 60)
        print(f"Configuring TMC2208 via UART...")
        print(f"Moving {steps} steps at {rpm} RPM ({microsteps}x microstepping)")
        
        if current_settings[0] is not None:
            print(f"Current: {current_settings[0]} (run), {current_settings[1]} (hold)")
        
        print(f"Direction: {'CW' if direction == 1 else 'CCW'}")
        print("=" * 60)
        
        try:
            # Configure microstepping via UART
            print("\nSetting microstepping via UART...")
            if not self.motor.set_microstepping(microsteps):
                print("ERROR: Failed to set microstepping")
                return False
            print(f"✓ Microstepping set to {microsteps}x")
            
            # Small delay for register to take effect
            time.sleep(0.02)
            
            # Configure current if provided
            if current_settings[0] is not None:
                print("\nSetting current via UART...")
                if not self.motor.set_current(
                    current_settings[0],
                    current_settings[1],
                    current_settings[2]
                ):
                    print("ERROR: Failed to set current")
                    return False
                print(f"✓ Current configured")
                time.sleep(0.01)
            
            # Calculate pulse delay
            pulse_delay = self.calculate_pulse_delay(rpm, microsteps)
            steps_per_sec = 1.0 / pulse_delay if pulse_delay > 0 else 0
            
            print(f"\nCalculated pulse delay: {pulse_delay*1000:.3f} ms")
            print(f"Steps per second: {steps_per_sec:.1f}")
            print(f"Estimated duration: {steps * pulse_delay:.2f} seconds")
            
            # Generate step pulses
            print("\nGenerating step pulses via GPIO...")
            start_time = time.time()
            self.generate_step_pulses(steps, direction, pulse_delay)
            elapsed = time.time() - start_time
            
            print(f"\n✓ Move completed in {elapsed:.2f} seconds")
            return True
            
        except Exception as e:
            print(f"\nERROR during move: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def run(self):
        """Main interactive loop."""
        print("=" * 60)
        print("TMC2208 Interactive Test Program")
        print("=" * 60)
        print("Hardware Configuration:")
        print("  - UART: /dev/serial0 (GPIO 14/TXD)")
        print("  - STEP: GPIO 5")
        print("  - DIR: GPIO 6")
        print("  - Motor: NEMA 17 (200 steps/rev)")
        print("=" * 60)
        
        # Initialize motor
        if not self.initialize_motor():
            return 1
        
        print("\n" + "=" * 60)
        print("Interactive Test Loop")
        print("=" * 60)
        print("Enter parameters for each move.")
        print("Press Ctrl+C at any time to safely exit.\n")
        
        try:
            while self.running:
                try:
                    # Get user inputs
                    microsteps = self.get_microstepping_input()
                    current_settings = self.get_current_input()
                    rpm = self.get_speed_input()
                    steps = self.get_steps_input()
                    direction = self.get_direction_input()
                    
                    # Execute move
                    self.execute_move(microsteps, current_settings, rpm, steps, direction)
                    
                    # Ask if user wants to continue
                    print("\n" + "-" * 60)
                    continue_input = input("\nExecute another move? (y/n) [default: y]: ").strip().lower()
                    if continue_input == 'n':
                        break
                    
                except KeyboardInterrupt:
                    raise
                except Exception as e:
                    print(f"\nError: {e}")
                    continue
        
        except KeyboardInterrupt:
            pass
        
        finally:
            self.cleanup()
        
        print("\nTest program exited.")
        return 0


def main():
    """Main entry point."""
    test = TMC2208InteractiveTest()
    try:
        exit_code = test.run()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\nTest Stopped.")
        test.cleanup()
        sys.exit(0)


if __name__ == "__main__":
    main()


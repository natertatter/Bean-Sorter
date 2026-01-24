"""
TMC2208 Stepper Motor Driver - UART Control Implementation

This driver implements the MotorInterface using UART communication for
TMC2208 stepper motor drivers. It provides precise control through
UART register access instead of STEP/DIR GPIO pins.

Hardware Context:
    - TMC2208 Stepper Motor Driver (UART Mode)
    - Single-wire UART interface on PDN_UART pin
    - Raspberry Pi 4 with 3.3V logic (VIO pin)
    - Left Motor: UART0 (/dev/serial0) on GPIO 14/15
    - Right Motor: UART5 (/dev/ttyAMA1) on GPIO 12/13
    - Baudrate: 115200
    - Slave Address: 0x00

UART Protocol:
    - Write: 0x05 (sync) + 0x00 (reserved) + (0x80 | addr) + 32-bit data + CRC8
    - Read: 0x05 (sync) + 0x00 (reserved) + addr + CRC8
    - Response: 0x05 + 0x00 + addr + 32-bit data + CRC8
"""

import serial
import time
import logging
from typing import Optional
from abc import ABC

try:
    from ..interfaces.motor_interface import MotorInterface
except ImportError:
    # Fallback for standalone testing
    from src.interfaces.motor_interface import MotorInterface

logger = logging.getLogger(__name__)


class TMC2208UARTError(Exception):
    """Exception raised for TMC2208 UART communication errors."""
    pass


class TMC2208(MotorInterface):
    """
    TMC2208 stepper motor driver with UART control.
    
    Implements the MotorInterface using UART register access for motor control.
    All hardware operations are wrapped in try/finally blocks to ensure
    proper cleanup and safe-state shutdowns.
    """
    
    # TMC2208 Register Addresses
    REG_GCONF = 0x00       # Global Configuration
    REG_GSTAT = 0x01       # Global Status
    REG_IHOLD_IRUN = 0x10  # Current Control
    REG_CHOPCONF = 0x6C    # Chopper Configuration (Microstepping)
    REG_XACTUAL = 0x21     # Actual Position
    REG_XTARGET = 0x2D     # Target Position
    REG_VACTUAL = 0x22     # Actual Velocity
    REG_VSTART = 0x23      # Start Velocity
    REG_VSTOP = 0x24       # Stop Velocity
    REG_VMAX = 0x27        # Maximum Velocity
    REG_A1 = 0x24          # Acceleration 1
    REG_D1 = 0x25          # Deceleration 1
    REG_DMAX = 0x33        # Maximum Deceleration
    REG_RAMPMODE = 0x20    # Ramp Mode
    
    # CHOPCONF Register Bits
    CHOPCONF_MRES_MASK = 0x0F000000  # Bits 24-27: Microstep Resolution
    CHOPCONF_MRES_SHIFT = 24
    
    # GCONF bits
    GCONF_PDN_DISABLE = 0x00000001  # Bit 0: Disable hardware power-down pin
    
    # CHOPCONF Register Bits
    CHOPCONF_MRES_MASK = 0x0F000000  # Bits 24-27: Microstep Resolution
    CHOPCONF_MRES_SHIFT = 24
    
    # RAMPMODE values
    RAMPMODE_POSITION = 0
    RAMPMODE_VELPOS = 1
    RAMPMODE_VELNEG = 2
    RAMPMODE_HOLD = 3
    
    # UART Protocol Constants
    SYNC_BYTE = 0x05
    RESERVED_BYTE = 0x00
    SLAVE_ADDRESS = 0x00
    WRITE_BIT = 0x80
    UART_TIMEOUT = 1.0  # seconds
    
    def __init__(self, uart_port: str = '/dev/serial0', baudrate: int = 115200):
        """
        Initialize TMC2208 driver with UART communication.
        
        Args:
            uart_port: UART device path (e.g., '/dev/serial0' or '/dev/ttyAMA1')
            baudrate: UART baudrate (default: 115200)
        """
        super().__init__()
        self.uart_port = uart_port
        self.baudrate = baudrate
        self.uart: Optional[serial.Serial] = None
        self._current_position = 0
        self._is_moving = False
        
    def __repr__(self):
        return f"TMC2208(uart_port='{self.uart_port}', initialized={self._is_initialized})"
    
    @staticmethod
    def _crc8_atm(data: bytes) -> int:
        """
        Calculate CRC8-ATM checksum for TMC2208 UART protocol.
        
        Uses polynomial 0x07 (x^8 + x^2 + x^1 + 1).
        
        Args:
            data: Bytes to calculate CRC for
            
        Returns:
            int: 8-bit CRC value
        """
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x01:
                    crc = (crc >> 1) ^ 0x07
                else:
                    crc >>= 1
        return crc & 0xFF
    
    def _write_register(self, address: int, value: int) -> bool:
        """
        Write a 32-bit value to a TMC2208 register via UART.
        
        Datagram format:
        - Byte 0: 0x05 (Sync + Reserved high nibble)
        - Byte 1: 0x00 (Reserved low nibble + Slave Address high)
        - Byte 2: 0x00 (Slave Address low) | (address | 0x80 for write)
        - Bytes 3-6: 32-bit data (MSB first)
        - Byte 7: CRC8-ATM checksum
        
        Args:
            address: Register address (0x00-0x7F)
            value: 32-bit value to write
            
        Returns:
            bool: True if write successful, False otherwise
            
        Raises:
            TMC2208UARTError: If UART communication fails
        """
        # Check only if UART is open (allow during initialization)
        if self.uart is None:
            raise TMC2208UARTError("TMC2208 UART not open")
        
        # Build write datagram
        # Format: SYNC(0x05) + RESERVED(0x00) + ADDR_WRITE(addr|0x80) + DATA(32-bit MSB) + CRC
        addr_byte = (address & 0x7F) | self.WRITE_BIT
        data_bytes = bytes([
            self.SYNC_BYTE,
            self.RESERVED_BYTE,
            addr_byte,
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF
        ])
        
        # Calculate CRC for all bytes except the CRC itself
        crc = self._crc8_atm(data_bytes)
        data_bytes += bytes([crc])
        
        try:
            # Clear input buffer
            self.uart.reset_input_buffer()
            
            # Send write command
            self.uart.write(data_bytes)
            self.uart.flush()
            
            # Small delay for processing
            time.sleep(0.001)
            
            logger.debug(f"Wrote register 0x{address:02X} = 0x{value:08X}")
            return True
            
        except serial.SerialException as e:
            logger.error(f"UART write error: {e}")
            raise TMC2208UARTError(f"Failed to write register 0x{address:02X}: {e}")
    
    def _read_register(self, address: int) -> Optional[int]:
        """
        Read a 32-bit value from a TMC2208 register via UART.
        
        Write datagram format:
        - Byte 0: 0x05 (Sync + Reserved high nibble)
        - Byte 1: 0x00 (Reserved low nibble + Slave Address)
        - Byte 2: address (read, no 0x80 bit)
        - Byte 3: CRC8-ATM checksum
        
        Response datagram format:
        - Byte 0: 0x05 (Sync)
        - Byte 1: 0x00 (Reserved + Slave Address)
        - Byte 2: address
        - Bytes 3-6: 32-bit data (MSB first)
        - Byte 7: CRC8-ATM checksum
        
        Args:
            address: Register address (0x00-0x7F)
            
        Returns:
            int: 32-bit register value, or None if read failed
            
        Raises:
            TMC2208UARTError: If UART communication fails
        """
        # Check only if UART is open (allow during initialization)
        if self.uart is None:
            raise TMC2208UARTError("TMC2208 UART not open")
        
        # Build read request datagram
        addr_byte = address & 0x7F  # No write bit for reads
        request_bytes = bytes([
            self.SYNC_BYTE,
            self.RESERVED_BYTE,
            addr_byte
        ])
        
        # Calculate CRC for request
        crc = self._crc8_atm(request_bytes)
        request_bytes += bytes([crc])
        
        try:
            # Clear input buffer
            self.uart.reset_input_buffer()
            
            # Send read request
            self.uart.write(request_bytes)
            self.uart.flush()
            
            # Wait for response (8 bytes: sync + reserved + addr + 32-bit data + CRC)
            response = self.uart.read(8)
            
            if len(response) < 8:
                logger.warning(f"Incomplete response for register 0x{address:02X}: {len(response)} bytes")
                return None
            
            # Verify sync byte
            if response[0] != self.SYNC_BYTE:
                logger.warning(f"Invalid sync byte in response: 0x{response[0]:02X}")
                return None
            
            # Verify address matches
            if (response[2] & 0x7F) != address:
                logger.warning(f"Address mismatch: expected 0x{address:02X}, got 0x{response[2]:02X}")
                return None
            
            # Verify CRC
            response_data = response[:7]  # All bytes except CRC
            response_crc = response[7]
            calculated_crc = self._crc8_atm(response_data)
            
            if response_crc != calculated_crc:
                logger.warning(
                    f"CRC mismatch for register 0x{address:02X}: "
                    f"expected 0x{calculated_crc:02X}, got 0x{response_crc:02X}"
                )
                return None
            
            # Extract 32-bit value (MSB first)
            value = (
                (response[3] << 24) |
                (response[4] << 16) |
                (response[5] << 8) |
                response[6]
            )
            
            logger.debug(f"Read register 0x{address:02X} = 0x{value:08X}")
            return value
            
        except serial.SerialException as e:
            logger.error(f"UART read error: {e}")
            raise TMC2208UARTError(f"Failed to read register 0x{address:02X}: {e}")
    
    def initialize(self) -> bool:
        """
        Initialize the TMC2208 driver.
        
        This method:
        1. Opens the UART connection
        2. Sets GCONF.pdn_disable = 1 to enable UART control
        3. Configures default motor parameters
        
        Returns:
            bool: True if initialization successful
            
        Raises:
            TMC2208UARTError: If initialization fails
        """
        try:
            # Open UART connection
            self.uart = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                timeout=self.UART_TIMEOUT,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Small delay for UART to stabilize
            time.sleep(0.1)
            
            # Set GCONF.pdn_disable = 1 to disable hardware PDN and enable UART control
            # This is critical for UART mode operation
            gconf_value = self.GCONF_PDN_DISABLE
            if not self._write_register(self.REG_GCONF, gconf_value):
                raise TMC2208UARTError("Failed to set GCONF register")
            
            # Verify GCONF was written correctly
            time.sleep(0.01)
            gconf_read = self._read_register(self.REG_GCONF)
            if gconf_read is None or (gconf_read & self.GCONF_PDN_DISABLE) == 0:
                logger.warning("GCONF verification failed - UART mode may not be active")
            
            # Set default current (can be adjusted later with set_current())
            # IHOLD_IRUN: bits 0-4 = IHOLD, bits 8-12 = IRUN, bits 16-20 = IHOLDDELAY
            # Default: IRUN=16 (50% of max), IHOLD=8 (25% of max), IHOLDDELAY=4
            default_current = (4 << 16) | (16 << 8) | 8
            self._write_register(self.REG_IHOLD_IRUN, default_current)
            
            # Set motor to hold position initially
            self.stop()
            
            self._is_initialized = True
            logger.info(f"TMC2208 initialized on {self.uart_port}")
            return True
            
        except serial.SerialException as e:
            logger.error(f"Failed to initialize UART on {self.uart_port}: {e}")
            self._cleanup()
            raise TMC2208UARTError(f"UART initialization failed: {e}")
        except Exception as e:
            logger.error(f"TMC2208 initialization error: {e}", exc_info=True)
            self._cleanup()
            raise TMC2208UARTError(f"Initialization failed: {e}")
    
    def set_microstepping(self, microsteps: int) -> bool:
        """
        Set microstepping resolution via CHOPCONF register.
        
        Args:
            microsteps: Microstep resolution (1, 2, 4, 8, 16, 32, 64, 128, or 256)
        
        Returns:
            bool: True if microstepping set successfully
        
        Note:
            MRES mapping: 256=0, 128=1, 64=2, 32=3, 16=4, 8=5, 4=6, 2=7, 1=8
        """
        if not self._is_initialized:
            raise TMC2208UARTError("TMC2208 not initialized")
        
        # Map microsteps to MRES value
        mres_map = {
            256: 0, 128: 1, 64: 2, 32: 3, 16: 4,
            8: 5, 4: 6, 2: 7, 1: 8
        }
        
        if microsteps not in mres_map:
            raise ValueError(
                f"Invalid microstep value: {microsteps}. "
                f"Valid values: {list(mres_map.keys())}"
            )
        
        mres_value = mres_map[microsteps]
        
        try:
            # Read current CHOPCONF value
            chopconf = self._read_register(self.REG_CHOPCONF)
            if chopconf is None:
                # If read fails, use default value
                chopconf = 0x00010080  # Default CHOPCONF value
            
            # Clear MRES bits (24-27) and set new value
            chopconf = (chopconf & ~self.CHOPCONF_MRES_MASK) | (mres_value << self.CHOPCONF_MRES_SHIFT)
            
            # Write updated CHOPCONF
            if not self._write_register(self.REG_CHOPCONF, chopconf):
                logger.error("Failed to set CHOPCONF register")
                return False
            
            # Small delay for register to take effect
            time.sleep(0.01)
            
            logger.info(f"Microstepping set to {microsteps}x (MRES={mres_value})")
            return True
            
        except Exception as e:
            logger.error(f"Failed to set microstepping: {e}")
            return False
    
    def set_current(self, run_current: int, hold_current: int, hold_delay: int = 4) -> bool:
        """
        Set motor current using IHOLD_IRUN register (0x10).
        
        Args:
            run_current: Run current value (0-31, typically 0-16 for safe operation)
            hold_current: Hold current value (0-31, typically 0-16)
            hold_delay: Hold delay value (0-15, delay before reducing to hold current)
            
        Returns:
            bool: True if current set successfully
        """
        if not self._is_initialized:
            raise TMC2208UARTError("TMC2208 not initialized")
        
        # Clamp values to valid ranges
        run_current = max(0, min(31, run_current))
        hold_current = max(0, min(31, hold_current))
        hold_delay = max(0, min(15, hold_delay))
        
        # Pack into 32-bit register value
        # Bits 0-4: IHOLD, Bits 8-12: IRUN, Bits 16-20: IHOLDDELAY
        value = (hold_delay << 16) | (run_current << 8) | hold_current
        
        try:
            return self._write_register(self.REG_IHOLD_IRUN, value)
        except Exception as e:
            logger.error(f"Failed to set current: {e}")
            return False
    
    def step(self, count: int, direction: int) -> bool:
        """
        Execute a specified number of steps in the given direction.
        
        Uses UART register access to control motor movement via XTARGET register.
        All operations are wrapped in try/finally for safety.
        
        Args:
            count: Number of steps to execute (must be positive)
            direction: Direction of rotation (1 or True for forward, -1 or False for reverse)
        
        Returns:
            bool: True if steps were executed successfully, False otherwise
        
        Raises:
            ValueError: If count is negative or direction is invalid
            RuntimeError: If motor is not properly initialized
        """
        if not self._is_initialized:
            raise RuntimeError("TMC2208 not initialized")
        
        if count < 0:
            raise ValueError(f"Step count must be positive, got {count}")
        
        # Normalize direction: 1 or True = forward, -1 or False = reverse
        if direction in (True, 1):
            step_direction = 1
        elif direction in (False, -1, 0):
            step_direction = -1
        else:
            raise ValueError(f"Invalid direction: {direction}. Use 1/-1 or True/False")
        
        try:
            # Read current position
            current_pos = self._read_register(self.REG_XACTUAL)
            if current_pos is None:
                current_pos = self._current_position
            else:
                self._current_position = current_pos
            
            # Calculate target position
            target_position = current_pos + (step_direction * count)
            
            # Set target position (motor will move to this position)
            if not self._write_register(self.REG_XTARGET, target_position):
                logger.error("Failed to set target position")
                return False
            
            # Update internal position tracking
            self._current_position = target_position
            self._is_moving = True
            
            # Wait for movement to complete (simplified - could be improved with status polling)
            # In a real implementation, you might want to poll XACTUAL until it matches XTARGET
            time.sleep(0.01 * count)  # Rough estimate: 10ms per step
            
            self._is_moving = False
            logger.debug(f"Executed {count} steps in direction {step_direction}")
            return True
            
        except TMC2208UARTError as e:
            logger.error(f"UART error during step: {e}")
            return False
        except Exception as e:
            logger.error(f"Error during step execution: {e}", exc_info=True)
            return False
        finally:
            # Ensure motor is in safe state even if error occurred
            try:
                self._set_safe_state()
            except Exception as e:
                logger.error(f"Error setting safe state after step: {e}")
    
    def stop(self) -> None:
        """
        Stop the motor immediately and enter safe state.
        
        Sets the motor to hold position and ensures proper cleanup.
        All operations are wrapped in try/finally for safety.
        
        Raises:
            RuntimeError: If hardware shutdown fails (cleanup still attempted)
        """
        try:
            if not self._is_initialized:
                return
            
            # Set target position to current position (stop movement)
            if self.uart is not None:
                current_pos = self._read_register(self.REG_XACTUAL)
                if current_pos is not None:
                    self._write_register(self.REG_XTARGET, current_pos)
                    self._current_position = current_pos
                
                # Set RAMPMODE to HOLD to stop any velocity-based movement
                self._write_register(self.REG_RAMPMODE, self.RAMPMODE_HOLD)
            
            self._is_moving = False
            logger.debug("Motor stopped")
            
        except Exception as e:
            logger.error(f"Error during motor stop: {e}", exc_info=True)
            # Continue with cleanup even if stop command failed
        finally:
            try:
                self._set_safe_state()
            except Exception as e:
                logger.error(f"Error in stop() finally block: {e}")
    
    def _cleanup(self) -> None:
        """
        Internal cleanup method for UART and hardware resources.
        
        Closes UART connection and ensures motor is in safe state.
        Idempotent - safe to call multiple times.
        """
        try:
            if self.uart is not None:
                # Ensure motor is stopped before closing
                self._set_safe_state()
                
                # Close UART connection
                self.uart.close()
                self.uart = None
            
            self._is_initialized = False
            self._is_moving = False
            logger.debug("TMC2208 cleanup completed")
            
        except Exception as e:
            logger.error(f"Error during cleanup: {e}", exc_info=True)
    
    def _set_safe_state(self) -> None:
        """
        Set motor and driver to a safe operating state.
        
        Called in finally blocks to ensure safe state even if errors occur.
        """
        try:
            if not self._is_initialized or self.uart is None:
                return
            
            # Set RAMPMODE to HOLD
            try:
                self._write_register(self.REG_RAMPMODE, self.RAMPMODE_HOLD)
            except Exception:
                pass  # Ignore errors during safe state setting
            
            # Set target to current position (if we can read it)
            try:
                current_pos = self._read_register(self.REG_XACTUAL)
                if current_pos is not None:
                    self._write_register(self.REG_XTARGET, current_pos)
            except Exception:
                pass  # Ignore errors during safe state setting
            
        except Exception as e:
            logger.error(f"Error in _set_safe_state: {e}", exc_info=True)


class MockTMC2208(TMC2208):
    """
    Mock TMC2208 driver for testing without hardware.
    
    Simulates UART register responses for testing on development machines
    without actual TMC2208 hardware or UART connections.
    """
    
    def __init__(self, uart_port: str = '/dev/serial0', baudrate: int = 115200):
        """Initialize mock TMC2208 driver."""
        # Don't call super().__init__() - we'll initialize differently
        MotorInterface.__init__(self)
        self.uart_port = uart_port
        self.baudrate = baudrate
        self._registers = {}
        self._current_position = 0
        self._is_moving = False
        self._is_initialized = False
    
    def __repr__(self):
        return f"MockTMC2208(uart_port='{self.uart_port}', initialized={self._is_initialized})"
    
    def initialize(self) -> bool:
        """Initialize mock TMC2208 - simulates successful initialization."""
        try:
            # Simulate register initialization
            self._registers[self.REG_GCONF] = self.GCONF_PDN_DISABLE
            self._registers[self.REG_XACTUAL] = 0
            self._registers[self.REG_XTARGET] = 0
            self._registers[self.REG_RAMPMODE] = self.RAMPMODE_HOLD
            self._registers[self.REG_IHOLD_IRUN] = (4 << 16) | (16 << 8) | 8
            
            self._is_initialized = True
            logger.info(f"MockTMC2208 initialized (simulated {self.uart_port})")
            return True
        except Exception as e:
            logger.error(f"Mock initialization error: {e}")
            return False
    
    def _write_register(self, address: int, value: int) -> bool:
        """Simulate writing to a register."""
        if not self._is_initialized:
            raise TMC2208UARTError("MockTMC2208 not initialized")
        
        self._registers[address] = value
        logger.debug(f"[MOCK] Wrote register 0x{address:02X} = 0x{value:08X}")
        
        # Simulate position updates
        if address == self.REG_XTARGET:
            self._registers[self.REG_XACTUAL] = value
            self._current_position = value
        
        return True
    
    def _read_register(self, address: int) -> Optional[int]:
        """Simulate reading from a register."""
        if not self._is_initialized:
            raise TMC2208UARTError("MockTMC2208 not initialized")
        
        # Return register value or 0 if not set
        value = self._registers.get(address, 0)
        logger.debug(f"[MOCK] Read register 0x{address:02X} = 0x{value:08X}")
        return value
    
    def set_current(self, run_current: int, hold_current: int, hold_delay: int = 4) -> bool:
        """Simulate setting motor current."""
        if not self._is_initialized:
            raise TMC2208UARTError("MockTMC2208 not initialized")
        
        run_current = max(0, min(31, run_current))
        hold_current = max(0, min(31, hold_current))
        hold_delay = max(0, min(15, hold_delay))
        
        value = (hold_delay << 16) | (run_current << 8) | hold_current
        return self._write_register(self.REG_IHOLD_IRUN, value)
    
    def step(self, count: int, direction: int) -> bool:
        """Simulate step execution."""
        if not self._is_initialized:
            raise RuntimeError("MockTMC2208 not initialized")
        
        if count < 0:
            raise ValueError(f"Step count must be positive, got {count}")
        
        if direction in (True, 1):
            step_direction = 1
        elif direction in (False, -1, 0):
            step_direction = -1
        else:
            raise ValueError(f"Invalid direction: {direction}")
        
        try:
            current_pos = self._read_register(self.REG_XACTUAL) or self._current_position
            target_position = current_pos + (step_direction * count)
            
            self._write_register(self.REG_XTARGET, target_position)
            self._current_position = target_position
            self._is_moving = True
            
            # Simulate movement delay
            time.sleep(0.001 * count)  # Faster for testing
            
            self._is_moving = False
            logger.debug(f"[MOCK] Executed {count} steps in direction {step_direction}")
            return True
            
        except Exception as e:
            logger.error(f"[MOCK] Error during step: {e}", exc_info=True)
            return False
        finally:
            try:
                self._set_safe_state()
            except Exception:
                pass
    
    def stop(self) -> None:
        """Simulate stopping the motor."""
        try:
            if not self._is_initialized:
                return
            
            current_pos = self._read_register(self.REG_XACTUAL) or self._current_position
            self._write_register(self.REG_XTARGET, current_pos)
            self._write_register(self.REG_RAMPMODE, self.RAMPMODE_HOLD)
            self._is_moving = False
            logger.debug("[MOCK] Motor stopped")
            
        except Exception as e:
            logger.error(f"[MOCK] Error during stop: {e}")
        finally:
            try:
                self._set_safe_state()
            except Exception:
                pass
    
    def _cleanup(self) -> None:
        """Cleanup mock resources."""
        try:
            self._registers.clear()
            self._is_initialized = False
            self._is_moving = False
            logger.debug("[MOCK] MockTMC2208 cleanup completed")
        except Exception as e:
            logger.error(f"[MOCK] Error during cleanup: {e}")
    
    def _set_safe_state(self) -> None:
        """Set mock motor to safe state."""
        try:
            if not self._is_initialized:
                return
            
            self._write_register(self.REG_RAMPMODE, self.RAMPMODE_HOLD)
            current_pos = self._read_register(self.REG_XACTUAL) or 0
            self._write_register(self.REG_XTARGET, current_pos)
        except Exception:
            pass


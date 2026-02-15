"""
TMC2208 Stepper Motor Driver - UART Control Implementation

This driver implements the MotorInterface using UART communication for
TMC2208 stepper motor drivers. It provides precise control through
UART register access instead of STEP/DIR GPIO pins.

Hardware Context:
    - TMC2208 Stepper Motor Driver (UART Mode)
    - Single-wire half-duplex UART interface on PDN_UART pin
    - Raspberry Pi 4 with 3.3V logic (VIO pin)
    - Left Motor: UART0 (/dev/serial0) on GPIO 14/15
    - Right Motor: UART5 (/dev/ttyAMA1) on GPIO 12/13
    - Wiring: GPIO 14 (TX) via 1kΩ resistor to PDN_UART, GPIO 15 (RX) direct to PDN_UART
    - Baudrate: 115200
    - Slave Address: 0x00
    - Protocol: Half-duplex (TX and RX share same line; timing delays required)

UART Protocol:
    - Write: 0x05 (sync) + 0x00 (reserved) + (0x80 | addr) + 32-bit data + CRC8
    - Read: 0x05 (sync) + 0x00 (reserved) + addr + CRC8
    - Response: 0x05 + 0x00 + addr + 32-bit data + CRC8
"""

import serial
import time
import logging
from typing import Optional, Any
from abc import ABC

# Optional GPIO for STEP/DIR mode (may not be available on dev machines)
try:
    from gpiozero import DigitalOutputDevice
    _GPIOZERO_AVAILABLE = True
except ImportError:
    _GPIOZERO_AVAILABLE = False
    DigitalOutputDevice = None

try:
    from ..interfaces.motor_interface import MotorInterface
    from ..config import Pins, MotorConfig, DEFAULT_PINS, DEFAULT_MOTOR_CONFIG, UART_BAUDRATE
except ImportError:
    # Fallback for standalone testing
    from src.interfaces.motor_interface import MotorInterface
    from src.config import Pins, MotorConfig, DEFAULT_PINS, DEFAULT_MOTOR_CONFIG, UART_BAUDRATE

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
    # Note: These are BASE addresses. The _write_register() method automatically
    # adds 0x80 for write operations (e.g., GCONF 0x00 → 0x80 when writing).
    # For reads, use the base address directly.
    # Write Access: Base address + 0x80 (handled automatically by _write_register)
    # Read Access: Use base address directly
    REG_GCONF = 0x00       # Global Configuration (base address, becomes 0x80 when writing)
    REG_GSTAT = 0x01       # Global Status
    REG_IHOLD_IRUN = 0x10  # Current Control
    REG_CHOPCONF = 0x6C    # Chopper Configuration (Microstepping)
    REG_DRV_STATUS = 0x6F  # Driver Status (Diagnostic information)
    REG_XACTUAL = 0x21     # Actual Position
    REG_XTARGET = 0x2D     # Target Position
    REG_VACTUAL = 0x22     # Actual Velocity (0=GPIO control, !=0=internal pulse generator)
    REG_VSTART = 0x23      # Start Velocity
    REG_VSTOP = 0x24       # Stop Velocity
    REG_VMAX = 0x27        # Maximum Velocity
    REG_A1 = 0x24          # Acceleration 1
    REG_D1 = 0x25          # Deceleration 1
    REG_DMAX = 0x33        # Maximum Deceleration
    REG_RAMPMODE = 0x20    # Ramp Mode
    
    # Register Map Dictionary for reference
    REGISTER_MAP = {
        'GCONF': 0x00,      # Global Configuration
        'GSTAT': 0x01,      # Global Status
        'IHOLD_IRUN': 0x10, # Current Control
        'CHOPCONF': 0x6C,   # Chopper Configuration
        'DRV_STATUS': 0x6F, # Driver Status
        'XACTUAL': 0x21,    # Actual Position
        'XTARGET': 0x2D,    # Target Position
        'VACTUAL': 0x22,    # Actual Velocity
        'VSTART': 0x23,     # Start Velocity
        'VSTOP': 0x24,      # Stop Velocity
        'VMAX': 0x27,       # Maximum Velocity
        'A1': 0x24,        # Acceleration 1
        'D1': 0x25,        # Deceleration 1
        'DMAX': 0x33,       # Maximum Deceleration
        'RAMPMODE': 0x20,   # Ramp Mode
    }
    
    # CHOPCONF Register Bits
    CHOPCONF_MRES_MASK = 0x0F000000  # Bits 24-27: Microstep Resolution
    CHOPCONF_MRES_SHIFT = 24
    
    # GCONF bits
    GCONF_PDN_DISABLE = 0x00000001  # Bit 0: Disable hardware power-down pin
    
    # DRV_STATUS Register Bits
    DRV_STATUS_STST = 0x00000001      # Bit 0: Standstill indicator
    DRV_STATUS_OLB = 0x00000002       # Bit 1: Open load B
    DRV_STATUS_OLA = 0x00000004       # Bit 2: Open load A
    DRV_STATUS_S2GB = 0x00000008      # Bit 3: Short to ground B
    DRV_STATUS_S2GA = 0x00000010      # Bit 4: Short to ground A
    DRV_STATUS_OTPW = 0x00000020      # Bit 5: Overtemperature pre-warning
    DRV_STATUS_OT = 0x00000040        # Bit 6: Overtemperature
    DRV_STATUS_STALLGUARD = 0x00000100  # Bit 8: StallGuard status
    DRV_STATUS_CS_ACTUAL_MASK = 0x1F0000  # Bits 16-20: Actual current scale
    DRV_STATUS_CS_ACTUAL_SHIFT = 16
    DRV_STATUS_STALLGUARD_MASK = 0xFE00  # Bits 9-15: StallGuard value
    DRV_STATUS_STALLGUARD_SHIFT = 9
    
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

    # Half-duplex timing (single-wire: TX/RX share line; delays for line turnaround)
    HALF_DUPLEX_WRITE_DELAY = 0.005   # 5ms after write
    HALF_DUPLEX_READ_DELAY = 0.005    # 5ms after read request before reading response
    HALF_DUPLEX_RETRY_DELAY = 0.010   # 10ms extra wait if response incomplete

    def __init__(
        self,
        uart_port: str = '/dev/serial0',
        baudrate: Optional[int] = None,
        pins: Optional[Pins] = None,
        motor_config: Optional[MotorConfig] = None,
    ):
        """
        Initialize TMC2208 driver with UART communication.

        Args:
            uart_port: UART device path (e.g., '/dev/serial0' or '/dev/ttyAMA1').
                       /dev/serial0 uses Pins.TX/RX (GPIO 14/15).
            baudrate: UART baudrate. Defaults to config UART_BAUDRATE.
            pins: Pin definitions (STEP, DIR, TX, RX). Used for GPIO mode if needed.
            motor_config: Motor settings (IRUN, IHOLD, microsteps). Used during init.
        """
        super().__init__()
        self.uart_port = uart_port
        self.baudrate = baudrate if baudrate is not None else UART_BAUDRATE
        self.pins = pins if pins is not None else DEFAULT_PINS
        self.motor_config = motor_config if motor_config is not None else DEFAULT_MOTOR_CONFIG
        self.uart: Optional[serial.Serial] = None
        self._step_pin: Any = None
        self._dir_pin: Any = None
        self._enn_pin: Any = None
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
        # IMPORTANT: Automatically adds 0x80 to address for write operations
        # Example: REG_GCONF (0x00) becomes 0x80 when writing
        #          REG_IHOLD_IRUN (0x10) becomes 0x90 when writing
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
            
            # Half-duplex delay: Wait for transmission to complete in single-wire mode.
            # TX and RX share the same line, so we need time for the line to go idle.
            time.sleep(self.HALF_DUPLEX_WRITE_DELAY)
            
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
            
            # Half-duplex delay: Wait for line turnaround in single-wire mode.
            # TX and RX share the same line, so we need time for the TMC2208 to switch to TX mode.
            time.sleep(self.HALF_DUPLEX_READ_DELAY)
            
            # Wait for response (8 bytes: sync + reserved + addr + 32-bit data + CRC)
            response = self.uart.read(8)
            
            # If incomplete, wait a bit more (half-duplex can be slower)
            if len(response) < 8:
                time.sleep(self.HALF_DUPLEX_RETRY_DELAY)
                additional = self.uart.read(8 - len(response))
                response += additional
            
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
            
            # Verify GCONF was written correctly.
            # In half-duplex mode, reads may fail initially, so we make this non-critical.
            time.sleep(0.01)
            try:
                gconf_read = self._read_register(self.REG_GCONF)
                if gconf_read is None or (gconf_read & self.GCONF_PDN_DISABLE) == 0:
                    logger.warning("GCONF verification failed - UART mode may not be active")
                    logger.warning("This may be normal in half-duplex mode - continuing anyway")
            except Exception as e:
                logger.warning(f"GCONF read verification failed: {e}")
                logger.warning("This may be normal in half-duplex mode - continuing anyway")
            
            # Set current from MotorConfig (IRUN, IHOLD, IHOLDDELAY)
            # IHOLD_IRUN: bits 0-4 = IHOLD, bits 8-12 = IRUN, bits 16-20 = IHOLDDELAY
            mc = self.motor_config
            default_current = (mc.ihold_delay << 16) | (mc.irun << 8) | mc.ihold
            self._write_register(self.REG_IHOLD_IRUN, default_current)

            # Optional: Setup STEP/DIR GPIO pins for external pulse mode
            if _GPIOZERO_AVAILABLE and DigitalOutputDevice is not None:
                try:
                    self._step_pin = DigitalOutputDevice(self.pins.STEP, initial_value=False)
                    self._dir_pin = DigitalOutputDevice(self.pins.DIR, initial_value=False)
                    if self.pins.ENN is not None:
                        self._enn_pin = DigitalOutputDevice(self.pins.ENN, initial_value=False)
                    logger.debug(f"STEP/DIR pins initialized: STEP={self.pins.STEP}, DIR={self.pins.DIR}")
                except Exception as e:
                    logger.warning(f"Could not initialize STEP/DIR GPIO: {e}")

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
    
    def move_using_vactual(self, velocity: int, duration: Optional[float] = None) -> bool:
        """
        Move motor using VACTUAL register (velocity mode).
        
        When VACTUAL != 0, the motor is controlled by the internal pulse generator
        instead of GPIO STEP pin. This allows testing without GPIO pulses.
        
        Args:
            velocity: Velocity value (signed 24-bit)
                     - Positive: Forward direction
                     - Negative: Reverse direction
                     - 0: Stop (returns to GPIO STEP control)
            duration: Optional duration in seconds. If None, motor runs until stopped.
        
        Returns:
            bool: True if velocity set successfully
        
        Note:
            - VACTUAL = 0: Motor controlled by GPIO STEP pin
            - VACTUAL != 0: Motor controlled by internal pulse generator
            - Velocity value determines speed and direction
        """
        if not self._is_initialized:
            raise TMC2208UARTError("TMC2208 not initialized")
        
        try:
            # Clamp velocity to signed 24-bit range
            max_velocity = 0x7FFFFF  # Maximum positive 24-bit value
            min_velocity = -0x800000  # Minimum negative 24-bit value
            velocity = max(min_velocity, min(max_velocity, velocity))
            
            # Convert signed to unsigned 32-bit (two's complement for negative)
            if velocity < 0:
                vactual_value = (1 << 24) + velocity  # Two's complement
            else:
                vactual_value = velocity & 0xFFFFFF
            
            # Write VACTUAL register
            if not self._write_register(self.REG_VACTUAL, vactual_value):
                logger.error("Failed to set VACTUAL register")
                return False
            
            logger.info(f"VACTUAL set to {velocity} (0x{vactual_value:06X})")
            
            # If duration specified, wait then stop
            if duration is not None and duration > 0:
                time.sleep(duration)
                self._write_register(self.REG_VACTUAL, 0)  # Stop (return to GPIO control)
                logger.info("VACTUAL stopped after duration")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to set VACTUAL: {e}")
            return False
    
    def get_drv_status(self) -> Optional[dict]:
        """
        Read DRV_STATUS register and return diagnostic information.
        
        Returns:
            dict: Dictionary containing status information with keys:
                - 'raw_value': Raw register value
                - 'standstill': bool - Motor is standing still
                - 'open_load_a': bool - Open load detected on coil A
                - 'open_load_b': bool - Open load detected on coil B
                - 'short_to_ground_a': bool - Short to ground on coil A
                - 'short_to_ground_b': bool - Short to ground on coil B
                - 'overtemp_warning': bool - Overtemperature pre-warning
                - 'overtemp': bool - Overtemperature shutdown
                - 'stallguard': bool - StallGuard active
                - 'stallguard_value': int - StallGuard value (0-255)
                - 'cs_actual': int - Actual current scale (0-31)
                - 'status_ok': bool - Overall status (True if no errors)
                - 'warnings': list - List of warning messages
                - 'errors': list - List of error messages
        """
        if self.uart is None:
            return None
        
        try:
            status_raw = self._read_register(self.REG_DRV_STATUS)
            if status_raw is None:
                return None
            
            # Parse status bits
            standstill = bool(status_raw & self.DRV_STATUS_STST)
            open_load_a = bool(status_raw & self.DRV_STATUS_OLA)
            open_load_b = bool(status_raw & self.DRV_STATUS_OLB)
            short_ground_a = bool(status_raw & self.DRV_STATUS_S2GA)
            short_ground_b = bool(status_raw & self.DRV_STATUS_S2GB)
            overtemp_warning = bool(status_raw & self.DRV_STATUS_OTPW)
            overtemp = bool(status_raw & self.DRV_STATUS_OT)
            stallguard = bool(status_raw & self.DRV_STATUS_STALLGUARD)
            
            # Extract StallGuard value (bits 9-15)
            stallguard_value = (status_raw & self.DRV_STATUS_STALLGUARD_MASK) >> self.DRV_STATUS_STALLGUARD_SHIFT
            
            # Extract actual current scale (bits 16-20)
            cs_actual = (status_raw & self.DRV_STATUS_CS_ACTUAL_MASK) >> self.DRV_STATUS_CS_ACTUAL_SHIFT
            
            # Build warnings and errors lists
            warnings = []
            errors = []
            
            if overtemp:
                errors.append("Overtemperature shutdown - driver disabled")
            elif overtemp_warning:
                warnings.append("Overtemperature pre-warning")
            
            if short_ground_a:
                errors.append("Short to ground detected on coil A")
            if short_ground_b:
                errors.append("Short to ground detected on coil B")
            
            if open_load_a:
                warnings.append("Open load detected on coil A")
            if open_load_b:
                warnings.append("Open load detected on coil B")
            
            status_ok = len(errors) == 0
            
            return {
                'raw_value': status_raw,
                'standstill': standstill,
                'open_load_a': open_load_a,
                'open_load_b': open_load_b,
                'short_to_ground_a': short_ground_a,
                'short_to_ground_b': short_ground_b,
                'overtemp_warning': overtemp_warning,
                'overtemp': overtemp,
                'stallguard': stallguard,
                'stallguard_value': stallguard_value,
                'cs_actual': cs_actual,
                'status_ok': status_ok,
                'warnings': warnings,
                'errors': errors
            }
            
        except Exception as e:
            logger.error(f"Error reading DRV_STATUS: {e}")
            return None
    
    def get_diagnostic_status(self) -> Optional[dict]:
        """
        Get comprehensive diagnostic status from DRV_STATUS register.
        
        This is an alias for get_drv_status() for consistency with the API.
        
        Returns:
            dict: Dictionary containing diagnostic status information
        """
        return self.get_drv_status()
    
    def set_current(self, run_current: int, hold_current: int, hold_delay: int = 4) -> bool:
        """
        Set motor current using IHOLD_IRUN register (0x10).
        
        Current calculation:
        I_RMS = (CS+1)/32 × V_FS / (R_SENSE + 0.03Ω) × 1/√2
        Where V_FS = 0.325V (default), CS = 0-31 (5-bit value)
        
        Args:
            run_current: Run current value (0-31, 5-bit)
                        - 0-31 maps to current scale (CS)
                        - Typically 0-16 for safe operation
            hold_current: Hold current value (0-31, 5-bit)
                         - Typically 0-16, usually less than run_current
            hold_delay: Hold delay value (0-15, 4-bit)
                       - Delay before reducing to hold current after standstill
        
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

        Closes UART connection, releases GPIO pins, and ensures motor is in safe state.
        Idempotent - safe to call multiple times.
        """
        try:
            if self.uart is not None:
                # Ensure motor is stopped before closing
                self._set_safe_state()

                # Close UART connection
                self.uart.close()
                self.uart = None

            # Release STEP/DIR/ENN GPIO pins
            for pin_attr in ("_step_pin", "_dir_pin", "_enn_pin"):
                pin = getattr(self, pin_attr, None)
                if pin is not None:
                    try:
                        if hasattr(pin, "close"):
                            pin.close()
                        setattr(self, pin_attr, None)
                    except Exception as e:
                        logger.debug(f"Error closing {pin_attr}: {e}")

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
    
    def __init__(
        self,
        uart_port: str = '/dev/serial0',
        baudrate: Optional[int] = None,
        pins: Optional[Pins] = None,
        motor_config: Optional[MotorConfig] = None,
    ):
        """Initialize mock TMC2208 driver."""
        # Don't call super().__init__() - we'll initialize differently
        MotorInterface.__init__(self)
        self.uart_port = uart_port
        self.baudrate = baudrate if baudrate is not None else UART_BAUDRATE
        self.pins = pins if pins is not None else DEFAULT_PINS
        self.motor_config = motor_config if motor_config is not None else DEFAULT_MOTOR_CONFIG
        self._registers = {}
        self._current_position = 0
        self._is_moving = False
        self._is_initialized = False
    
    def __repr__(self):
        return f"MockTMC2208(uart_port='{self.uart_port}', initialized={self._is_initialized})"
    
    def initialize(self) -> bool:
        """Initialize mock TMC2208 - simulates successful initialization."""
        try:
            mc = self.motor_config
            default_current = (mc.ihold_delay << 16) | (mc.irun << 8) | mc.ihold
            # Simulate register initialization
            self._registers[self.REG_GCONF] = self.GCONF_PDN_DISABLE
            self._registers[self.REG_XACTUAL] = 0
            self._registers[self.REG_XTARGET] = 0
            self._registers[self.REG_RAMPMODE] = self.RAMPMODE_HOLD
            self._registers[self.REG_IHOLD_IRUN] = default_current

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


"""
AS5600 Magnetic Encoder Driver - I2C Implementation

This driver implements the SensorInterface using I2C communication for
AS5600 magnetic angle encoder sensors. It provides precise absolute
position sensing via magnetic field detection.

Hardware Context:
    - AS5600 Magnetic Encoder (I2C Mode)
    - I2C interface at address 0x36
    - Left Motor Sensor: I2C Bus 1 (GPIO 2/3), Address 0x36
    - Right Motor Sensor: I2C Bus 3 (GPIO 4/5), Address 0x36
    - 12-bit resolution (4096 steps per revolution)
    - Magnetic field sensing for absolute position
"""

import logging
import time
from typing import Dict, Any, Optional

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    # Fallback for systems without smbus2
    try:
        from smbus import SMBus
        i2c_msg = None
    except ImportError:
        SMBus = None
        i2c_msg = None

try:
    from ..interfaces.sensor_interface import SensorInterface, SensorStatus
except ImportError:
    # Fallback for standalone testing
    from src.interfaces.sensor_interface import SensorInterface, SensorStatus

logger = logging.getLogger(__name__)


class AS5600I2CError(Exception):
    """Exception raised for AS5600 I2C communication errors."""
    pass


class AS5600(SensorInterface):
    """
    AS5600 magnetic encoder driver with I2C communication.
    
    Implements the SensorInterface using I2C register access for angle reading.
    All hardware operations are wrapped in try/finally blocks to ensure
    proper cleanup and safe-state handling.
    """
    
    # AS5600 I2C Address
    I2C_ADDRESS = 0x36
    
    # AS5600 Register Addresses
    REG_STATUS = 0x0B        # Status register
    REG_RAW_ANGLE_H = 0x0C   # Raw angle high byte (bits 11-8)
    REG_RAW_ANGLE_L = 0x0D   # Raw angle low byte (bits 7-0)
    
    # STATUS Register Bits
    STATUS_MD = 0x08         # Bit 3: Magnet Detected (1 = detected, 0 = not detected)
    STATUS_ML = 0x10         # Bit 4: Magnet too weak
    STATUS_MH = 0x20         # Bit 5: Magnet too strong
    
    # AS5600 Constants
    RESOLUTION_BITS = 12     # 12-bit resolution
    MAX_RAW_VALUE = 0x0FFF   # Maximum raw angle value (4095)
    DEGREES_PER_REVOLUTION = 360.0
    
    def __init__(self, i2c_bus: int = 1):
        """
        Initialize AS5600 driver with I2C communication.
        
        Args:
            i2c_bus: I2C bus number (1 for default bus, 3 for right encoder bus)
        """
        super().__init__()
        self.i2c_bus_number = i2c_bus
        self.bus: Optional[SMBus] = None
        self._last_angle: Optional[float] = None
        self._last_status_byte: Optional[int] = None
        
        if SMBus is None:
            raise ImportError(
                "smbus2 library not available. Install with: pip install smbus2"
            )
    
    def __repr__(self):
        return f"AS5600(i2c_bus={self.i2c_bus_number}, initialized={self._is_initialized})"
    
    def initialize(self) -> bool:
        """
        Initialize the AS5600 driver and open I2C connection.
        
        Returns:
            bool: True if initialization successful
            
        Raises:
            AS5600I2CError: If initialization fails
        """
        try:
            # Open I2C bus
            self.bus = SMBus(self.i2c_bus_number)
            
            # Small delay for I2C bus to stabilize
            time.sleep(0.01)
            
            # Verify communication by reading STATUS register
            status_byte = self._read_status_register()
            if status_byte is None:
                raise AS5600I2CError("Failed to communicate with AS5600")
            
            self._last_status_byte = status_byte
            self._is_initialized = True
            logger.info(f"AS5600 initialized on I2C bus {self.i2c_bus_number}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize AS5600 on bus {self.i2c_bus_number}: {e}")
            self._cleanup()
            raise AS5600I2CError(f"I2C initialization failed: {e}")
    
    def _read_status_register(self) -> Optional[int]:
        """
        Read the STATUS register (0x0B) to check sensor state.
        
        Returns:
            int: Status register byte value, or None if read failed
        """
        if not self._is_initialized or self.bus is None:
            return None
        
        try:
            status = self.bus.read_byte_data(self.I2C_ADDRESS, self.REG_STATUS)
            return status
        except Exception as e:
            logger.debug(f"I2C read error for STATUS register: {e}")
            return None
    
    def _is_magnet_detected(self) -> bool:
        """
        Check if magnet is detected by reading STATUS register MD bit.
        
        Returns:
            bool: True if magnet is detected, False otherwise
        """
        status_byte = self._read_status_register()
        if status_byte is None:
            return False
        
        self._last_status_byte = status_byte
        # MD bit (bit 3) = 1 means magnet is detected
        return bool(status_byte & self.STATUS_MD)
    
    def _read_raw_angle(self) -> Optional[int]:
        """
        Read 12-bit RAW ANGLE from registers 0x0C (high) and 0x0D (low).
        
        Register 0x0C: Bits 11-8 (high nibble)
        Register 0x0D: Bits 7-0 (full byte)
        
        Returns:
            int: Raw angle value (0-4095), or None if read failed
        """
        if not self._is_initialized or self.bus is None:
            return None
        
        try:
            # Read both bytes in a single block read for atomicity
            # Start at register 0x0C, read 2 bytes
            data = self.bus.read_i2c_block_data(self.I2C_ADDRESS, self.REG_RAW_ANGLE_H, 2)
            
            if len(data) < 2:
                logger.warning("Incomplete read from RAW ANGLE registers")
                return None
            
            # Combine high and low bytes
            # High byte: bits 11-8 (only upper 4 bits are used)
            # Low byte: bits 7-0 (all 8 bits)
            raw_angle = ((data[0] & 0x0F) << 8) | data[1]
            
            # Ensure value is within valid range (0-4095)
            raw_angle = raw_angle & self.MAX_RAW_VALUE
            
            return raw_angle
            
        except Exception as e:
            logger.debug(f"I2C read error for RAW ANGLE: {e}")
            return None
    
    def get_angle(self) -> Optional[float]:
        """
        Read the current angle from the sensor.
        
        This method:
        1. Checks STATUS register for magnet detection (MD bit)
        2. Reads RAW ANGLE from registers 0x0C and 0x0D
        3. Converts 12-bit raw value to degrees (0-360)
        
        All operations are wrapped in try/finally blocks for safety.
        
        Returns:
            float: Current angle in degrees (0.0 to 360.0), or None if read failed
        
        Raises:
            RuntimeError: If sensor is not properly initialized
        """
        if not self._is_initialized:
            raise RuntimeError("AS5600 not initialized")
        
        try:
            # Check if magnet is detected before reading angle
            if not self._is_magnet_detected():
                logger.warning("Magnet not detected - cannot read reliable angle")
                return None
            
            # Read raw angle value
            raw_angle = self._read_raw_angle()
            if raw_angle is None:
                logger.warning("Failed to read RAW ANGLE from AS5600")
                return None
            
            # Convert 12-bit raw value (0-4095) to degrees (0-360)
            # Formula: degrees = (raw_angle / 4096) * 360
            angle_degrees = (raw_angle / 4096.0) * self.DEGREES_PER_REVOLUTION
            
            # Ensure angle is in valid range [0, 360)
            if angle_degrees >= 360.0:
                angle_degrees = 0.0
            
            self._last_angle = angle_degrees
            logger.debug(f"AS5600 angle: {angle_degrees:.2f}° (raw: {raw_angle})")
            return angle_degrees
            
        except RuntimeError:
            raise
        except Exception as e:
            self._handle_communication_error(e)
            logger.error(f"Error reading angle from AS5600: {e}")
            return None
        finally:
            # Cleanup is handled at bus level, but ensure state is maintained
            pass
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get the current status and health information of the sensor.
        
        Reads the STATUS register and returns comprehensive status information.
        All operations are wrapped in try/finally blocks for safety.
        
        Returns:
            Dict[str, Any]: Dictionary containing sensor status information with keys:
                - 'status': SensorStatus enum value indicating overall health
                - 'angle': Current angle in degrees (if available)
                - 'error_code': Optional error code or message
                - 'initialized': bool indicating if sensor is initialized
                - 'connection_ok': bool indicating if communication is working
                - 'magnet_detected': bool indicating if magnet is present
                - 'magnet_too_weak': bool indicating if magnet is too weak
                - 'magnet_too_strong': bool indicating if magnet is too strong
        
        Raises:
            RuntimeError: If sensor is not properly initialized
        """
        if not self._is_initialized:
            status_info = {
                'status': SensorStatus.NOT_CONNECTED,
                'angle': None,
                'error_code': 'Sensor not initialized',
                'initialized': False,
                'connection_ok': False,
                'magnet_detected': False,
                'magnet_too_weak': False,
                'magnet_too_strong': False
            }
            return status_info
        
        try:
            # Read STATUS register
            status_byte = self._read_status_register()
            self._last_status_byte = status_byte
            
            if status_byte is None:
                # Communication failure
                return {
                    'status': SensorStatus.COMMUNICATION_FAILURE,
                    'angle': self._last_angle,
                    'error_code': 'I2C communication failed',
                    'initialized': True,
                    'connection_ok': False,
                    'magnet_detected': False,
                    'magnet_too_weak': False,
                    'magnet_too_strong': False
                }
            
            # Parse STATUS register bits
            magnet_detected = bool(status_byte & self.STATUS_MD)
            magnet_too_weak = bool(status_byte & self.STATUS_ML)
            magnet_too_strong = bool(status_byte & self.STATUS_MH)
            
            # Determine overall status
            if not magnet_detected:
                sensor_status = SensorStatus.NOT_CONNECTED
                error_code = 'Magnet not detected'
            elif magnet_too_weak:
                sensor_status = SensorStatus.ERROR
                error_code = 'Magnet too weak'
            elif magnet_too_strong:
                sensor_status = SensorStatus.ERROR
                error_code = 'Magnet too strong'
            else:
                sensor_status = SensorStatus.OK
                error_code = None
            
            # Try to read current angle if magnet is detected
            angle = None
            if magnet_detected:
                angle = self.get_angle()
            
            return {
                'status': sensor_status,
                'angle': angle,
                'error_code': error_code,
                'initialized': True,
                'connection_ok': True,
                'magnet_detected': magnet_detected,
                'magnet_too_weak': magnet_too_weak,
                'magnet_too_strong': magnet_too_strong
            }
            
        except Exception as e:
            self._handle_communication_error(e)
            return {
                'status': SensorStatus.ERROR,
                'angle': self._last_angle,
                'error_code': str(e),
                'initialized': True,
                'connection_ok': False,
                'magnet_detected': False,
                'magnet_too_weak': False,
                'magnet_too_strong': False
            }
        finally:
            # Cleanup handled at bus level
            pass
    
    def _cleanup(self) -> None:
        """
        Internal cleanup method for I2C and hardware resources.
        
        Closes I2C bus connection and resets state.
        Idempotent - safe to call multiple times.
        """
        try:
            if self.bus is not None:
                self.bus.close()
                self.bus = None
            
            self._is_initialized = False
            self._last_angle = None
            self._last_status_byte = None
            logger.debug("AS5600 cleanup completed")
            
        except Exception as e:
            logger.error(f"Error during AS5600 cleanup: {e}", exc_info=True)
    
    def _handle_communication_error(self, error: Exception) -> None:
        """
        Handle I2C communication errors gracefully.
        
        Logs the error and optionally resets communication state.
        
        Args:
            error: The exception that occurred during communication
        """
        logger.warning(f"AS5600 I2C communication error: {error}")
        # Could add retry logic or state reset here if needed


class MockAS5600(AS5600):
    """
    Mock AS5600 driver for testing without hardware.
    
    Simulates I2C register responses for testing on development machines
    without actual AS5600 hardware or I2C connections.
    """
    
    def __init__(self, i2c_bus: int = 1, simulate_magnet: bool = True):
        """
        Initialize mock AS5600 driver.
        
        Args:
            i2c_bus: I2C bus number (for compatibility)
            simulate_magnet: If True, simulates magnet being detected
        """
        # Don't call super().__init__() - we'll initialize differently
        SensorInterface.__init__(self)
        self.i2c_bus_number = i2c_bus
        self.bus = None
        self._simulate_magnet = simulate_magnet
        self._mock_angle = 0.0
        self._last_angle = None
        self._last_status_byte = None
    
    def __repr__(self):
        return f"MockAS5600(i2c_bus={self.i2c_bus_number}, initialized={self._is_initialized})"
    
    def initialize(self) -> bool:
        """Initialize mock AS5600 - simulates successful initialization."""
        try:
            self._last_status_byte = self.STATUS_MD if self._simulate_magnet else 0x00
            self._is_initialized = True
            logger.info(f"[MOCK] AS5600 initialized on I2C bus {self.i2c_bus_number}")
            return True
        except Exception as e:
            logger.error(f"[MOCK] Initialization error: {e}")
            return False
    
    def _read_status_register(self) -> Optional[int]:
        """Simulate reading STATUS register."""
        if not self._is_initialized:
            return None
        
        # Return simulated status byte
        status = self.STATUS_MD if self._simulate_magnet else 0x00
        self._last_status_byte = status
        logger.debug(f"[MOCK] STATUS register: 0x{status:02X}")
        return status
    
    def _read_raw_angle(self) -> Optional[int]:
        """Simulate reading RAW ANGLE."""
        if not self._is_initialized:
            return None
        
        # Return simulated raw angle value (0-4095)
        # Increment angle slightly to simulate rotation
        raw_angle = int((self._mock_angle / 360.0) * 4096.0) & self.MAX_RAW_VALUE
        logger.debug(f"[MOCK] RAW ANGLE: {raw_angle} (0x{raw_angle:03X})")
        return raw_angle
    
    def get_angle(self) -> Optional[float]:
        """Simulate getting angle."""
        if not self._is_initialized:
            raise RuntimeError("MockAS5600 not initialized")
        
        try:
            if not self._is_magnet_detected():
                logger.warning("[MOCK] Magnet not detected")
                return None
            
            raw_angle = self._read_raw_angle()
            if raw_angle is None:
                return None
            
            angle_degrees = (raw_angle / 4096.0) * self.DEGREES_PER_REVOLUTION
            if angle_degrees >= 360.0:
                angle_degrees = 0.0
            
            self._last_angle = angle_degrees
            logger.debug(f"[MOCK] AS5600 angle: {angle_degrees:.2f}°")
            return angle_degrees
            
        except Exception as e:
            logger.error(f"[MOCK] Error reading angle: {e}")
            return None
    
    def set_mock_angle(self, angle_degrees: float) -> None:
        """
        Set the mock angle for testing.
        
        Args:
            angle_degrees: Angle in degrees (0-360)
        """
        self._mock_angle = angle_degrees % 360.0
    
    def set_magnet_detected(self, detected: bool) -> None:
        """
        Set magnet detection state for testing.
        
        Args:
            detected: True to simulate magnet detected, False otherwise
        """
        self._simulate_magnet = detected
        self._last_status_byte = self.STATUS_MD if detected else 0x00
    
    def _cleanup(self) -> None:
        """Cleanup mock resources."""
        try:
            self._is_initialized = False
            self._last_angle = None
            self._last_status_byte = None
            logger.debug("[MOCK] MockAS5600 cleanup completed")
        except Exception as e:
            logger.error(f"[MOCK] Error during cleanup: {e}")


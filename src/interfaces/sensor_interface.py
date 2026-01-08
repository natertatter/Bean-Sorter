"""
Sensor Interface - Abstract Base Class for Position/Angle Sensors

This interface defines the standard contract for sensors in the HAL.
All sensor implementations (e.g., AS5600 encoder) must conform to this interface.

Hardware Context:
    - AS5600 Magnetic Encoder
    - I2C interface
    - Left Motor Sensor: I2C Bus 1 (GPIO 2/3), Address 0x36
    - Right Motor Sensor: I2C Bus 3 (GPIO 4/5), Address 0x36
    - 12-bit resolution (4096 steps per revolution)
    - Magnetic field sensing for absolute position
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class SensorStatus(Enum):
    """Status enumeration for sensor health and operation state."""
    OK = "ok"
    ERROR = "error"
    NOT_CONNECTED = "not_connected"
    COMMUNICATION_FAILURE = "communication_failure"
    OUT_OF_RANGE = "out_of_range"
    UNKNOWN = "unknown"


class SensorInterface(ABC):
    """
    Abstract base class defining the interface for position/angle sensors.
    
    This HAL interface ensures all sensor drivers implement a consistent
    API, allowing controllers to work with any sensor implementation.
    
    All hardware operations (I2C communication) are wrapped in try/finally
    blocks to ensure proper resource cleanup and safe-state handling in
    case of errors.
    """
    
    def __init__(self):
        """Initialize the sensor interface."""
        self._is_initialized = False
    
    @abstractmethod
    def get_angle(self) -> Optional[float]:
        """
        Read the current angle from the sensor.
        
        This method must wrap all hardware operations (I2C reads) in
        try/finally blocks to ensure proper cleanup even if communication
        fails.
        
        Returns:
            float: Current angle in degrees (0.0 to 360.0), or None if read failed
        
        Raises:
            RuntimeError: If sensor is not properly initialized
            IOError: If I2C communication fails (should return None after cleanup)
        
        Note:
            - For AS5600: Returns angle based on 12-bit resolution
            - Angle is absolute position within one rotation
            - None indicates sensor read failure or invalid data
        """
        pass
    
    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """
        Get the current status and health information of the sensor.
        
        This method must wrap all hardware operations in try/finally blocks
        to ensure proper cleanup even if status check fails.
        
        Returns:
            Dict[str, Any]: Dictionary containing sensor status information with keys:
                - 'status': SensorStatus enum value indicating overall health
                - 'angle': Current angle in degrees (if available)
                - 'error_code': Optional error code or message
                - 'initialized': bool indicating if sensor is initialized
                - 'connection_ok': bool indicating if communication is working
        
        Raises:
            RuntimeError: If status check fails critically (should still return dict)
        
        Example:
            {
                'status': SensorStatus.OK,
                'angle': 180.5,
                'error_code': None,
                'initialized': True,
                'connection_ok': True
            }
        """
        pass
    
    @abstractmethod
    def _cleanup(self) -> None:
        """
        Internal cleanup method for I2C and hardware resources.
        
        This method is called in finally blocks to ensure proper cleanup.
        Must be idempotent (safe to call multiple times).
        
        Implementation should:
        - Close I2C bus connections
        - Release any hardware resources
        - Reset communication state if needed
        """
        pass
    
    @abstractmethod
    def _handle_communication_error(self, error: Exception) -> None:
        """
        Handle I2C communication errors gracefully.
        
        This method should be called in exception handlers to log errors
        and potentially reset communication state.
        
        Args:
            error: The exception that occurred during communication
        """
        pass
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup on exit."""
        try:
            self._cleanup()
        except Exception as e:
            logger.error(f"Error during sensor cleanup: {e}", exc_info=True)


"""
Motor Interface - Abstract Base Class for Stepper Motor Control

This interface defines the standard contract for motor drivers in the HAL.
All motor driver implementations (e.g., TMC2208) must conform to this interface.

Hardware Context:
    - TMC2208 Stepper Motor Driver
    - STEP/DIR interface via GPIO pins
    - Left Motor: GPIO 17 (STEP), GPIO 27 (DIR)
    - Right Motor: GPIO 10 (STEP), GPIO 9 (DIR)
    - Motor power: 12V via VM pin
    - Logic level: 3.3V via VIO pin
"""

from abc import ABC, abstractmethod
from typing import Optional
import logging

logger = logging.getLogger(__name__)


class MotorInterface(ABC):
    """
    Abstract base class defining the interface for stepper motor control.
    
    This HAL interface ensures all motor drivers implement a consistent
    API, allowing controllers to work with any motor driver implementation.
    
    All hardware operations are wrapped in try/finally blocks to ensure
    proper GPIO cleanup and safe-state shutdowns in case of errors.
    """
    
    def __init__(self):
        """Initialize the motor interface."""
        self._is_initialized = False
    
    @abstractmethod
    def step(self, count: int, direction: int) -> bool:
        """
        Execute a specified number of steps in the given direction.
        
        This method must wrap all hardware operations in try/finally blocks
        to ensure GPIO pins are properly cleaned up or motors enter safe
        states even if an error occurs.
        
        Args:
            count: Number of steps to execute (must be positive)
            direction: Direction of rotation (1 for forward, -1 for reverse)
                      or (True/False for forward/reverse)
        
        Returns:
            bool: True if steps were executed successfully, False otherwise
        
        Raises:
            ValueError: If count is negative or direction is invalid
            RuntimeError: If motor is not properly initialized or hardware error occurs
        """
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """
        Stop the motor immediately and enter safe state.
        
        This method must ensure the motor enters a safe state and all GPIO
        operations are properly cleaned up. Should be callable even if the
        motor is already stopped. Must be implemented with try/finally to
        guarantee cleanup.
        
        Safe state typically means:
        - Motor coils de-energized or in holding position
        - GPIO pins set to safe states
        - Driver in known safe configuration
        
        Raises:
            RuntimeError: If hardware shutdown fails (should attempt cleanup anyway)
        """
        pass
    
    @abstractmethod
    def _cleanup(self) -> None:
        """
        Internal cleanup method for GPIO and hardware resources.
        
        This method is called in finally blocks to ensure proper cleanup.
        Must be idempotent (safe to call multiple times).
        
        Implementation should:
        - Clean up GPIO pins
        - Set driver to safe state
        - Release any hardware resources
        """
        pass
    
    @abstractmethod
    def _set_safe_state(self) -> None:
        """
        Set motor and driver to a safe operating state.
        
        This method should be called in finally blocks to ensure the motor
        enters a safe state even if an error occurs during operation.
        """
        pass
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup on exit."""
        try:
            self.stop()
        except Exception as e:
            logger.error(f"Error during motor cleanup: {e}", exc_info=True)
        finally:
            try:
                self._cleanup()
            except Exception as e:
                logger.error(f"Error during motor final cleanup: {e}", exc_info=True)


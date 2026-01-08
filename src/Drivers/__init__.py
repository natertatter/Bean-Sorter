"""
Hardware Drivers Module

This module contains concrete implementations of hardware drivers that conform
to the interfaces defined in src/interfaces/. These drivers provide low-level
communication with physical hardware components.

Drivers:
    - TMC2208: Stepper motor driver with UART control
    - MockTMC2208: Mock implementation for testing without hardware
    - AS5600: Magnetic encoder with I2C control
    - MockAS5600: Mock implementation for testing without hardware
    - IMX296Camera: Global shutter camera with threaded capture
    - MockIMX296Camera: Mock implementation for testing without hardware
"""

from .tmc2208 import TMC2208, MockTMC2208, TMC2208UARTError
from .as5600 import AS5600, MockAS5600, AS5600I2CError

# Camera driver import is optional (requires Picamera2)
try:
    from .imx296_camera import IMX296Camera, MockIMX296Camera, IMX296CameraError
    __all__ = [
        'TMC2208', 'MockTMC2208', 'TMC2208UARTError',
        'AS5600', 'MockAS5600', 'AS5600I2CError',
        'IMX296Camera', 'MockIMX296Camera', 'IMX296CameraError'
    ]
except ImportError:
    # Picamera2 not available - exclude camera drivers
    __all__ = [
        'TMC2208', 'MockTMC2208', 'TMC2208UARTError',
        'AS5600', 'MockAS5600', 'AS5600I2CError'
    ]


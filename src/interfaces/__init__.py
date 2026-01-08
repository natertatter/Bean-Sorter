"""
Hardware Abstraction Layer (HAL) Interfaces

This module provides abstract base classes that define the standard interface
for hardware components in the embedded system. Concrete implementations
should be provided in the drivers/ directory.

Interfaces:
    - MotorInterface: Standard interface for stepper motor control
    - SensorInterface: Standard interface for position/angle sensors
"""

from .motor_interface import MotorInterface
from .sensor_interface import SensorInterface

__all__ = ['MotorInterface', 'SensorInterface']


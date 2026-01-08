"""
Controllers Module

This module contains high-level control logic that coordinates multiple
drivers to achieve system-level behaviors.

Controllers:
    - MotionController: Closed-loop position control using motor and encoder
"""

from .motion_controller import MotionController, MotionControllerError, ControlState

__all__ = ['MotionController', 'MotionControllerError', 'ControlState']


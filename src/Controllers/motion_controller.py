"""
Motion Controller - Closed-Loop Position Control

This controller implements closed-loop position control by combining
TMC2208 motor drivers with AS5600 encoder feedback. Uses asyncio to
ensure motor control operations don't block the camera thread.

Architecture:
    - Reads current position from AS5600 encoder (12-bit, 0-360°)
    - Calculates steps needed to reach target angle
    - Commands TMC2208 motor to move
    - Continuously monitors and corrects position in closed loop
    - All operations are async to prevent blocking camera thread
"""

import asyncio
import logging
from typing import Optional, Tuple
from enum import Enum

try:
    from ..Drivers.tmc2208 import TMC2208, TMC2208UARTError
    from ..Drivers.as5600 import AS5600, AS5600I2CError
except ImportError:
    try:
        from ..drivers.tmc2208 import TMC2208, TMC2208UARTError
        from ..drivers.as5600 import AS5600, AS5600I2CError
    except ImportError:
        from src.Drivers.tmc2208 import TMC2208, TMC2208UARTError
        from src.Drivers.as5600 import AS5600, AS5600I2CError

logger = logging.getLogger(__name__)


class ControlState(Enum):
    """State enumeration for motion controller."""
    IDLE = "idle"
    MOVING = "moving"
    HOLDING = "holding"
    ERROR = "error"


class MotionControllerError(Exception):
    """Exception raised for motion controller errors."""
    pass


class MotionController:
    """
    Closed-loop motion controller for stepper motor with encoder feedback.
    
    Combines TMC2208 motor driver and AS5600 encoder to provide precise
    position control. Uses asyncio for non-blocking operations to prevent
    interference with camera thread.
    
    Closed-Loop Control:
        1. Read current encoder angle (AS5600, 12-bit, 0-360°)
        2. Calculate error = target_angle - current_angle
        3. Convert angle error to motor steps
        4. Command motor to move calculated steps
        5. Repeat until error is within tolerance
    """
    
    # Default configuration
    DEFAULT_STEPS_PER_REVOLUTION = 200  # Typical NEMA 17 stepper (1.8° per step)
    DEFAULT_MICROSTEPS = 16  # Typical TMC2208 microstep setting
    DEFAULT_POSITION_TOLERANCE = 0.5  # Degrees - acceptable position error
    DEFAULT_CONTROL_PERIOD = 0.01  # seconds - control loop update rate
    DEFAULT_MAX_STEPS_PER_ITERATION = 100  # Limit steps per control iteration
    
    def __init__(
        self,
        motor: TMC2208,
        encoder: AS5600,
        steps_per_revolution: int = DEFAULT_STEPS_PER_REVOLUTION,
        microsteps: int = DEFAULT_MICROSTEPS,
        position_tolerance: float = DEFAULT_POSITION_TOLERANCE,
        control_period: float = DEFAULT_CONTROL_PERIOD
    ):
        """
        Initialize motion controller.
        
        Args:
            motor: TMC2208 motor driver instance
            encoder: AS5600 encoder instance
            steps_per_revolution: Full steps per motor revolution (typically 200 for 1.8° stepper)
            microsteps: Microstep setting on motor driver (typically 16)
            position_tolerance: Acceptable position error in degrees
            control_period: Control loop update period in seconds
        """
        self.motor = motor
        self.encoder = encoder
        
        # Calculate effective steps per revolution (full steps * microsteps)
        self.steps_per_revolution = steps_per_revolution * microsteps
        
        # Encoder resolution: 12-bit = 4096 steps = 360 degrees
        self.encoder_resolution = 4096
        self.degrees_per_step = 360.0 / self.steps_per_revolution
        self.steps_per_degree = self.steps_per_revolution / 360.0
        
        # Control parameters
        self.position_tolerance = position_tolerance
        self.control_period = control_period
        self.max_steps_per_iteration = self.DEFAULT_MAX_STEPS_PER_ITERATION
        
        # State
        self.target_angle: Optional[float] = None
        self.current_angle: Optional[float] = None
        self.state = ControlState.IDLE
        self._control_task: Optional[asyncio.Task] = None
        self._stop_event: Optional[asyncio.Event] = None
        self._is_running = False
        
        # Statistics
        self._control_iterations = 0
        self._total_steps_moved = 0
        
    def __repr__(self):
        return (
            f"MotionController(state={self.state.value}, "
            f"target={self.target_angle}°, "
            f"current={self.current_angle}°)"
        )
    
    async def initialize(self) -> bool:
        """
        Initialize motor and encoder drivers.
        
        Returns:
            bool: True if initialization successful
        """
        try:
            # Initialize motor driver (wrap blocking I/O in executor)
            logger.info("Initializing motor driver...")
            motor_init = await asyncio.to_thread(self.motor.initialize)
            if not motor_init:
                raise MotionControllerError("Motor initialization failed")
            
            # Initialize encoder (wrap blocking I/O in executor)
            logger.info("Initializing encoder...")
            encoder_init = await asyncio.to_thread(self.encoder.initialize)
            if not encoder_init:
                raise MotionControllerError("Encoder initialization failed")
            
            # Read initial position
            self.current_angle = await self._read_encoder_angle()
            logger.info(
                f"MotionController initialized - "
                f"steps/rev={self.steps_per_revolution}, "
                f"initial_angle={self.current_angle}°"
            )
            
            return True
            
        except Exception as e:
            logger.error(f"MotionController initialization failed: {e}", exc_info=True)
            raise MotionControllerError(f"Initialization failed: {e}")
    
    async def _read_encoder_angle(self) -> Optional[float]:
        """
        Read current angle from encoder (async wrapper).
        
        Returns:
            float: Current angle in degrees (0-360), or None if read failed
        """
        try:
            # Wrap blocking I/O operation in thread executor
            angle = await asyncio.to_thread(self.encoder.get_angle)
            return angle
        except Exception as e:
            logger.error(f"Error reading encoder angle: {e}")
            return None
    
    def _normalize_angle(self, angle: float) -> float:
        """
        Normalize angle to range [0, 360).
        
        Args:
            angle: Input angle in degrees
        
        Returns:
            float: Normalized angle in range [0, 360)
        """
        while angle < 0:
            angle += 360.0
        while angle >= 360.0:
            angle -= 360.0
        return angle
    
    def _calculate_angle_error(self, current: float, target: float) -> float:
        """
        Calculate shortest angular error between current and target.
        
        Handles wraparound (e.g., error from 350° to 10° = 20°, not -340°).
        
        Args:
            current: Current angle in degrees
            target: Target angle in degrees
        
        Returns:
            float: Signed angular error in degrees (-180 to +180)
        """
        # Normalize angles
        current = self._normalize_angle(current)
        target = self._normalize_angle(target)
        
        # Calculate error
        error = target - current
        
        # Handle wraparound - choose shortest path
        if error > 180.0:
            error -= 360.0
        elif error < -180.0:
            error += 360.0
        
        return error
    
    def _angle_to_steps(self, angle_degrees: float) -> int:
        """
        Convert angle in degrees to motor steps.
        
        Args:
            angle_degrees: Angle in degrees
        
        Returns:
            int: Number of motor steps (signed)
        """
        steps = int(angle_degrees * self.steps_per_degree)
        return steps
    
    def _steps_to_angle(self, steps: int) -> float:
        """
        Convert motor steps to angle in degrees.
        
        Args:
            steps: Number of motor steps
        
        Returns:
            float: Angle in degrees
        """
        angle = steps * self.degrees_per_step
        return angle
    
    async def _execute_motor_steps(self, steps: int) -> bool:
        """
        Execute motor steps (async wrapper).
        
        Args:
            steps: Number of steps to execute (signed)
        
        Returns:
            bool: True if movement successful
        """
        if steps == 0:
            return True
        
        try:
            # Determine direction from sign
            direction = 1 if steps > 0 else -1
            step_count = abs(steps)
            
            # Limit steps per iteration for safety
            if step_count > self.max_steps_per_iteration:
                step_count = self.max_steps_per_iteration
                logger.warning(
                    f"Step count limited from {abs(steps)} to {step_count} "
                    f"for safety"
                )
            
            # Wrap blocking motor operation in thread executor
            success = await asyncio.to_thread(
                self.motor.step,
                step_count,
                direction
            )
            
            if success:
                self._total_steps_moved += step_count
            
            return success
            
        except Exception as e:
            logger.error(f"Error executing motor steps: {e}", exc_info=True)
            return False
    
    async def move_to_angle(self, target_angle: float, wait: bool = True) -> bool:
        """
        Move to target angle using closed-loop control.
        
        This method starts the control loop and optionally waits for
        completion. The control loop runs asynchronously and won't
        block the camera thread.
        
        Args:
            target_angle: Target angle in degrees (0-360)
            wait: If True, wait for movement to complete
        
        Returns:
            bool: True if movement successful
        """
        # Normalize target angle
        target_angle = self._normalize_angle(target_angle)
        self.target_angle = target_angle
        
        # Start control loop if not already running
        if not self._is_running:
            await self.start_control_loop()
        
        if wait:
            # Wait for position to reach target
            await self.wait_for_position()
        
        return True
    
    async def start_control_loop(self) -> None:
        """
        Start the closed-loop control task.
        
        The control loop runs asynchronously and continuously adjusts
        motor position based on encoder feedback.
        """
        if self._is_running:
            logger.warning("Control loop already running")
            return
        
        self._stop_event = asyncio.Event()
        self._control_task = asyncio.create_task(self._control_loop())
        self._is_running = True
        logger.info("Control loop started")
    
    async def stop_control_loop(self) -> None:
        """Stop the closed-loop control task."""
        if not self._is_running:
            return
        
        if self._stop_event:
            self._stop_event.set()
        
        if self._control_task:
            try:
                await asyncio.wait_for(self._control_task, timeout=2.0)
            except asyncio.TimeoutError:
                logger.warning("Control loop did not stop within timeout")
                self._control_task.cancel()
        
        self._is_running = False
        logger.info("Control loop stopped")
    
    async def _control_loop(self) -> None:
        """
        Main closed-loop control loop.
        
        Continuously:
        1. Reads encoder position
        2. Calculates error
        3. Moves motor to correct position
        4. Checks if target reached
        
        This loop runs asynchronously and yields control to prevent
        blocking the camera thread.
        """
        logger.info("Control loop started")
        
        try:
            while not (self._stop_event and self._stop_event.is_set()):
                try:
                    # Read current encoder position
                    current_angle = await self._read_encoder_angle()
                    
                    if current_angle is None:
                        logger.warning("Failed to read encoder - skipping iteration")
                        await asyncio.sleep(self.control_period)
                        continue
                    
                    self.current_angle = current_angle
                    
                    # If no target set, hold position
                    if self.target_angle is None:
                        self.state = ControlState.HOLDING
                        await asyncio.sleep(self.control_period)
                        continue
                    
                    # Calculate angle error
                    error = self._calculate_angle_error(
                        self.current_angle,
                        self.target_angle
                    )
                    
                    # Check if within tolerance
                    if abs(error) <= self.position_tolerance:
                        self.state = ControlState.HOLDING
                        self._control_iterations += 1
                        await asyncio.sleep(self.control_period)
                        continue
                    
                    # Convert angle error to steps
                    steps_needed = self._angle_to_steps(error)
                    
                    # Execute movement
                    self.state = ControlState.MOVING
                    success = await self._execute_motor_steps(steps_needed)
                    
                    if not success:
                        logger.warning("Motor step execution failed")
                        self.state = ControlState.ERROR
                    
                    self._control_iterations += 1
                    
                    # Yield control to event loop (prevents blocking camera thread)
                    await asyncio.sleep(self.control_period)
                    
                except asyncio.CancelledError:
                    logger.info("Control loop cancelled")
                    break
                except Exception as e:
                    logger.error(f"Error in control loop: {e}", exc_info=True)
                    self.state = ControlState.ERROR
                    await asyncio.sleep(self.control_period)
        
        except Exception as e:
            logger.error(f"Fatal error in control loop: {e}", exc_info=True)
            self.state = ControlState.ERROR
        finally:
            logger.info("Control loop exited")
    
    async def wait_for_position(self, timeout: Optional[float] = None) -> bool:
        """
        Wait for motor to reach target position.
        
        Args:
            timeout: Maximum time to wait in seconds (None = wait indefinitely)
        
        Returns:
            bool: True if target reached, False if timeout
        """
        start_time = asyncio.get_event_loop().time()
        
        while True:
            if self.target_angle is None:
                return True
            
            current = await self._read_encoder_angle()
            if current is None:
                await asyncio.sleep(0.1)
                continue
            
            error = abs(self._calculate_angle_error(current, self.target_angle))
            
            if error <= self.position_tolerance:
                logger.debug(f"Position reached: {current}° (target: {self.target_angle}°)")
                return True
            
            # Check timeout
            if timeout is not None:
                elapsed = asyncio.get_event_loop().time() - start_time
                if elapsed >= timeout:
                    logger.warning(f"Position wait timeout after {timeout}s")
                    return False
            
            await asyncio.sleep(self.control_period)
    
    async def stop(self) -> None:
        """Stop motor and control loop."""
        try:
            await self.stop_control_loop()
            
            # Stop motor (wrap blocking operation)
            await asyncio.to_thread(self.motor.stop)
            
            self.state = ControlState.IDLE
            self.target_angle = None
            logger.info("Motion controller stopped")
            
        except Exception as e:
            logger.error(f"Error stopping motion controller: {e}", exc_info=True)
    
    def get_status(self) -> dict:
        """
        Get current controller status.
        
        Returns:
            dict: Status information
        """
        return {
            'state': self.state.value,
            'target_angle': self.target_angle,
            'current_angle': self.current_angle,
            'error': (
                abs(self._calculate_angle_error(self.current_angle, self.target_angle))
                if (self.current_angle is not None and self.target_angle is not None)
                else None
            ),
            'is_running': self._is_running,
            'control_iterations': self._control_iterations,
            'total_steps_moved': self._total_steps_moved,
            'position_tolerance': self.position_tolerance
        }
    
    async def cleanup(self) -> None:
        """Cleanup resources and stop all operations."""
        try:
            await self.stop()
            
            # Cleanup motor (wrap blocking operation)
            await asyncio.to_thread(self.motor._cleanup)
            
            # Cleanup encoder (wrap blocking operation)
            await asyncio.to_thread(self.encoder._cleanup)
            
            logger.info("Motion controller cleanup completed")
            
        except Exception as e:
            logger.error(f"Error during cleanup: {e}", exc_info=True)
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.initialize()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit - ensures cleanup on exit."""
        try:
            await self.cleanup()
        except Exception as e:
            logger.error(f"Error during context exit: {e}", exc_info=True)


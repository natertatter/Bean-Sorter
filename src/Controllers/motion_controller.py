"""
Motion Controller - Position-Triggered Capture Loop

Implements a trigger logic loop: move motor, poll AS5600 encoder, and capture
a single frame when the angle delta exceeds InspectionConfig.TRIGGER_INTERVAL_DEG.
Frame is passed to TFLite analyzer for classification.

Architecture:
    - Initialize: motor, encoder, camera. Calibrate encoder zero.
    - Motion loop: move motor, poll get_angle(), compute delta (wrap-aware).
    - Trigger: when abs(delta) >= trigger_interval, capture frame, analyze, update last_trigger.
"""

import logging
import time
from typing import Optional, Callable

try:
    from ..Drivers.tmc2208 import TMC2208, TMC2208UARTError
    from ..Drivers.as5600 import AS5600, AS5600I2CError
    from ..Drivers.imx296_camera import IMX296Camera, IMX296CameraError
    from ..Drivers.vision_inference import TFLiteInference
    from ..config import InspectionConfig, DEFAULT_INSPECTION_CONFIG
except ImportError:
    try:
        from ..drivers.tmc2208 import TMC2208, TMC2208UARTError
        from ..drivers.as5600 import AS5600, AS5600I2CError
        from ..drivers.imx296_camera import IMX296Camera, IMX296CameraError
        from ..drivers.vision_inference import TFLiteInference
        from ..config import InspectionConfig, DEFAULT_INSPECTION_CONFIG
    except ImportError:
        from src.Drivers.tmc2208 import TMC2208, TMC2208UARTError
        from src.Drivers.as5600 import AS5600, AS5600I2CError
        from src.Drivers.imx296_camera import IMX296Camera, IMX296CameraError
        from src.Drivers.vision_inference import TFLiteInference
        from src.config import InspectionConfig, DEFAULT_INSPECTION_CONFIG

logger = logging.getLogger(__name__)


class MotionControllerError(Exception):
    """Exception raised for motion controller errors."""
    pass


def _analyze_frame_stub(frame, model_path: str) -> Optional[dict]:
    """
    Stub for TFLite frame analysis when inference engine unavailable.

    Returns:
        dict: {'label': str, 'confidence': float}, or None
    """
    logger.debug(f"TFLite stub: frame shape={frame.shape if frame is not None else None}")
    return None


def _delta_angle_wrap(current: float, last: float) -> float:
    """
    Compute signed angular difference with 0-360 wrap-around.

    Handles 359° -> 0° correctly. E.g., current=10, last=350 -> +20 (not -340).

    Args:
        current: Current angle (0-360)
        last: Last trigger angle (0-360)

    Returns:
        float: Signed delta in degrees (-180 to +180)
    """
    delta = ((current - last + 180.0) % 360.0) - 180.0
    return delta


class MotionController:
    """
    Position-triggered motion controller.

    Runs a loop that moves the motor, polls the AS5600 encoder, and triggers
    camera capture + TFLite analysis when the angle delta exceeds the configured
    trigger interval.
    """

    def __init__(
        self,
        motor: TMC2208,
        encoder: AS5600,
        camera: IMX296Camera,
        inspection_config: Optional[InspectionConfig] = None,
        inference: Optional[TFLiteInference] = None,
        analyze_fn: Optional[Callable] = None,
    ):
        """
        Initialize motion controller.

        Args:
            motor: TMC2208 motor driver
            encoder: AS5600 encoder
            camera: IMX296 camera (trigger capture mode)
            inspection_config: Trigger interval, calibration delay, model path
            inference: TFLiteInference engine. If None, created from inspection_config.model_path.
            analyze_fn: Optional override callable(frame) -> dict. Uses inference.predict if None.
        """
        self.motor = motor
        self.encoder = encoder
        self.camera = camera
        self.inspection_config = inspection_config or DEFAULT_INSPECTION_CONFIG

        # Initialize inference engine (or use provided)
        if inference is not None:
            self.inference = inference
        else:
            self.inference = TFLiteInference(model_path=self.inspection_config.model_path)

        self.analyze_fn = analyze_fn  # Override for custom analysis

        self._last_trigger_angle: Optional[float] = None
        self._trigger_count = 0
        self._running = False

    def initialize(self) -> bool:
        """
        Initialize all drivers and calibrate encoder zero.

        Returns:
            bool: True if initialization successful
        """
        try:
            logger.info("Initializing motion controller...")

            if not self.motor.initialize():
                raise MotionControllerError("Motor initialization failed")

            if not self.encoder.initialize():
                raise MotionControllerError("Encoder initialization failed")

            # Calibrate zero position
            if not self.encoder.zero_position():
                raise MotionControllerError("Encoder zero_position failed")

            # Wait during calibration (allow mechanical settling)
            time.sleep(self.inspection_config.calibration_delay)

            # Initialize camera (Standby - no continuous capture)
            if not self.camera.initialize():
                raise MotionControllerError("Camera initialization failed")

            # Set last_trigger to current position so first trigger is after interval
            angle = self.encoder.get_angle()
            if angle is not None:
                self._last_trigger_angle = angle
                logger.info(f"Initial angle (zero-relative): {angle:.2f}°")
            else:
                self._last_trigger_angle = 0.0
                logger.warning("Could not read initial angle, using 0°")

            logger.info("Motion controller initialized")
            return True

        except Exception as e:
            logger.error(f"Motion controller initialization failed: {e}", exc_info=True)
            self._cleanup()
            raise MotionControllerError(f"Initialization failed: {e}")

    def run_trigger_loop(
        self,
        move_fn: Optional[Callable[[], None]] = None,
        poll_interval_s: float = 0.01,
        max_triggers: Optional[int] = None,
    ) -> int:
        """
        Run the position-triggered capture loop.

        Args:
            move_fn: Optional callable to move the motor (e.g., step or velocity).
                     If None, motor is not moved (encoder poll only, for testing).
            poll_interval_s: Seconds between encoder polls.
            max_triggers: Stop after this many triggers (None = unlimited).

        Returns:
            int: Number of triggers executed
        """
        self._running = True
        self._trigger_count = 0
        cfg = self.inspection_config

        try:
            while self._running:
                # Move motor (if provided)
                if move_fn is not None:
                    try:
                        move_fn()
                    except Exception as e:
                        logger.warning(f"move_fn error: {e}")

                # Poll encoder
                current_angle = self.encoder.get_angle()
                if current_angle is None:
                    time.sleep(poll_interval_s)
                    continue

                # Compute delta (wrap-aware)
                last = self._last_trigger_angle
                if last is None:
                    last = current_angle
                    self._last_trigger_angle = last

                delta = _delta_angle_wrap(current_angle, last)

                # Trigger condition
                if abs(delta) >= cfg.trigger_interval_deg:
                    # Optional: pause motor for capture (depends on speed)
                    try:
                        self.motor.stop()
                    except Exception as e:
                        logger.debug(f"Motor stop during trigger: {e}")

                    # Capture single frame
                    frame = self.camera.capture_single_frame()
                    if frame is not None:
                        # Analyze with TFLite
                        if self.analyze_fn is not None:
                            result = self.analyze_fn(frame)
                        else:
                            result = self.inference.predict(frame)
                        if result:
                            logger.info(
                                f"Trigger #{self._trigger_count + 1}: angle={current_angle:.2f}° | "
                                f"Class: {result.get('label', '?')} ({result.get('confidence', 0):.2f})"
                            )
                        else:
                            logger.info(f"Trigger #{self._trigger_count + 1}: angle={current_angle:.2f}° | (inference failed)")

                    # Update last trigger angle
                    self._last_trigger_angle = current_angle
                    self._trigger_count += 1

                    if max_triggers is not None and self._trigger_count >= max_triggers:
                        logger.info(f"Reached max_triggers={max_triggers}")
                        break

                    # Resume motor if move_fn will run again
                    time.sleep(0.05)

                time.sleep(poll_interval_s)

        except KeyboardInterrupt:
            logger.info("Trigger loop interrupted by user")
        except Exception as e:
            logger.error(f"Trigger loop error: {e}", exc_info=True)
        finally:
            self._running = False
            try:
                self.motor.stop()
            except Exception as e:
                logger.debug(f"Motor stop on exit: {e}")

        return self._trigger_count

    def stop(self) -> None:
        """Signal the trigger loop to stop."""
        self._running = False

    def _cleanup(self) -> None:
        """Release all resources."""
        try:
            self._running = False
            try:
                self.motor.stop()
            except Exception as e:
                logger.debug(f"Motor stop during cleanup: {e}")
            try:
                self.motor._cleanup()
            except Exception as e:
                logger.debug(f"Motor cleanup: {e}")
            try:
                self.encoder._cleanup()
            except Exception as e:
                logger.debug(f"Encoder cleanup: {e}")
            try:
                self.camera._cleanup()
            except Exception as e:
                logger.debug(f"Camera cleanup: {e}")
            logger.info("Motion controller cleanup completed")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}", exc_info=True)

    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup."""
        try:
            self._cleanup()
        except Exception as e:
            logger.error(f"Error during context exit: {e}", exc_info=True)

"""
IMX296 Global Shutter Camera Driver - Position-Triggered Capture

This driver implements image capture for the Sony IMX296 global shutter camera
using Picamera2. The camera is kept in Standby (initialized, buffers allocated)
and captures a single frame on demand via capture_single_frame().

Hardware Context:
    - Sony IMX296 Global Shutter Camera
    - Connected via CSI port on Raspberry Pi 4
    - Fixed shutter duration to avoid AGC lag on first frame
    - Position-triggered: capture exactly when AS5600 reports target angle
"""

import logging
import time
from typing import Optional, Tuple
import numpy as np

try:
    from ..config import VisionConfig, DEFAULT_VISION_CONFIG
except ImportError:
    from src.config import VisionConfig, DEFAULT_VISION_CONFIG

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    Picamera2 = None
    PICAMERA2_AVAILABLE = False
    logging.warning("Picamera2 not available - camera functionality disabled")

logger = logging.getLogger(__name__)


class IMX296CameraError(Exception):
    """Exception raised for IMX296 camera errors."""
    pass


class IMX296Camera:
    """
    IMX296 Global Shutter Camera Driver - Standby + Trigger Capture.

    Camera is initialized and kept in Standby (buffers allocated, fixed shutter).
    capture_single_frame() captures the immediate sensor data and returns to standby.
    No continuous threading or queue - eliminates latency for position-triggered analysis.
    """

    def __init__(
        self,
        vision_config: Optional[VisionConfig] = None,
        resolution: Optional[Tuple[int, int]] = None,
        framerate: Optional[int] = None,
        shutter_duration_us: Optional[int] = None,
    ):
        """
        Initialize IMX296 camera driver.

        Args:
            vision_config: Vision settings (resolution, shutter). Defaults to DEFAULT_VISION_CONFIG.
            resolution: Override resolution (width, height).
            framerate: Override framerate in FPS.
            shutter_duration_us: Override fixed shutter in µs. Prevents AGC lag on first frame.

        Raises:
            IMX296CameraError: If Picamera2 is not available
        """
        if not PICAMERA2_AVAILABLE:
            raise IMX296CameraError(
                "Picamera2 library not available. "
                "Install with: sudo apt install -y python3-picamera2"
            )

        vc = vision_config if vision_config is not None else DEFAULT_VISION_CONFIG
        self.vision_config = vc
        self.resolution = resolution if resolution is not None else vc.resolution
        self.framerate = framerate if framerate is not None else vc.framerate
        self.shutter_duration_us = (
            shutter_duration_us if shutter_duration_us is not None else vc.shutter_speed_us
        )

        self.camera: Optional[Picamera2] = None
        self._is_initialized = False

    def __repr__(self):
        return (
            f"IMX296Camera(resolution={self.resolution}, "
            f"shutter={self.shutter_duration_us}us, "
            f"initialized={self._is_initialized})"
        )

    def initialize(self) -> bool:
        """
        Initialize the camera and put it in Standby state.

        Configures camera with fixed shutter duration to prevent AGC lag.
        Buffers are allocated; no continuous capture is started.

        Returns:
            bool: True if initialization successful

        Raises:
            IMX296CameraError: If initialization fails
        """
        if self._is_initialized:
            logger.warning("Camera already initialized")
            return True

        try:
            self.camera = Picamera2()

            config = self.camera.create_preview_configuration(
                main={
                    "size": self.resolution,
                    "format": "RGB888"
                }
            )
            self.camera.configure(config)

            # Fixed controls - prevents AGC lag on first frame in trigger mode
            controls = {
                'ExposureTime': self.shutter_duration_us,
                'AeEnable': False,
                'AwbEnable': False,
                'AwbMode': 0,
                'AnalogueGain': 1.0,
                'FrameRate': self.framerate,
            }
            self.camera.set_controls(controls)

            self.camera.start()
            time.sleep(0.5)

            self._is_initialized = True
            logger.info(
                f"IMX296 camera initialized (Standby): {self.resolution}, "
                f"shutter={self.shutter_duration_us}us"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to initialize IMX296 camera: {e}", exc_info=True)
            self._cleanup()
            raise IMX296CameraError(f"Camera initialization failed: {e}")

    def capture_single_frame(self) -> Optional[np.ndarray]:
        """
        Capture exactly one frame and return it as a NumPy array.

        The camera must be initialized (Standby). Captures immediate sensor data,
        returns the frame, and returns to standby. No queue or threading.

        Returns:
            np.ndarray: RGB image array (height, width, 3), or None if capture failed
        """
        if not self._is_initialized or self.camera is None:
            logger.error("Camera not initialized - call initialize() first")
            return None

        try:
            request = self.camera.capture_request()
            array = request.make_array("main")
            request.release()
            return array.copy()  # Copy to avoid buffer reuse issues
        except Exception as e:
            logger.error(f"Error capturing single frame: {e}", exc_info=True)
            return None

    def _cleanup(self) -> None:
        """
        Internal cleanup method for camera resources.
        Idempotent - safe to call multiple times.
        """
        try:
            if self.camera is not None:
                try:
                    self.camera.stop()
                except Exception as e:
                    logger.debug(f"Error stopping camera: {e}")
                self.camera = None

            self._is_initialized = False
            logger.debug("IMX296 camera cleanup completed")

        except Exception as e:
            logger.error(f"Error during camera cleanup: {e}", exc_info=True)

    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup on exit."""
        try:
            self._cleanup()
        except Exception as e:
            logger.error(f"Error during camera context exit: {e}", exc_info=True)


class MockIMX296Camera(IMX296Camera):
    """
    Mock IMX296 camera driver for testing without hardware.
    """

    def __init__(
        self,
        vision_config: Optional[VisionConfig] = None,
        resolution: Optional[Tuple[int, int]] = None,
        framerate: Optional[int] = None,
        shutter_duration_us: Optional[int] = None,
    ):
        """Initialize mock camera (doesn't require Picamera2)."""
        if not PICAMERA2_AVAILABLE:
            # Allow mock to work when Picamera2 not available
            pass
        vc = vision_config if vision_config is not None else DEFAULT_VISION_CONFIG
        self.vision_config = vc
        self.resolution = resolution if resolution is not None else vc.resolution
        self.framerate = framerate if framerate is not None else vc.framerate
        self.shutter_duration_us = (
            shutter_duration_us if shutter_duration_us is not None else vc.shutter_speed_us
        )
        self.camera = None
        self._is_initialized = False
        self._mock_frame_counter = 0

    def initialize(self) -> bool:
        """Initialize mock camera."""
        try:
            self._is_initialized = True
            logger.info(f"[MOCK] IMX296 camera initialized: {self.resolution}")
            return True
        except Exception as e:
            logger.error(f"[MOCK] Initialization error: {e}")
            return False

    def capture_single_frame(self) -> Optional[np.ndarray]:
        """Capture a mock single frame (test pattern)."""
        if not self._is_initialized:
            return None

        img = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
        for y in range(self.resolution[1]):
            for x in range(self.resolution[0]):
                img[y, x] = [
                    (x + self._mock_frame_counter) % 256,
                    (y + self._mock_frame_counter) % 256,
                    (x + y + self._mock_frame_counter) % 256
                ]
        self._mock_frame_counter += 1
        return img

    def _cleanup(self) -> None:
        """Cleanup mock resources."""
        try:
            self._is_initialized = False
            logger.debug("[MOCK] IMX296 camera cleanup completed")
        except Exception as e:
            logger.error(f"[MOCK] Error during cleanup: {e}")

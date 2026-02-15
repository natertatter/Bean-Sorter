"""
IMX296 Global Shutter Camera Driver - Picamera2 Implementation

This driver implements image capture for the Sony IMX296 global shutter camera
using the modern Picamera2 library. It uses a separate thread for image capture
to prevent blocking motor control signals during high-speed motion.

Hardware Context:
    - Sony IMX296 Global Shutter Camera
    - Connected via CSI port on Raspberry Pi 4
    - 60 FPS capability
    - Global shutter prevents rolling artifacts during motion
    - Fixed shutter duration to avoid AGC interference
"""

import logging
import threading
import time
from queue import Queue, Empty
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


class CameraThread(threading.Thread):
    """
    Separate thread for camera capture operations.
    
    This thread handles all camera I/O to prevent blocking motor control
    operations (PWM/STEP signals) during image capture and processing.
    """
    
    def __init__(
        self,
        camera: 'Picamera2',
        image_queue: Queue,
        shutter_duration_us: int,
        resolution: Tuple[int, int] = (1440, 1080),
        framerate: int = 60,
        stop_event: Optional[threading.Event] = None
    ):
        """
        Initialize camera thread.
        
        Args:
            camera: Picamera2 camera instance
            image_queue: Queue to store captured images
            shutter_duration_us: Fixed shutter duration in microseconds
            resolution: Camera resolution (width, height)
            framerate: Desired framerate (FPS)
            stop_event: Event to signal thread to stop
        """
        super().__init__(name="CameraThread", daemon=True)
        self.camera = camera
        self.image_queue = image_queue
        self.shutter_duration_us = shutter_duration_us
        self.resolution = resolution
        self.framerate = framerate
        self.stop_event = stop_event or threading.Event()
        self._capture_count = 0
        self._error_count = 0
        self._is_capturing = False
        
    def run(self):
        """Main thread loop - continuously captures images."""
        logger.info("CameraThread started")
        
        try:
            while not self.stop_event.is_set():
                try:
                    # Capture image from camera
                    # This is non-blocking for the main thread
                    request = self.camera.capture_request()
                    
                    # Convert to numpy array for processing
                    # Using 'main' stream which is the full resolution stream
                    array = request.make_array("main")
                    
                    # Store image data with timestamp
                    image_data = {
                        'array': array,
                        'timestamp': time.time(),
                        'frame_number': self._capture_count
                    }
                    
                    # Put in queue (non-blocking, drops oldest if full)
                    try:
                        self.image_queue.put_nowait(image_data)
                        self._capture_count += 1
                    except Exception:
                        # Queue full - drop oldest image
                        try:
                            self.image_queue.get_nowait()
                            self.image_queue.put_nowait(image_data)
                            logger.debug("Image queue full, dropped oldest frame")
                        except Empty:
                            pass
                    
                    # Release request to return buffer to camera
                    request.release()
                    
                    # Small delay to prevent excessive CPU usage
                    time.sleep(1.0 / self.framerate)
                    
                except Exception as e:
                    self._error_count += 1
                    logger.error(f"Error in CameraThread capture loop: {e}", exc_info=True)
                    time.sleep(0.1)  # Brief pause on error
                    
        except Exception as e:
            logger.critical(f"CameraThread fatal error: {e}", exc_info=True)
        finally:
            logger.info(f"CameraThread stopped (captured {self._capture_count} frames, {self._error_count} errors)")
    
    @property
    def capture_count(self) -> int:
        """Get total number of frames captured."""
        return self._capture_count
    
    @property
    def error_count(self) -> int:
        """Get total number of capture errors."""
        return self._error_count


class IMX296Camera:
    """
    IMX296 Global Shutter Camera Driver.
    
    Uses Picamera2 library with separate CameraThread to prevent image
    processing from blocking motor control operations. Features fixed
    shutter duration to avoid AGC interference during high-speed motion.
    """
    
    def __init__(
        self,
        vision_config: Optional[VisionConfig] = None,
        resolution: Optional[Tuple[int, int]] = None,
        framerate: Optional[int] = None,
        shutter_duration_us: Optional[int] = None,
        queue_size: int = 10,
    ):
        """
        Initialize IMX296 camera driver.

        Args:
            vision_config: Vision settings (resolution, shutter). Defaults to DEFAULT_VISION_CONFIG.
            resolution: Override resolution (width, height). Takes precedence over vision_config.
            framerate: Override framerate in FPS.
            shutter_duration_us: Override fixed shutter in µs. Prevents AGC interference during motion.
            queue_size: Maximum number of images to buffer in queue.

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
        self.queue_size = queue_size
        
        self.camera: Optional[Picamera2] = None
        self.camera_thread: Optional[CameraThread] = None
        self.image_queue: Optional[Queue] = None
        self.stop_event: Optional[threading.Event] = None
        self._is_initialized = False
        self._is_capturing = False
        
    def __repr__(self):
        return (
            f"IMX296Camera(resolution={self.resolution}, "
            f"framerate={self.framerate}, "
            f"initialized={self._is_initialized}, "
            f"capturing={self._is_capturing})"
        )
    
    def initialize(self) -> bool:
        """
        Initialize the camera and start capture thread.
        
        Configures camera with fixed shutter duration to prevent AGC
        interference during high-speed motion capture.
        
        Returns:
            bool: True if initialization successful
            
        Raises:
            IMX296CameraError: If initialization fails
        """
        if self._is_initialized:
            logger.warning("Camera already initialized")
            return True
        
        try:
            # Create camera instance
            self.camera = Picamera2()
            
            # Configure camera with fixed shutter settings
            # Disable AGC/AEC to prevent interference with fixed shutter
            config = self.camera.create_preview_configuration(
                main={
                    "size": self.resolution,
                    "format": "RGB888"
                }
            )
            
            self.camera.configure(config)
            
            # Set fixed controls to prevent AGC interference
            controls = {
                # Fixed exposure time in microseconds (prevents AGC from changing it)
                'ExposureTime': self.shutter_duration_us,
                
                # Disable automatic exposure control
                'AeEnable': False,
                
                # Disable automatic white balance
                'AwbEnable': False,
                'AwbMode': 0,  # Off
                
                # Disable automatic gain control
                'AnalogueGain': 1.0,  # Fixed gain
                
                # Framerate
                'FrameRate': self.framerate,
            }
            
            self.camera.set_controls(controls)
            
            # Start camera
            self.camera.start()
            
            # Small delay for camera to stabilize
            time.sleep(0.5)
            
            # Create image queue for thread-safe image access
            self.image_queue = Queue(maxsize=self.queue_size)
            
            # Create stop event for thread control
            self.stop_event = threading.Event()
            
            # Create and start camera thread
            self.camera_thread = CameraThread(
                camera=self.camera,
                image_queue=self.image_queue,
                shutter_duration_us=self.shutter_duration_us,
                resolution=self.resolution,
                framerate=self.framerate,
                stop_event=self.stop_event
            )
            
            self._is_initialized = True
            logger.info(
                f"IMX296 camera initialized: {self.resolution} @ {self.framerate}fps, "
                f"shutter={self.shutter_duration_us}us"
            )
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize IMX296 camera: {e}", exc_info=True)
            self._cleanup()
            raise IMX296CameraError(f"Camera initialization failed: {e}")
    
    def start_capture(self) -> bool:
        """
        Start continuous image capture in background thread.
        
        Returns:
            bool: True if capture started successfully
        """
        if not self._is_initialized:
            logger.error("Camera not initialized - call initialize() first")
            return False
        
        if self._is_capturing:
            logger.warning("Capture already in progress")
            return True
        
        try:
            if self.camera_thread and not self.camera_thread.is_alive():
                self.camera_thread.start()
                self._is_capturing = True
                logger.info("Image capture started")
                return True
            elif self.camera_thread and self.camera_thread.is_alive():
                self._is_capturing = True
                logger.info("Capture thread already running")
                return True
            else:
                logger.error("Camera thread not available")
                return False
                
        except Exception as e:
            logger.error(f"Failed to start capture: {e}", exc_info=True)
            return False
    
    def stop_capture(self) -> None:
        """Stop continuous image capture."""
        if not self._is_capturing:
            return
        
        try:
            if self.stop_event:
                self.stop_event.set()
            
            if self.camera_thread and self.camera_thread.is_alive():
                # Wait for thread to finish (with timeout)
                self.camera_thread.join(timeout=2.0)
                if self.camera_thread.is_alive():
                    logger.warning("CameraThread did not stop within timeout")
            
            self._is_capturing = False
            logger.info("Image capture stopped")
            
        except Exception as e:
            logger.error(f"Error stopping capture: {e}", exc_info=True)
    
    def get_latest_image(self, timeout: float = 1.0) -> Optional[dict]:
        """
        Get the latest captured image from the queue.
        
        This method is non-blocking and thread-safe. It retrieves images
        captured by the CameraThread without interfering with motor control.
        
        Args:
            timeout: Maximum time to wait for an image (seconds)
        
        Returns:
            dict: Image data with keys:
                - 'array': numpy array of image data
                - 'timestamp': Capture timestamp
                - 'frame_number': Frame sequence number
            Or None if no image available within timeout
        """
        if not self._is_initialized or self.image_queue is None:
            return None
        
        try:
            image_data = self.image_queue.get(timeout=timeout)
            return image_data
        except Empty:
            return None
        except Exception as e:
            logger.error(f"Error getting image from queue: {e}")
            return None
    
    def get_all_images(self) -> list:
        """
        Get all currently queued images (non-blocking).
        
        Returns:
            list: List of image data dictionaries
        """
        if not self._is_initialized or self.image_queue is None:
            return []
        
        images = []
        try:
            while True:
                image_data = self.image_queue.get_nowait()
                images.append(image_data)
        except Empty:
            pass
        except Exception as e:
            logger.error(f"Error retrieving images: {e}")
        
        return images
    
    def capture_single_image(self, timeout: float = 2.0) -> Optional[np.ndarray]:
        """
        Capture a single image (blocking operation).
        
        This method temporarily stops continuous capture, captures one frame,
        and can optionally resume capture. Use with caution as it may block
        motor control briefly.
        
        Args:
            timeout: Maximum time to wait for image
        
        Returns:
            numpy.ndarray: Image array, or None if capture failed
        """
        if not self._is_initialized or self.camera is None:
            logger.error("Camera not initialized")
            return None
        
        try:
            # Capture single frame directly (blocks until capture)
            request = self.camera.capture_request()
            array = request.make_array("main")
            request.release()
            return array
            
        except Exception as e:
            logger.error(f"Error capturing single image: {e}", exc_info=True)
            return None
    
    @property
    def is_capturing(self) -> bool:
        """Check if continuous capture is active."""
        return self._is_capturing and (
            self.camera_thread is not None and self.camera_thread.is_alive()
        )
    
    @property
    def capture_stats(self) -> dict:
        """Get capture statistics from camera thread."""
        if self.camera_thread is None:
            return {'capture_count': 0, 'error_count': 0}
        
        return {
            'capture_count': self.camera_thread.capture_count,
            'error_count': self.camera_thread.error_count,
            'queue_size': self.image_queue.qsize() if self.image_queue else 0
        }
    
    def _cleanup(self) -> None:
        """
        Internal cleanup method for camera resources.
        
        Stops capture thread and releases camera resources.
        Idempotent - safe to call multiple times.
        """
        try:
            # Stop capture if running
            self.stop_capture()
            
            # Stop camera
            if self.camera is not None:
                try:
                    self.camera.stop()
                except Exception as e:
                    logger.debug(f"Error stopping camera: {e}")
                self.camera = None
            
            # Clear queue
            if self.image_queue is not None:
                try:
                    while not self.image_queue.empty():
                        try:
                            self.image_queue.get_nowait()
                        except Empty:
                            break
                except Exception:
                    pass
                self.image_queue = None
            
            self._is_initialized = False
            self._is_capturing = False
            logger.debug("IMX296 camera cleanup completed")
            
        except Exception as e:
            logger.error(f"Error during camera cleanup: {e}", exc_info=True)
    
    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        self.start_capture()
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

    Simulates camera capture for testing on development machines
    without actual camera hardware or CSI connection.
    """

    def __init__(
        self,
        vision_config: Optional[VisionConfig] = None,
        resolution: Optional[Tuple[int, int]] = None,
        framerate: Optional[int] = None,
        shutter_duration_us: Optional[int] = None,
        queue_size: int = 10,
    ):
        """Initialize mock camera (doesn't require Picamera2)."""
        # Don't call super().__init__() - we'll initialize differently
        vc = vision_config if vision_config is not None else DEFAULT_VISION_CONFIG
        self.vision_config = vc
        self.resolution = resolution if resolution is not None else vc.resolution
        self.framerate = framerate if framerate is not None else vc.framerate
        self.shutter_duration_us = (
            shutter_duration_us if shutter_duration_us is not None else vc.shutter_speed_us
        )
        self.queue_size = queue_size
        
        self.camera = None
        self.camera_thread = None
        self.image_queue = None
        self.stop_event = None
        self._is_initialized = False
        self._is_capturing = False
        self._mock_frame_counter = 0
    
    def initialize(self) -> bool:
        """Initialize mock camera - simulates successful initialization."""
        try:
            self.image_queue = Queue(maxsize=self.queue_size)
            self.stop_event = threading.Event()
            self._is_initialized = True
            logger.info(
                f"[MOCK] IMX296 camera initialized: {self.resolution} @ {self.framerate}fps"
            )
            return True
        except Exception as e:
            logger.error(f"[MOCK] Initialization error: {e}")
            return False
    
    def start_capture(self) -> bool:
        """Start mock capture - generates synthetic images."""
        if not self._is_initialized:
            return False
        
        if self._is_capturing:
            return True
        
        try:
            # Create a simple thread that generates mock images
            def mock_capture_loop():
                while not self.stop_event.is_set():
                    try:
                        # Generate a simple test pattern image
                        # Create a gradient pattern with frame counter
                        img = np.zeros(
                            (self.resolution[1], self.resolution[0], 3),
                            dtype=np.uint8
                        )
                        
                        # Add some pattern based on frame number
                        frame_num = self._mock_frame_counter
                        for y in range(self.resolution[1]):
                            for x in range(self.resolution[0]):
                                img[y, x] = [
                                    (x + frame_num) % 256,
                                    (y + frame_num) % 256,
                                    (x + y + frame_num) % 256
                                ]
                        
                        image_data = {
                            'array': img,
                            'timestamp': time.time(),
                            'frame_number': self._mock_frame_counter
                        }
                        
                        try:
                            self.image_queue.put_nowait(image_data)
                            self._mock_frame_counter += 1
                        except Exception:
                            try:
                                self.image_queue.get_nowait()
                                self.image_queue.put_nowait(image_data)
                            except Empty:
                                pass
                        
                        time.sleep(1.0 / self.framerate)
                        
                    except Exception as e:
                        logger.error(f"[MOCK] Error in capture loop: {e}")
                        time.sleep(0.1)
            
            self.camera_thread = threading.Thread(
                target=mock_capture_loop,
                name="MockCameraThread",
                daemon=True
            )
            self.camera_thread.start()
            self._is_capturing = True
            logger.info("[MOCK] Image capture started")
            return True
            
        except Exception as e:
            logger.error(f"[MOCK] Failed to start capture: {e}")
            return False
    
    def capture_single_image(self, timeout: float = 2.0) -> Optional[np.ndarray]:
        """Capture a mock single image."""
        if not self._is_initialized:
            return None
        
        # Generate a simple test pattern
        img = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
        for y in range(self.resolution[1]):
            for x in range(self.resolution[0]):
                img[y, x] = [x % 256, y % 256, (x + y) % 256]
        
        return img
    
    def _cleanup(self) -> None:
        """Cleanup mock resources."""
        try:
            self.stop_capture()
            if self.image_queue is not None:
                while not self.image_queue.empty():
                    try:
                        self.image_queue.get_nowait()
                    except Empty:
                        break
                self.image_queue = None
            self._is_initialized = False
            self._is_capturing = False
            logger.debug("[MOCK] IMX296 camera cleanup completed")
        except Exception as e:
            logger.error(f"[MOCK] Error during cleanup: {e}")


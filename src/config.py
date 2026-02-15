"""
Central configuration for Sorter MVP.

All pin definitions, communication parameters, and mechanical constants
are centralized here. Use dataclasses for type safety.
"""
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple


# --- Project Paths ---
PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = PROJECT_ROOT / "models"
MODEL_PATH = MODELS_DIR / "coffee_bean_mobilenet_dynamic.tflite"
LABELS_PATH = MODELS_DIR / "labels.json"

# --- ML / TFLite ---
IMG_SIZE = 224
TFLITE_NUM_THREADS = 4


@dataclass(frozen=True)
class Pins:
    """
    GPIO pin definitions for hardware interfaces.
    All values are BCM (Broadcom) GPIO numbers.
    """
    # Motor pins (STEP/DIR mode)
    STEP: int = 5
    DIR: int = 6
    ENN: Optional[int] = None  # Optional enable pin

    # UART pins (for TMC2208 UART mode)
    TX: int = 14
    RX: int = 15

    # Sensor pins (AS5600 / I2C)
    AS5600_DIR: Optional[int] = None  # Optional direction/select GPIO
    I2C_SDA: int = 2
    I2C_SCL: int = 3


@dataclass(frozen=True)
class MotorConfig:
    """
    Motor driver configuration: currents, microstepping, physical constants.
    """
    target_rpm: int = 60
    microsteps: int = 256
    irun: int = 31       # Run current (0-31)
    ihold: int = 16      # Hold current (0-31)
    ihold_delay: int = 4  # Hold delay (0-15)
    steps_per_rev: int = 200  # NEMA 17 full steps per revolution


@dataclass(frozen=True)
class SensorConfig:
    """
    AS5600 encoder configuration: I2C bus and address.
    """
    i2c_addr: int = 0x36
    i2c_bus: int = 1


@dataclass(frozen=True)
class VisionConfig:
    """
    IMX296 camera configuration: resolution and shutter.
    Fixed shutter prevents AGC fluctuation during motion.
    """
    resolution: Tuple[int, int] = (1456, 1088)  # IMX296 native
    shutter_speed_us: int = 10000  # Fixed shutter (µs)
    framerate: int = 60


@dataclass(frozen=True)
class InspectionConfig:
    """
    Position-triggered inspection configuration.
    Used for capture-on-angle-delta and TFLite analysis.
    """
    trigger_interval_deg: float = 15.0   # Angle delta to trigger a photo
    calibration_delay: float = 1.0       # Time to wait during zeroing (seconds)
    model_path: str = ""                 # Path to TFLite model


# --- UART (Motor Control) - legacy / convenience ---
UART_BAUDRATE = 115200
UART_LEFT_DEVICE = "/dev/serial0"   # UART0, GPIO 14/15
UART_RIGHT_DEVICE = "/dev/ttyAMA1"  # UART5, GPIO 12/13

# --- Default config instances ---
DEFAULT_PINS = Pins()
DEFAULT_MOTOR_CONFIG = MotorConfig()
DEFAULT_SENSOR_CONFIG = SensorConfig()
DEFAULT_VISION_CONFIG = VisionConfig()
DEFAULT_INSPECTION_CONFIG = InspectionConfig(model_path=str(MODEL_PATH))

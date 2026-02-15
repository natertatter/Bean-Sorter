"""
TFLite Vision Inference Engine

Consumes NumPy arrays from the Global Shutter Camera and returns classification
results. Handles both float32 and quantized (int8/uint8) models dynamically.
"""

import json
import logging
from pathlib import Path
from typing import Dict, List, Optional, Any

import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    import tflite_runtime.interpreter as tflite
    TFLITE_AVAILABLE = True
except ImportError:
    try:
        import tensorflow.lite as tflite
        TFLITE_AVAILABLE = True
    except ImportError:
        tflite = None
        TFLITE_AVAILABLE = False

try:
    from ..config import TFLITE_NUM_THREADS
except ImportError:
    from src.config import TFLITE_NUM_THREADS

logger = logging.getLogger(__name__)


class TFLiteInferenceError(Exception):
    """Exception raised for TFLite inference errors."""
    pass


class TFLiteInference:
    """
    TFLite inference engine for image classification.

    Loads model and labels, infers input shape/dtype from the model,
    and provides predict() for NumPy array input.
    """

    def __init__(self, model_path: str, labels_path: Optional[str] = None):
        """
        Initialize TFLite inference engine.

        Args:
            model_path: Path to .tflite model file
            labels_path: Path to labels JSON. If None, inferred by replacing
                         .tflite with .json in model_path.
        """
        self.model_path = model_path
        self.labels_path = labels_path or self._infer_labels_path(model_path)
        self.labels: List[str] = []
        self.interpreter = None
        self._input_index: Optional[int] = None
        self._output_index: Optional[int] = None
        self._input_height: int = 224
        self._input_width: int = 224
        self._input_dtype = np.float32
        self._input_scale: float = 1.0
        self._input_zero_point: int = 0
        self._is_quantized: bool = False
        self._initialized = False

        self._load()

    def _infer_labels_path(self, model_path: str) -> str:
        """Infer labels path: try .tflite->.json, else labels.json in same dir."""
        p = Path(model_path)
        # Common: labels.json in same directory as model
        labels_in_dir = p.parent / "labels.json"
        if labels_in_dir.exists():
            return str(labels_in_dir)
        # Fallback: same stem with .json (e.g., model.json)
        if p.suffix.lower() == ".tflite":
            return str(p.with_suffix(".json"))
        return str(labels_in_dir)

    def _load(self) -> bool:
        """
        Load model and labels. Handles missing files gracefully.

        Returns:
            bool: True if load successful
        """
        if not TFLITE_AVAILABLE:
            logger.error("TFLite runtime not available. Install: pip install tflite-runtime")
            return False

        if not CV2_AVAILABLE:
            logger.error("OpenCV not available. Install: pip install opencv-python-headless")
            return False

        try:
            # Load labels
            labels_file = Path(self.labels_path)
            if not labels_file.exists():
                logger.warning(f"Labels file not found: {self.labels_path}")
                self.labels = []
            else:
                with open(labels_file, "r") as f:
                    data = json.load(f)
                self.labels = data.get("classes", data.get("labels", []))
                logger.info(f"Loaded {len(self.labels)} labels from {self.labels_path}")

            # Load model
            model_file = Path(self.model_path)
            if not model_file.exists():
                logger.error(f"Model file not found: {self.model_path}")
                return False

            self.interpreter = tflite.Interpreter(
                model_path=str(model_file),
                num_threads=TFLITE_NUM_THREADS,
            )
            self.interpreter.allocate_tensors()

            # Read input details for dynamic preprocessing
            input_details = self.interpreter.get_input_details()
            output_details = self.interpreter.get_output_details()

            if not input_details or not output_details:
                logger.error("Could not read model input/output details")
                return False

            inp = input_details[0]
            out = output_details[0]

            self._input_index = inp["index"]
            self._output_index = out["index"]

            # Input shape: typically [1, H, W, 3]
            shape = inp["shape"]
            if len(shape) >= 3:
                self._input_height = int(shape[1])
                self._input_width = int(shape[2])
            else:
                self._input_height = 224
                self._input_width = 224

            # Input dtype: float32 vs quantized (int8/uint8)
            self._input_dtype = np.dtype(inp["dtype"])
            if self._input_dtype in (np.int8, np.uint8):
                self._is_quantized = True
                self._input_scale = float(inp.get("quantization_parameters", {}).get("scales", [1.0])[0])
                self._input_zero_point = int(inp.get("quantization_parameters", {}).get("zero_points", [0])[0])
                logger.info(
                    f"Quantized model: dtype={self._input_dtype}, "
                    f"scale={self._input_scale}, zero_point={self._input_zero_point}"
                )
            else:
                self._is_quantized = False
                logger.info(f"Float model: dtype={self._input_dtype}")

            self._initialized = True
            logger.info(
                f"TFLite model loaded: {self.model_path}, "
                f"input shape=({self._input_height}, {self._input_width})"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to load TFLite model: {e}", exc_info=True)
            self._initialized = False
            return False

    def predict(self, image_array: np.ndarray) -> Optional[Dict[str, Any]]:
        """
        Run inference on a NumPy image array.

        Args:
            image_array: RGB image (H, W, 3), uint8. From camera.capture_single_frame().

        Returns:
            dict: {'label': str, 'confidence': float}, or None if inference failed
        """
        if not self._initialized or self.interpreter is None:
            logger.warning("TFLiteInference not initialized")
            return None

        if image_array is None or image_array.size == 0:
            logger.warning("Empty image array")
            return None

        try:
            # Resize: use INTER_AREA for quality downscaling (large ~1.5MP -> 224x224)
            if image_array.shape[:2] != (self._input_height, self._input_width):
                resized = cv2.resize(
                    image_array,
                    (self._input_width, self._input_height),
                    interpolation=cv2.INTER_AREA,
                )
            else:
                resized = image_array

            # Normalization based on model dtype
            if self._is_quantized:
                # Quantized: keep uint8 or cast to int8
                if self._input_dtype == np.uint8:
                    input_data = resized.astype(np.uint8)
                else:
                    # int8: typically -128..127
                    input_data = resized.astype(np.int8)
            else:
                # Float32: normalize 0-1 (common) or -1 to 1
                input_data = resized.astype(np.float32) / 255.0

            # Add batch dimension
            input_data = np.expand_dims(input_data, axis=0)

            # Inference
            self.interpreter.set_tensor(self._input_index, input_data)
            self.interpreter.invoke()
            output = self.interpreter.get_tensor(self._output_index)

            # Post-process: argmax, map to label
            scores = output[0]
            idx = int(np.argmax(scores))
            confidence = float(scores[idx])

            # Dequantize confidence if needed (quantized models output int8)
            if self._is_quantized and output.dtype in (np.int8, np.uint8):
                out_details = self.interpreter.get_output_details()[0]
                out_scale = out_details.get("quantization_parameters", {}).get("scales", [1.0])
                out_zp = out_details.get("quantization_parameters", {}).get("zero_points", [0])
                if out_scale and out_zp:
                    confidence = (confidence - out_zp[0]) * out_scale[0]

            label = self.labels[idx] if idx < len(self.labels) else f"Class_{idx}"

            return {"label": label, "confidence": confidence}

        except Exception as e:
            logger.error(f"Inference failed: {e}", exc_info=True)
            return None

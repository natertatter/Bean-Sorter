#!/usr/bin/env python3
"""
Desktop GUI: Capture -> Inference -> Result Pipeline Validation

Validates the full pipeline: IMX296 camera capture, TFLite inference, result display.
Uses tkinter for Raspberry Pi with HDMI monitor, keyboard, and mouse.

Keyboard Shortcuts:
    SPACE: Toggle Trigger Inspection (when ready) / Reset (when showing results)
    q or Esc: Safely close and release camera resources
"""

import atexit
import logging
import sys
import threading
from pathlib import Path

# Add project root for imports
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    tk = None

# Config
try:
    from src.config import (
        DEFAULT_VISION_CONFIG,
        DEFAULT_INSPECTION_CONFIG,
        MODEL_PATH,
    )
except ImportError:
    from config import DEFAULT_VISION_CONFIG, DEFAULT_INSPECTION_CONFIG, MODEL_PATH

# Drivers
try:
    from src.Drivers.imx296_camera import IMX296Camera, MockIMX296Camera, IMX296CameraError
    from src.Drivers.vision_inference import TFLiteInference
except ImportError:
    from src.drivers.imx296_camera import IMX296Camera, MockIMX296Camera, IMX296CameraError
    from src.drivers.vision_inference import TFLiteInference

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Display size for live feed and captured image
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480


def _numpy_to_photoimage(arr: np.ndarray, width: int, height: int):
    """Convert NumPy RGB array to PhotoImage for tkinter."""
    if not PIL_AVAILABLE or arr is None:
        return None
    img = Image.fromarray(arr)
    try:
        img = img.resize((width, height), Image.Resampling.LANCZOS)
    except AttributeError:
        img = img.resize((width, height), Image.LANCZOS)
    return ImageTk.PhotoImage(img)


class CameraVisionDesktopApp:
    """
    Desktop GUI for Capture -> Inference -> Result validation.

    State 1 (Standby): Live feed, "READY - Press SPACE to Inspect"
    State 2 (Inspection): Static captured image + Top 3 results, "INSPECTION COMPLETE - Press SPACE to Reset"
    """

    def __init__(self):
        self.root = tk.Tk() if tk else None
        self.root.title("Sorter MVP - Camera Vision Validation")
        self.root.geometry("1000x600")
        self.root.minsize(800, 500)

        self.camera = None
        self.inference = None
        self._camera_ready = False
        self._inference_ready = False

        # State: "standby" | "inspection"
        self._state = "standby"
        self._live_feed_active = False
        self._live_feed_job = None
        self._last_captured_image = None
        self._last_results = None

        # UI refs
        self._photo_ref = None  # Keep reference to prevent GC
        self._canvas = None
        self._results_frame = None
        self._status_label = None
        self._result_labels = []

        self._setup_ui()
        self._setup_bindings()
        self._init_hardware()
        self._register_cleanup()

    def _setup_ui(self):
        """Build the UI layout."""
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # Left: Image display
        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._canvas = tk.Canvas(left, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, bg="black")
        self._canvas.pack()

        # Right: Results panel
        right = ttk.Frame(main, width=280)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))

        ttk.Label(right, text="Inference Results", font=("", 12, "bold")).pack(anchor=tk.W)
        self._results_frame = ttk.Frame(right)
        self._results_frame.pack(fill=tk.X, pady=(5, 0))

        for i in range(3):
            lbl = ttk.Label(self._results_frame, text="—", font=("", 10))
            lbl.pack(anchor=tk.W)
            self._result_labels.append(lbl)

        # Status bar
        self._status_label = ttk.Label(main, text="Initializing...", font=("", 11))
        self._status_label.pack(pady=(10, 0))

        # Inspect button (optional - Spacebar is primary)
        ttk.Button(main, text="Inspect (or press SPACE)", command=self._on_inspect_click).pack(
            pady=(5, 0)
        )

    def _setup_bindings(self):
        """Keyboard shortcuts."""
        self.root.bind("<space>", lambda e: self._on_space())
        self.root.bind("<Return>", lambda e: self._on_space())
        self.root.bind("<Escape>", lambda e: self._on_quit())
        self.root.bind("q", lambda e: self._on_quit())
        self.root.protocol("WM_DELETE_WINDOW", self._on_quit)

    def _init_hardware(self):
        """Initialize camera and inference from config."""
        try:
            # Camera: use VisionConfig
            try:
                self.camera = IMX296Camera(
                    resolution=DEFAULT_VISION_CONFIG.resolution,
                    shutter_duration_us=DEFAULT_VISION_CONFIG.shutter_speed_us,
                )
            except IMX296CameraError:
                logger.warning("Using MockIMX296Camera (Picamera2 unavailable)")
                self.camera = MockIMX296Camera(
                    resolution=DEFAULT_VISION_CONFIG.resolution,
                    shutter_duration_us=DEFAULT_VISION_CONFIG.shutter_speed_us,
                )

            if self.camera.initialize():
                self._camera_ready = True
                logger.info("Camera initialized")
            else:
                self._status_label.config(text="Camera init failed")

            # Inference: use InspectionConfig model path
            model_path = DEFAULT_INSPECTION_CONFIG.model_path or str(MODEL_PATH)
            self.inference = TFLiteInference(model_path=model_path)
            if self.inference._initialized:
                self._inference_ready = True
                logger.info("Inference engine initialized")
            else:
                self._status_label.config(text="Inference init failed (model/labels missing)")

            if self._camera_ready:
                self._state = "standby"
                self._update_status()
                self._start_live_feed()

        except Exception as e:
            logger.error(f"Hardware init failed: {e}", exc_info=True)
            self._status_label.config(text=f"Init error: {e}")

    def _register_cleanup(self):
        """Ensure camera is released on exit."""

        def cleanup():
            self._stop_live_feed()
            if self.camera is not None:
                try:
                    self.camera._cleanup()
                    logger.info("Camera resources released")
                except Exception as e:
                    logger.debug(f"Cleanup: {e}")

        atexit.register(cleanup)

    def _start_live_feed(self):
        """Start periodic capture for live feed display."""
        if not self._camera_ready or self._state != "standby":
            return
        self._live_feed_active = True
        self._tick_live_feed()

    def _stop_live_feed(self):
        """Stop live feed updates."""
        self._live_feed_active = False
        if self._live_feed_job:
            self.root.after_cancel(self._live_feed_job)
            self._live_feed_job = None

    def _tick_live_feed(self):
        """Capture one frame and schedule next (live feed loop)."""
        if not self._live_feed_active or self._state != "standby":
            return
        try:
            frame = self.camera.capture_single_frame()
            if frame is not None:
                photo = _numpy_to_photoimage(frame, DISPLAY_WIDTH, DISPLAY_HEIGHT)
                if photo:
                    self._photo_ref = photo
                    self._canvas.delete("all")
                    self._canvas.create_image(0, 0, anchor=tk.NW, image=photo)
        except Exception as e:
            logger.debug(f"Live feed tick: {e}")
        self._live_feed_job = self.root.after(150, self._tick_live_feed)  # ~6–7 fps

    def _update_status(self):
        """Update status label based on state."""
        if self._state == "standby":
            self._status_label.config(text="READY - Press SPACE to Inspect")
        else:
            self._status_label.config(text="INSPECTION COMPLETE - Press SPACE to Reset")

    def _on_inspect_click(self):
        """Button click handler - delegates to perform_inspection."""
        self._on_space()

    def _on_space(self):
        """Spacebar: Toggle between Trigger Inspection and Reset."""
        if self._state == "standby":
            self.perform_inspection()
        else:
            self._reset_to_standby()

    def perform_inspection(self):
        """
        Trigger event: Capture -> Inference -> Display.

        Callable by both UI Button and Keyboard Shortcut.
        Runs inference in thread to avoid GUI freeze.
        """
        if not self._camera_ready:
            self._status_label.config(text="Camera not ready")
            return

        self._stop_live_feed()
        self._state = "inspection"
        self._status_label.config(text="Capturing...")

        def _do_inference():
            try:
                frame = self.camera.capture_single_frame()
                if frame is None:
                    self.root.after(0, lambda: self._status_label.config(text="Capture failed"))
                    self._reset_to_standby()
                    return

                results = None
                if self._inference_ready:
                    results = self.inference.predict_top_k(frame, k=3)

                self.root.after(
                    0,
                    lambda: self._show_inspection_result(frame, results),
                )
            except Exception as e:
                logger.error(f"Inspection error: {e}", exc_info=True)
                self.root.after(
                    0,
                    lambda: self._status_label.config(text=f"Inspection error: {e}"),
                )
                self.root.after(0, self._reset_to_standby)

        threading.Thread(target=_do_inference, daemon=True).start()

    def _show_inspection_result(self, frame: np.ndarray, results: list):
        """Update UI with captured image and inference results (main thread)."""
        self._last_captured_image = frame
        self._last_results = results or []

        photo = _numpy_to_photoimage(frame, DISPLAY_WIDTH, DISPLAY_HEIGHT)
        if photo:
            self._photo_ref = photo
            self._canvas.delete("all")
            self._canvas.create_image(0, 0, anchor=tk.NW, image=photo)

        for i, lbl in enumerate(self._result_labels):
            if i < len(self._last_results):
                r = self._last_results[i]
                lbl.config(text=f"{i + 1}. {r['label']}: {r['confidence'] * 100:.1f}%")
            else:
                lbl.config(text="—")

        self._update_status()

    def _reset_to_standby(self):
        """Reset to live feed state."""
        self._state = "standby"
        self._last_captured_image = None
        self._last_results = None
        for lbl in self._result_labels:
            lbl.config(text="—")
        self._update_status()
        self._start_live_feed()

    def _on_quit(self):
        """Safely close application and release resources."""
        self._stop_live_feed()
        try:
            if self.camera is not None:
                self.camera._cleanup()
                logger.info("Camera resources released")
        except Exception as e:
            logger.debug(f"Quit cleanup: {e}")
        self.root.quit()
        self.root.destroy()
        sys.exit(0)

    def run(self):
        """Start the application."""
        self.root.mainloop()


def main():
    if not PIL_AVAILABLE:
        print("Install PIL: pip install Pillow")
        sys.exit(1)
    if tk is None:
        print("tkinter not available")
        sys.exit(1)

    app = CameraVisionDesktopApp()
    app.run()


if __name__ == "__main__":
    main()

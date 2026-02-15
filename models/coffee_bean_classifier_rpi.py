"""
Coffee Bean Classifier - Raspberry Pi 4 Inference Script

Run this script on your Raspberry Pi 4 with the exported TFLite model.
Requires: pip install tensorflow tflite-runtime opencv-python-headless

Usage:
  python coffee_bean_classifier_rpi.py path/to/image.jpg
  python coffee_bean_classifier_rpi.py path/to/folder/
"""
import os
import sys
import json
import numpy as np

# Prefer tflite_runtime on RPi for smaller footprint
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

try:
    import cv2
except ImportError:
    print("Install: pip install opencv-python-headless")
    sys.exit(1)

# Paths - model and labels must be in same folder as this script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "coffee_bean_mobilenet_dynamic.tflite")
LABELS_PATH = os.path.join(SCRIPT_DIR, "labels.json")
IMG_SIZE = 224

def load_classifier():
    with open(LABELS_PATH) as f:
        labels = json.load(f)
    interpreter = tflite.Interpreter(model_path=MODEL_PATH, num_threads=4)
    interpreter.allocate_tensors()
    return interpreter, labels["classes"]

def classify_image(interpreter, class_names, image_path):
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    img = cv2.imread(image_path)
    if img is None:
        return None
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (IMG_SIZE, IMG_SIZE))
    img = img.astype(np.float32) / 255.0
    img = np.expand_dims(img, axis=0)
    
    interpreter.set_tensor(input_details[0]["index"], img)
    interpreter.invoke()
    preds = interpreter.get_tensor(output_details[0]["index"])[0]
    
    idx = np.argmax(preds)
    return class_names[idx], float(preds[idx])

def main():
    if len(sys.argv) < 2:
        print("Usage: python coffee_bean_classifier_rpi.py <image_or_folder>")
        sys.exit(1)
    
    interpreter, class_names = load_classifier()
    path = sys.argv[1]
    
    if os.path.isfile(path):
        result = classify_image(interpreter, class_names, path)
        if result:
            print(f"{path}: {result[0]} ({result[1]:.2f})")
    else:
        for f in os.listdir(path):
            if f.lower().endswith((".jpg", ".jpeg", ".png")):
                full = os.path.join(path, f)
                result = classify_image(interpreter, class_names, full)
                if result:
                    print(f"{f}: {result[0]} ({result[1]:.2f})")

if __name__ == "__main__":
    main()

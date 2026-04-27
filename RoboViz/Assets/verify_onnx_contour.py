"""
Verify ONNX Contour Model with OpenCV DNN.

Tests the exported ONNX model using OpenCV's DNN module as an alternative
to ONNX Runtime. This is useful for deployment scenarios where OpenCV
is preferred over ONNX Runtime.

Usage:
    python onnx_export/verify_onnx_contour.py
    python onnx_export/verify_onnx_contour.py --image path/to/test/image.bmp
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE = SCRIPT_DIR.parent
ONNX_PATH = SCRIPT_DIR / "yolo11n_seg_contour.onnx"

# Model parameters
INPUT_SIZE = (640, 512)  # W, H
CATEGORIES = {0: 'cut', 1: 'deformation', 2: 'hole', 3: 'tear'}


def preprocess_image(image_path: str):
    """Preprocess image for ONNX inference."""
    # Load image
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Could not load image: {image_path}")

    original_shape = img.shape[:2]  # H, W

    # Resize to model input size
    img_resized = cv2.resize(img, INPUT_SIZE, interpolation=cv2.INTER_LINEAR)

    # Convert BGR→RGB, normalize to 0-1, HWC→CHW, add batch dimension
    img_rgb = img_resized[:, :, ::-1]  # BGR→RGB
    img_normalized = img_rgb.astype(np.float32) / 255.0
    img_chw = np.transpose(img_normalized, (2, 0, 1))  # HWC→CHW
    img_batch = np.expand_dims(img_chw, axis=0)  # Add batch dimension

    return img_batch, original_shape


def postprocess_detections(outputs, conf_threshold=0.3):
    """Post-process ONNX outputs to extract detections."""
    # outputs[0]: [1, 40, 6720] - boxes, scores, classes
    # outputs[1]: [1, 32, 128, 160] - mask coefficients

    det_output = outputs[0][0]  # Remove batch dimension: [40, 6720]
    mask_output = outputs[1][0] if len(outputs) > 1 else None  # [32, 128, 160]

    # Extract boxes (first 4 rows), scores (5th row), classes (remaining rows)
    boxes = det_output[:4].T  # [6720, 4] - x1,y1,x2,y2
    scores = det_output[4].T  # [6720] - confidence scores
    class_probs = det_output[5:].T  # [6720, 35] - class probabilities

    # Get class IDs and confidence scores
    class_ids = np.argmax(class_probs, axis=1)
    confidences = scores * np.max(class_probs, axis=1)

    # Filter by confidence threshold
    valid_detections = confidences > conf_threshold

    if not np.any(valid_detections):
        return [], []

    filtered_boxes = boxes[valid_detections]
    filtered_confidences = confidences[valid_detections]
    filtered_class_ids = class_ids[valid_detections]

    # Apply NMS
    indices = cv2.dnn.NMSBoxes(
        filtered_boxes.tolist(),
        filtered_confidences.tolist(),
        conf_threshold,
        0.45  # NMS threshold
    )

    detections = []
    if len(indices) > 0:
        indices = indices.flatten()
        for i in indices:
            box = filtered_boxes[i]
            conf = filtered_confidences[i]
            cls_id = filtered_class_ids[i]
            cls_name = CATEGORIES.get(cls_id, 'unknown')

            # Scale boxes back to original image coordinates
            x1, y1, x2, y2 = box
            width = x2 - x1
            height = y2 - y1

            detections.append({
                'class': cls_name,
                'confidence': conf,
                'box': [x1, y1, x2, y2],
                'width': width,
                'height': height
            })

    return detections, mask_output


def main():
    parser = argparse.ArgumentParser(description="Verify ONNX Contour Model with OpenCV DNN")
    parser.add_argument("--image", type=str, default="",
                        help="Test image path (auto-detect if omitted)")
    parser.add_argument("--conf", type=float, default=0.3,
                        help="Confidence threshold")
    args = parser.parse_args()

    if not ONNX_PATH.exists():
        print(f"ERROR: ONNX model not found at {ONNX_PATH}")
        print("Run export_yolo_contour_onnx.py first!")
        return

    # Find test image if not provided
    test_image = args.image
    if not test_image:
        # Look for validation images in contour dataset
        val_dir = WORKSPACE / "contour" / "images" / "val"
        if val_dir.exists():
            images = list(val_dir.glob("*.png"))
            if images:
                test_image = str(images[0])

    if not test_image or not Path(test_image).exists():
        print("ERROR: No test image found. Provide --image path/to/image")
        return

    print("=" * 70)
    print("ONNX CONTOUR MODEL VERIFICATION (OpenCV DNN)")
    print("=" * 70)
    print(f"  ONNX Model : {ONNX_PATH}")
    print(f"  Test Image : {test_image}")
    print(f"  Input Size : {INPUT_SIZE}")
    print(f"  Confidence : {args.conf}")

    # Load ONNX model with OpenCV DNN
    net = cv2.dnn.readNetFromONNX(str(ONNX_PATH))

    # Check if GPU is available
    if cv2.cuda.getCudaEnabledDeviceCount() > 0:
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        backend = "CUDA"
    else:
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        backend = "CPU"

    print(f"  Backend    : OpenCV DNN ({backend})")

    # Preprocess image
    input_blob, original_shape = preprocess_image(test_image)

    # Run inference
    net.setInput(input_blob)

    # Warmup
    _ = net.forward()

    # Benchmark
    n_runs = 10
    times = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        outputs = net.forward()
        times.append((time.perf_counter() - t0) * 1000)

    times = np.array(times)
    print(f"\n  Benchmark ({backend}, {n_runs} runs):")
    print(f"    mean={times.mean():.1f}ms  min={times.min():.1f}ms  max={times.max():.1f}ms")

    # Process detections
    detections, _ = postprocess_detections(outputs, args.conf)

    print(f"\n  Detections:")
    if detections:
        for i, det in enumerate(detections, 1):
            print(f"    [{i}] {det['class']}")
            print(f"        Confidence: {det['confidence']:.3f}")
            print(f"        Box size: {det['width']:.1f} × {det['height']:.1f} px")
    else:
        print(f"    No detections above {args.conf:.0%} confidence")

    print("\n" + "=" * 70)
    print("VERIFICATION COMPLETE")
    print(f"  OpenCV DNN backend: {backend}")
    print(f"  Avg inference time: {times.mean():.1f}ms")
    print("=" * 70)


if __name__ == "__main__":
    main()
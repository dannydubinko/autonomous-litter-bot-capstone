import sys
import os
import cv2
import numpy as np

# 1. POINT TO THE FOLDER (not the file)
# If your file is in /home/pi5/librealsense/build/Release/
realsense_dir = "/home/pi5/librealsense/build/Release"
if realsense_dir not in sys.path:
    sys.path.append(realsense_dir)

# 2. NOW import should work
try:
    import pyrealsense2 as rs
except ImportError:
    print(f"Still can't find pyrealsense2 in {realsense_dir}")
    sys.exit(1)

from ultralytics import YOLO

# --- SETTINGS FOR PI 5 ---
# Use the full path so there is no guessing
MODEL_PATH = "./models/segmentation_small_openvino_model"

# Check if it exists before loading to avoid the crash
if not os.path.exists(MODEL_PATH):
    print(f"CRITICAL: {MODEL_PATH} not found. Did you run the export script?")
    sys.exit(1)

model = YOLO(MODEL_PATH)

IMG_SIZE = 640  # 960 is very heavy for Pi 5 RAM; 640 is the 'sweet spot'
# -------------------------

# Configure RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Load Model (Note: OpenVINO doesn't need .to(device))
model = YOLO(MODEL_PATH)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: continue

        frame = np.asanyarray(color_frame.get_data())
        H, W = frame.shape[:2]

        # Use OpenVINO for inference (much faster)
        results = model.predict(frame, imgsz=IMG_SIZE, conf=0.8, verbose=False)
        r = results[0]
        annotated_frame = r.plot()

        if r.masks is not None:
            for mask_data in r.masks.data:
                # Optimized mask processing for Pi 5
                mask = mask_data.numpy()
                mask_uint8 = cv2.resize((mask * 255).astype(np.uint8), (W, H), interpolation=cv2.INTER_NEAREST)
                
                contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not contours: continue

                c = max(contours, key=cv2.contourArea)
                if len(c) >= 5:
                    ellipse = cv2.fitEllipse(c)
                    cv2.ellipse(annotated_frame, ellipse, (255, 0, 0), 2)

        cv2.imshow("Pi 5 - Litter Bot Vision", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
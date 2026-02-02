import sys
import os
import cv2
import numpy as np
import time  # <--- Added for FPS calculation

# --- 1. SETUP REALSENSE PATHS (For Pi 5) ---
realsense_dir = "/home/pi5/librealsense/build/Release"
if realsense_dir not in sys.path:
    sys.path.append(realsense_dir)

try:
    import pyrealsense2 as rs
except ImportError:
    print(f"❌ Error: Could not find pyrealsense2 in {realsense_dir}")
    sys.exit(1)

from ultralytics import YOLO

# --- 2. CONFIGURATION ---
MODEL_PATH = "./models/segmentation_small_openvino_model"

if not os.path.exists(MODEL_PATH):
    print(f"❌ CRITICAL: Model folder not found at: {MODEL_PATH}")
    sys.exit(1)

print("Loading OpenVINO model...")
model = YOLO(MODEL_PATH, task="segment")

# --- 3. REALSENSE PIPELINE SETUP ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

print("Starting RealSense stream... (Wait for auto-exposure)")
pipeline.start(config)

# --- FPS VARS ---
prev_frame_time = 0
new_frame_time = 0

try:
    while True:
        # Start timer for this frame
        new_frame_time = time.time()

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        H, W = frame.shape[:2]

        results = model.predict(frame, imgsz=640, conf=0.75, verbose=False)
        r = results[0]

        try:
            annotated_frame = r.plot()
        except KeyError:
            annotated_frame = frame.copy()

        if r.masks is not None:
            lowres_masks = r.masks.data.cpu().numpy()
            full_masks = []
            for m in lowres_masks:
                up = cv2.resize(m, (W, H), interpolation=cv2.INTER_NEAREST)
                full_masks.append(up)
            full_masks = np.array(full_masks)

            for mask in full_masks:
                mask_uint8 = (mask * 255).astype(np.uint8)
                _, binary = cv2.threshold(mask_uint8, 128, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if not contours: continue
                c = max(contours, key=cv2.contourArea)
                if len(c) < 5: continue

                ellipse = cv2.fitEllipse(c)
                (cx, cy), (MA, ma), angle = ellipse

                cv2.ellipse(annotated_frame, ellipse, (255, 0, 0), 2)

                theta = np.deg2rad(angle)
                dx_major = (MA / 2) * np.cos(theta)
                dy_major = (MA / 2) * np.sin(theta)
                
                # Draw Lines
                cv2.line(annotated_frame, 
                         (int(cx - dx_major), int(cy - dy_major)), 
                         (int(cx + dx_major), int(cy + dy_major)), (0, 0, 255), 2)
                
                dx_minor = (ma / 2) * np.sin(theta)
                dy_minor = (ma / 2) * np.cos(theta)
                cv2.line(annotated_frame, 
                         (int(cx - dx_minor), int(cy + dy_minor)), 
                         (int(cx + dx_minor), int(cy - dy_minor)), (0, 255, 0), 2)

                angle_major = angle
                if MA < ma: angle_major += 90
                angle_major = abs((angle_major + 90) % 180 - 90)

                cv2.putText(annotated_frame, f"{angle_major:.1f} deg", (int(cx), int(cy)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # --- CALCULATE AND DRAW FPS ---
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        
        # Draw FPS in the top-left corner
        cv2.putText(annotated_frame, f"FPS: {int(fps)}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        # -----------------------------

        cv2.imshow("Pi 5: YOLOv8 + RealSense + OpenVINO", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
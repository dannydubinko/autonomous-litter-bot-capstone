import sys
import os
import json
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

# --- Import RealSense ---
realsense_dir = "/home/pi5/librealsense/build/Release"
if realsense_dir not in sys.path:
    sys.path.append(realsense_dir)

try:
    import pyrealsense2 as rs
except ImportError:
    print(f"❌ Error: Could not find pyrealsense2 in {realsense_dir}")
    sys.exit(1)

from ultralytics import YOLO

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.declare_parameter('enable_vis', True)
        self.bridge = CvBridge()

        # 1. Path & Model Loading
        package_name = 'autonomous_litter_bot_package'
        package_share = get_package_share_directory(package_name)
        self.model_path = os.path.join(package_share, 'models', 'segmentation_small_openvino_model')
        
        self.get_logger().info(f"Loading Model: {self.model_path}")
        self.model = YOLO(self.model_path, task="segment")

        # 2. Publishers
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.debug_pub_ = self.create_publisher(Image, 'debug_image', 10) 

        # 3. Setup RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        self.align = rs.align(rs.stream.color)

        profile = self.pipeline.start(self.config)
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_stream.get_intrinsics()
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("✅ Node Initialized and Running.")

    def timer_callback(self):
        show_debug = self.get_parameter('enable_vis').value

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        except RuntimeError:
            return

        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame: return

        frame = np.asanyarray(color_frame.get_data())
        H, W = frame.shape[:2]
        debug_frame = frame.copy() if show_debug else None

        results = self.model.predict(frame, imgsz=640, conf=0.75, verbose=False)
        r = results[0]
        detected_list = []

        if r.masks is not None:
            for mask_points in r.masks.xy:
                c = mask_points.astype(np.int32)
                if len(c) < 5: continue 

                # --- 1. Geometry Calculation ---
                ellipse = cv2.fitEllipse(c)
                (cx, cy), (MA, ma), angle = ellipse
                
                # Convert to int and clamp to image boundaries
                cx_int = max(0, min(int(cx), W - 1))
                cy_int = max(0, min(int(cy), H - 1))

                # --- 2. Distance & Real Size ---
                distance_meters = depth_frame.get_distance(cx_int, cy_int)
                if distance_meters <= 0: continue 

                # Real world size in cm
                real_width_cm = (ma * distance_meters / self.intrinsics.fx) * 100
                real_length_cm = (MA * distance_meters / self.intrinsics.fy) * 100
                
                # 1. OpenCV 'angle' is already 0 at vertical, increasing clockwise.
                # 2. We want to wrap it so that 0-90 is the right side (negative) 
                #    and 90-180 is the left side (positive).

                if angle > 90:
                    angle_major = angle - 180
                else:
                    angle_major = angle

                # 3. Final Flip: Since OpenCV increases clockwise but you want 
                #    positive to be the left tilt and negative to be the right tilt:
                angle_major = -angle_major

                # --- 4. Visualization ---
                if show_debug:
                    cv2.drawContours(debug_frame, [c], -1, (0, 255, 0), 2)
                    cv2.circle(debug_frame, (cx_int, cy_int), 5, (0, 0, 255), -1)
                    
                    labels = [
                        f"Dist: {distance_meters:.2f}m",
                        f"Size: {real_width_cm:.1f}x{real_length_cm:.1f}cm",
                        f"Ang: {angle_major:.1f}deg",
                    ]
                    
                    text_x = min(cx_int + 15, W - 160)
                    text_y = cy_int
                    for line in labels:
                        cv2.putText(debug_frame, line, (text_x, text_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                        text_y += 18

                # --- 5. Data Collection ---
                obj_data = {
                    "x_pos": float(cx),
                    "y_pos": float(cy),
                    "dist_meters": float(distance_meters),
                    "width_cm": float(real_width_cm),
                    "length_cm": float(real_length_cm),
                    "angle": float(angle_major)
                }
                detected_list.append(obj_data)

        # Publish JSON string
        self.publisher_.publish(String(data=json.dumps(detected_list)))

        # Publish Debug Image
        if show_debug and debug_frame is not None:
            self.debug_pub_.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
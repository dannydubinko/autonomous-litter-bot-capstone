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

# --- Import RealSense ---
realsense_dir = "/home/pi5/librealsense/build/Release"
if realsense_dir not in sys.path:
    sys.path.append(realsense_dir)

try:
    import pyrealsense2 as rs
except ImportError:
    print(f"❌ Error: Could not find pyrealsense2 in {realsense_dir}")
    sys.exit(1)

class OrangeDetector(Node):
    def __init__(self):
        super().__init__('orange_detector_node')
        
        self.declare_parameter('enable_vis', True)
        self.bridge = CvBridge()

        # --- Constants (Tuned for Orange) ---
        self.LOWER_ORANGE = np.array([5, 100, 100]) # Increased saturation/value for robustness
        self.UPPER_ORANGE = np.array([25, 255, 255])
        self.MIN_CONTOUR_AREA = 500
        self.KERNEL = np.ones((5, 5), np.uint8)

        # --- Publishers ---
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.debug_pub_ = self.create_publisher(Image, 'orange_detector/debug_image', 10) 

        # --- Setup RealSense ---
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        self.align = rs.align(rs.stream.color)

        profile = self.pipeline.start(self.config)
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_stream.get_intrinsics()
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("✅ Orange Detection Node Initialized.")

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

        # --- 1. Color Masking & Filtering ---
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, self.LOWER_ORANGE, self.UPPER_ORANGE)
        
        # Morphological operations to clean noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.KERNEL, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.KERNEL, iterations=1)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_list = []
        best_cnt = None
        max_area = 0

        # Find the single largest valid contour (Confidence Proxy)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.MIN_CONTOUR_AREA and area > max_area:
                max_area = area
                best_cnt = cnt

        if best_cnt is not None and len(best_cnt) >= 5: # Need 5 points for ellipse fit
            # --- 2. Geometry Calculation ---
            ellipse = cv2.fitEllipse(best_cnt)
            (cx, cy), (MA, ma), angle = ellipse
            
            cx_int = max(0, min(int(cx), W - 1))
            cy_int = max(0, min(int(cy), H - 1))

            # --- 3. Distance & Real Size ---
            distance_meters = depth_frame.get_distance(cx_int, cy_int)
            
            if distance_meters > 0:
                # Real world size in cm
                real_width_cm = (ma * distance_meters / self.intrinsics.fx) * 100
                real_length_cm = (MA * distance_meters / self.intrinsics.fy) * 100
                
                # Normalize angle (consistent with your trash_detection logic)
                if angle > 90:
                    angle_major = angle - 180
                else:
                    angle_major = angle
                angle_major = -angle_major

                # --- 4. Visualization ---
                if show_debug:
                    cv2.ellipse(debug_frame, ellipse, (0, 255, 0), 2)
                    cv2.circle(debug_frame, (cx_int, cy_int), 5, (0, 0, 255), -1)
                    
                    label = f"Dist: {distance_meters:.2f}m | Ang: {angle_major:.1f}deg"
                    cv2.putText(debug_frame, label, (cx_int - 20, cy_int - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

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
    node = OrangeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
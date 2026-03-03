import math
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseTracker(Node):
    def __init__(self):
        super().__init__('pose_tracker')
        self.latest_pose = None
        
        # Subscribe to the topic directly
        self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.pose_callback, 
            10
        )

    def pose_callback(self, msg):
        # Constantly update the background variable
        self.latest_pose = msg

    def capture_my_position_now(self):
        """Call this normal Python method whenever you want the position."""
        if self.latest_pose is None:
            self.get_logger().warn("Waiting for amcl_pose...")
            return None
            
        pos = self.latest_pose.pose.pose.position
        ori = self.latest_pose.pose.pose.orientation
        theta = math.degrees(2.0 * math.atan2(ori.z, ori.w))
        
        return pos.x, pos.y, theta

# --- Example Usage inside the same script ---
# tracker.capture_my_position_now()
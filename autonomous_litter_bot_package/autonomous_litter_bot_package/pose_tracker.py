import rclpy
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
        
        # Use our new built-in conversion method!
        roll, pitch, yaw = self.quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)
        
        # Convert yaw from radians back to degrees for human readability
        theta_deg = math.degrees(yaw)
        
        return pos.x, pos.y, theta_deg

    # ------------------------------------------------------------------ #
    #  Math Utilities (Inheritable by other classes)                     #
    # ------------------------------------------------------------------ #
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Converts Euler angles (in radians) to a Quaternion.
        For 2D Nav2 ground robots, roll and pitch are usually 0.0.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q_x = sr * cp * cy - cr * sp * sy
        q_y = cr * sp * cy + sr * cp * sy
        q_z = cr * cp * sy - sr * sp * cy
        q_w = cr * cp * cy + sr * sp * sy

        return q_x, q_y, q_z, q_w

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """
        Converts a Quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw in radians).
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            # Use 90 degrees if out of bounds (gimbal lock prevention)
            pitch = math.copysign(math.pi / 2, sinp) 
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
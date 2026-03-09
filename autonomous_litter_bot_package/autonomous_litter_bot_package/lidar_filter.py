import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

# Import tf2 modules
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class BaseLinkScanFilter(Node):
    def __init__(self):
        super().__init__('base_link_scan_filter')
        
        # 1. Set up the TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to raw scan
        self.subscription = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        # Publish filtered scan
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        # 2. Define bounding box relative to 'base_link' (in meters)
        # E.g., The robot is 1.0m long and 0.6m wide, centered at base_link
        self.x_min = -0.26  
        self.x_max = 0.26
        self.y_min = -0.1524
        self.y_max = 0.1524

    def scan_callback(self, msg):
        # 3. Look up the transform from the LiDAR frame to base_link
        try:
            # We want to transform points FROM the laser frame TO base_link
            t = self.tf_buffer.lookup_transform(
                'base_link', 
                msg.header.frame_id, 
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform {msg.header.frame_id} to base_link: {e}')
            return # Skip filtering this scan until TF is ready

        # Extract translation
        dx = t.transform.translation.x
        dy = t.transform.translation.y

        # Extract yaw rotation from the quaternion
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Setup the new message
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header # Keep the original frame_id!
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.intensities = msg.intensities 

        new_ranges = list(msg.ranges)

        for i, r in enumerate(msg.ranges):
            # Sanitize raw data
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                new_ranges[i] = 0.0
                continue

            # Calculate the angle for this specific ray in the LiDAR frame
            angle = msg.angle_min + (i * msg.angle_increment)

            # Polar to Cartesian in the LiDAR frame
            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)

            # Transform the Cartesian point to the base_link frame
            x_base = x_laser * math.cos(yaw) - y_laser * math.sin(yaw) + dx
            y_base = x_laser * math.sin(yaw) + y_laser * math.cos(yaw) + dy

            # Check if the transformed point falls inside the base_link bounding box
            if self.x_min <= x_base <= self.x_max and self.y_min <= y_base <= self.y_max:
                new_ranges[i] = 0.0

        filtered_msg.ranges = new_ranges
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    scan_filter = BaseLinkScanFilter()
    
    try:
        rclpy.spin(scan_filter)
    except KeyboardInterrupt:
        pass
        
    scan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()